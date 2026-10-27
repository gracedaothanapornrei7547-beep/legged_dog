#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
debug_console.py —— 决策机联调 · 交互式调试台（一个程序搞定 OPERATIONS.md §三）

把导航测试、X5 串口测试、底盘控制封装成一个菜单程序：
  运行 → 选 [1]导航 / [2]X5 / [3]底盘 → 按提示继续输入。
  · 导航：先列出路点文件里已打的点名，输入一个或多个点名 → 调 run_route 走。
  · X5  ：按索引表输入序号 → 发单条串口命令、打印 X5 回帧；
           收到移动帧(微调)时 → 自动向底盘发 Twist 完成微调动作。
  · 底盘：手动输入 Vx/Vy/ω 发 /cmd_vel，或持续发直到回车停。

前置（各自单独终端，本程序不负责起）：
  · FSM + 底盘 + 定位：按 OPERATIONS.md §2.3 终端1、终端2 起好。
  · 导航测试需在 source 了导航环境的终端运行本程序（要能调到 run_route）。
  · 底盘控制需 source ROS 环境（rclpy 可用）。

用法：
  python3 debug_console.py
  python3 debug_console.py --port /dev/ttyUSB0            # 指定 X5 串口
  python3 debug_console.py --cmd-vel-topic /cmd_vel_raw   # 指定底盘控制话题
  python3 debug_console.py --nudge-v 0.05 --nudge-w 0.15  # 微调速度
"""
import argparse
import os
import queue
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

WAYPOINTS_FILE = Path(os.path.expanduser("~/.simple_nav_fsm_waypoints.yaml"))

# ── X5 单条命令索引表（对齐 OPERATIONS.md §B.8 / §五）────────────
#   序号 → (名称, s1, s2, 超时秒)
X5_COMMANDS = [
    ("初始化",       0x01, 0x00, 60),
    ("数字识别",     0x02, 0x00, 60),
    ("扫描8箱",      0x03, 0x00, 60),
    ("抓取(左右一起)", 0x04, 0x01, 60),
    ("放置-右托盘",   0x05, 0x02, 60),
    ("放置-左托盘",   0x05, 0x01, 60),
]

S3_NAMES = {1: "运行中", 2: "完成", 3: "失败", 4: "需微调"}
DIR = {0: "·", 1: "+", 2: "-"}

# ── 帧解析器（直接复用 task_decision/serial_protocol.py）────
STATE_HEAD, STATE_TAIL = 0xAB, 0xBA
DIGIT_HEAD, DIGIT_TAIL = 0xCD, 0xDC
MOVE_HEAD,  MOVE_TAIL  = 0xEF, 0xFE
LEN_STATE_REPLY, LEN_DIGIT, LEN_MOVE = 5, 11, 14


class _FrameParser:
    """逐字节增量解析 A→B 三种帧，帧尾不匹配只丢 1 字节重同步。"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        if data:
            self.buf.extend(data)
        out = []
        while True:
            frame, consumed = self._try_one()
            if consumed == 0:
                break
            if frame is not None:
                out.append(frame)
        return out

    def _try_one(self):
        buf = self.buf
        if not buf:
            return None, 0
        h = buf[0]
        if h == STATE_HEAD:
            need, tail = LEN_STATE_REPLY, STATE_TAIL
        elif h == DIGIT_HEAD:
            need, tail = LEN_DIGIT, DIGIT_TAIL
        elif h == MOVE_HEAD:
            need, tail = LEN_MOVE, MOVE_TAIL
        else:
            del buf[0]
            return None, 1
        if len(buf) < need:
            return None, 0
        if buf[need - 1] != tail:
            del buf[0]
            return None, 1
        raw = bytes(buf[:need])
        del buf[:need]
        if h == STATE_HEAD:
            return ('state', raw[1], raw[2], raw[3]), need
        if h == DIGIT_HEAD:
            return ('digit', raw[1], list(raw[2:10])), need
        if h == MOVE_HEAD:
            return ('move', *struct.unpack('<fff', raw[1:13])), need
        return None, need


# ══════════════════════════════════════════════════════════════
# 底盘控制器 —— 通过 ROS 2 发 Twist 到底盘
# ══════════════════════════════════════════════════════════════
class ChassisController:
    """ROS 2 底盘速度发布器。

    用 rclpy 发 geometry_msgs/Twist 到指定话题（默认 /cmd_vel，
    即 cmd_bridge 直接订阅的话题，无需 cmd_vel_guard 转发）。
    支持 oneshot（发一次）和 timed（发 dur 秒后自动停）。

    底盘指令链路:
      /cmd_vel → cmd_bridge → /dev/shm/legged_nav_cmd → nav_quad → CAN → 电机
    """

    def __init__(self, topic: str = "/cmd_vel"):
        self._topic = topic
        self._node = None
        self._pub = None
        self._executor = None
        self._spin_thread = None
        self._Twist = None
        self._ready = False
        self._init_error = ""
        self._init_ros()

    def _init_ros(self):
        """初始化 ROS 节点和 publisher。"""
        # 1. 导入
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            self._Twist = Twist
            self._rclpy = rclpy
        except ImportError as e:
            self._init_error = f"ROS 2 未安装或未 source: {e}"
            return

        # 2. 初始化 rclpy（如果还没初始化的话）
        if not self._rclpy.ok():
            try:
                self._rclpy.init(args=[])
            except Exception as e:
                self._init_error = f"rclpy.init 失败: {e}"
                return
        else:
            pass  # 已在外部初始化（如 mission_director 的 executor）

        # 3. 创建节点
        try:
            self._node = self._rclpy.create_node("debug_console_chassis")
        except Exception as e:
            self._init_error = f"create_node 失败: {e}"
            return

        # 4. 创建 publisher
        self._pub = self._node.create_publisher(
            self._Twist, self._topic, 10)
        self._get_logger = self._node.get_logger

        # 5. 启动后台 spin
        self._executor = self._rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, name="chassis_spin", daemon=True)
        self._spin_thread.start()

        self._ready = True
        self._get_logger().info(
            f"[debug_console] 底盘控制器就绪 → 话题 {self._topic}")

    def _spin(self):
        while self._rclpy.ok() and self._executor is not None:
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def init_error(self) -> str:
        return getattr(self, "_init_error", "未知错误")

    def send_twist(self, vx: float = 0.0, vy: float = 0.0,
                   wz: float = 0.0, dur: float = 0.0,
                   blocking: bool = True):
        """发布 Twist 并可选持续 dur 秒后自动停。

        Args:
            vx, vy, wz: 线速度/角速度（m/s, rad/s）。x前+ y左+ z逆时针+
            dur: 持续秒数（0=仅发一次不自动停）
            blocking: dur>0 时是否阻塞等 dur 结束
        """
        if not self._ready:
            print(f"  ⚠ 底盘控制器未就绪: {self.init_error}")
            return False
        t = self._Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self._pub.publish(t)

        if dur > 0:
            if blocking:
                time.sleep(dur)
                self.stop()
            else:
                threading.Thread(
                    target=self._timed_stop, args=(dur,),
                    name="nudge_timer", daemon=True).start()
        return True

    def _timed_stop(self, dur: float):
        time.sleep(dur)
        self.stop()

    def stop(self):
        """发零速让底盘停止。"""
        if not self._ready:
            return
        t = self._Twist()
        self._pub.publish(t)

    def close(self):
        self.stop()
        self._ready = False
        if self._executor:
            self._executor.remove_node(self._node)
        if self._node:
            self._node.destroy_node()
        self._node = None
        self._pub = None
        self._executor = None

    # ── 便捷：方向标志 → Twist ──────────────────────────────
    @staticmethod
    def moveflags_to_twist(fx: int, fy: int, fyaw: int,
                           v_forward: float = 0.15, v_lateral: float = 0.1,
                           w: float = 0.6):
        """把 X5 移动帧的方向标志转为 (vx, vy, wz)。
        fx: 1右 2左 ; fy: 1前 2后 ; fyaw: 1顺 2逆。
        底盘 Twist: x前+ / y左+ / z逆时针+。
        """
        vx = vy = wz = 0.0
        if fy == 1:       # 前
            vx = v_forward
        elif fy == 2:     # 后
            vx = -v_forward
        if fx == 1:       # 右 → y-
            vy = -v_lateral
        elif fx == 2:     # 左 → y+
            vy = v_lateral
        if fyaw == 1:     # 顺时针 → z-
            wz = -w
        elif fyaw == 2:   # 逆时针 → z+
            wz = w
        return vx, vy, wz


# ══════════════════════════════════════════════════════════════
# 通用小工具
# ══════════════════════════════════════════════════════════════
def ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print("\n退出。")
        sys.exit(0)


def load_waypoint_names():
    """读路点文件，返回已打的点名列表（不含 routes）。"""
    if not WAYPOINTS_FILE.exists():
        return None
    try:
        import yaml
        data = yaml.safe_load(open(WAYPOINTS_FILE)) or {}
    except Exception as e:
        print(f"读取路点文件失败: {e}")
        return None
    wps = data.get("waypoints", data)   # 兼容顶层直接是点表
    names = [k for k in wps.keys() if k != "routes"] if isinstance(wps, dict) else []
    return names


# ══════════════════════════════════════════════════════════════
# 导航测试：列点名 → 输入 → 调 run_route
# ══════════════════════════════════════════════════════════════
def nav_menu():
    print("\n" + "=" * 48)
    print("  导航测试（调用 run_route，输入一个或多个点名）")
    print("=" * 48)

    names = load_waypoint_names()
    if names is None:
        print(f"⚠ 找不到路点文件 {WAYPOINTS_FILE}")
        print("  请先按 OPERATIONS.md §2.2 用 record_waypoint 打点。")
        return
    if not names:
        print("⚠ 路点文件里还没有任何点，请先打点。")
        return

    print(f"\n当前已打的 {len(names)} 个路点：")
    for i, n in enumerate(names, 1):
        print(f"  {n}", end="\n" if i % 3 == 0 else "\t")
    if len(names) % 3 != 0:
        print()

    print("\n输入要走的点名（空格分隔可多个，按顺序走）；直接回车返回：")
    line = ask("点名 > ")
    if not line:
        return

    want = line.split()
    bad = [w for w in want if w not in names]
    if bad:
        print(f"⚠ 这些点不在路点文件里: {bad}，取消。")
        return

    print(f"\n▶ 依次导航: {' → '.join(want)}")
    print("  (调用 run_route，狗会实际移动，Ctrl+C 可中断)\n")
    try:
        # 依赖已 source 的导航环境；等价于命令行 ros2 run simple_nav_fsm run_route <names>
        subprocess.run(
            ["ros2", "run", "simple_nav_fsm", "run_route", *want],
            check=False)
    except FileNotFoundError:
        print("⚠ 找不到 ros2 命令。请在 source 了导航环境的终端运行本程序：")
        print("  source ~/dog_auto/nav/install/setup.bash")
    except KeyboardInterrupt:
        print("\n已中断导航。")


# ══════════════════════════════════════════════════════════════
# X5 测试：索引表 → 输入序号 → 发单条命令、读回帧
#   收到移动帧(微调)后自动向底盘发 Twist 完成挪动。
# ══════════════════════════════════════════════════════════════
def x5_menu(port: str, chassis: ChassisController = None,
            nudge_vx: float = 0.15, nudge_vy: float = 0.1,
            nudge_w: float = 0.6, nudge_step_sec: float = 0.2):
    print("\n" + "=" * 48)
    print(f"  X5 串口测试（端口 {port}）")
    if chassis and chassis.ready:
        print(f"  底盘联动: ON（收到微调帧→自动发 Twist, "
              f"vx={nudge_vx} vy={nudge_vy} w={nudge_w} step={nudge_step_sec}s）")
    else:
        print(f"  底盘联动: OFF（仅打印移动帧，不实际挪动）")
    print("=" * 48)

    try:
        import serial  # noqa: F401
    except ImportError:
        print("⚠ 未安装 pyserial：pip3 install pyserial")
        return

    while True:
        print("\n  可发送的单条命令：")
        for i, (name, s1, s2, _to) in enumerate(X5_COMMANDS, 1):
            print(f"   [{i}] {name:<14} 帧: AB {s1:02X} {s2:02X} BA")
        print("   [0] 返回上级菜单")

        sel = ask("  选命令序号 > ")
        if sel in ("0", ""):
            return
        if not sel.isdigit() or not (1 <= int(sel) <= len(X5_COMMANDS)):
            print("  ⚠ 无效序号")
            continue

        name, s1, s2, timeout = X5_COMMANDS[int(sel) - 1]
        _x5_send_one(port, name, s1, s2, timeout,
                     chassis=chassis, nudge_vx=nudge_vx,
                     nudge_vy=nudge_vy, nudge_w=nudge_w,
                     nudge_step_sec=nudge_step_sec)


def _x5_send_one(port, name, s1, s2, timeout,
                 chassis=None, nudge_vx=0.15, nudge_vy=0.1,
                 nudge_w=0.6, nudge_step_sec=0.2):
    import serial
    frame = bytes([0xAB, s1, s2, 0xBA])
    print(f"\n  ▶ 发送【{name}】: {frame.hex(' ').upper()}  "
          f"(最多等 {timeout}s，Ctrl+C 停)")
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"  ⚠ 打开串口失败: {e}")
        return

    ser.reset_input_buffer()

    # ── 后台读线程（复用 task_decision FrameParser）─────────
    frame_queue = queue.Queue()
    stop_event = threading.Event()
    parser = _FrameParser()

    def reader():
        while not stop_event.is_set():
            try:
                data = ser.read(64)
            except Exception:
                time.sleep(0.1)
                continue
            if not data:
                continue
            for f in parser.feed(data):
                frame_queue.put(f)

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # ── 发命令 → 等回帧 ──────────────────────────────────
    ser.write(frame)
    done = False
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            try:
                item = frame_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            kind = item[0]
            if kind == 'state':
                s3 = item[3]
                print(f"    状态帧: s3={s3} ({S3_NAMES.get(s3, '?')})")
                if s3 == 0x02:
                    done = True
                    break
            elif kind == 'digit':
                print(f"    数字帧: 答案(高分区)={item[1]} "
                      f"箱编号={item[2]}")
            elif kind == 'move':
                f1, f2, f3 = item[1], item[2], item[3]
                fx = int(round(f1))
                fy = int(round(f2))
                fyaw = int(round(f3))
                print(f"    移动帧(微调): x{DIR.get(fx, '?')} "
                      f"y{DIR.get(fy, '?')} "
                      f"yaw{DIR.get(fyaw, '?')}")

                # ★ 收到移动帧 → 向底盘发 Twist 完成微调
                if chassis and chassis.ready:
                    vx, vy, wz = ChassisController.moveflags_to_twist(
                        fx, fy, fyaw,
                        v_forward=nudge_vx, v_lateral=nudge_vy, w=nudge_w)
                    if vx == 0 and vy == 0 and wz == 0:
                        print(f"    → 底盘: 全零标志，不动")
                    else:
                        print(f"    → 底盘: vx={vx:+.2f} vy={vy:+.2f} "
                              f"wz={wz:+.2f} "
                              f"(持续{nudge_step_sec}s)")
                        chassis.send_twist(vx, vy, wz, dur=nudge_step_sec,
                                          blocking=True)
    except KeyboardInterrupt:
        print("\n    已停止等待。")
    finally:
        stop_event.set()
        ser.close()
        if chassis and chassis.ready:
            chassis.stop()

    if done:
        print(f"  ✓ 【{name}】完成 (s3=02)")
    else:
        print(f"  ⚠ 【{name}】未在 {timeout}s 内收到完成(s3=02)，"
              f"检查接线/X5 状态")


# ══════════════════════════════════════════════════════════════
# 底盘直控测试：手动发 Twist 到 /cmd_vel
# ══════════════════════════════════════════════════════════════
def chassis_menu(chassis: ChassisController):
    print("\n" + "=" * 48)
    print("  底盘直控测试（手动发 Twist 速度指令）")
    print("=" * 48)

    if not chassis.ready:
        print(f"\n  ⚠ 底盘控制器未就绪: {chassis.init_error}")
        print("  请在 source 了 ROS 环境的终端运行本程序，例如：")
        print("    source /opt/ros/humble/setup.bash")
        print("    source ~/dog_auto/nav/install/setup.bash")
        return

    print(f"\n  发布话题: {chassis._topic}")
    print(f"  方向约定: x前+ / y左+ / z逆时针+")
    print(f"  警告：狗会实际移动！确保有足够空间。")

    while True:
        print("\n" + "-" * 48)
        print("  [1] 单步发一次（输入 Vx Vy ω 发一条）")
        print("  [2] 持续发（输入 Vx Vy ω 后一直发，回车停止）")
        print("  [3] 快捷：前进 0.3m/s（持续，回车停）")
        print("  [4] 快捷：后退 0.3m/s（持续，回车停）")
        print("  [5] 快捷：左转 0.3rad/s（持续，回车停）")
        print("  [6] 快捷：右转 0.3rad/s（持续，回车停）")
        print("  [7] 急停（发零速）")
        print("  [0] 返回上级菜单")

        sel = ask("  选择 > ")
        if sel in ("0", ""):
            chassis.stop()
            return

        if sel == "1":
            line = ask("    Vx Vy ω (空格分隔, m/s & rad/s) > ")
            try:
                parts = [float(x) for x in line.split()]
                if len(parts) != 3:
                    print("    ⚠ 请输入 3 个数")
                    continue
                vx, vy, wz = parts
                chassis.send_twist(vx, vy, wz)
                print(f"    → 已发: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            except ValueError:
                print("    ⚠ 数字格式错误")

        elif sel == "2":
            line = ask("    Vx Vy ω (空格分隔, m/s & rad/s) > ")
            try:
                parts = [float(x) for x in line.split()]
                if len(parts) != 3:
                    print("    ⚠ 请输入 3 个数")
                    continue
                vx, vy, wz = parts
                # 后台线程持续发（50Hz）
                stop_flag = threading.Event()

                def _pub_loop():
                    while not stop_flag.is_set():
                        chassis.send_twist(vx, vy, wz, dur=0.0,
                                          blocking=False)
                        time.sleep(0.02)

                t = threading.Thread(target=_pub_loop, daemon=True)
                t.start()
                print(f"    ▶ 持续发送 vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}"
                      f" ... 按回车停止")
                input()
                stop_flag.set()
                chassis.stop()
                print("    ■ 已停止")
            except ValueError:
                print("    ⚠ 数字格式错误")

        elif sel in ("3", "4", "5", "6"):
            presets = {
                "3": (0.30, 0.0, 0.0, "前进 0.3m/s"),
                "4": (-0.30, 0.0, 0.0, "后退 0.3m/s"),
                "5": (0.0, 0.0, 0.30, "左转 0.3rad/s"),
                "6": (0.0, 0.0, -0.30, "右转 0.3rad/s"),
            }
            vx, vy, wz, desc = presets[sel]
            stop_flag = threading.Event()

            def _pub_loop():
                while not stop_flag.is_set():
                    chassis.send_twist(vx, vy, wz, dur=0.0,
                                      blocking=False)
                    time.sleep(0.02)

            t = threading.Thread(target=_pub_loop, daemon=True)
            t.start()
            print(f"    ▶ {desc} ... 按回车停止")
            input()
            stop_flag.set()
            chassis.stop()
            print("    ■ 已停止")

        elif sel == "7":
            chassis.stop()
            print("    ■ 已发零速（急停）")

        else:
            print("  ⚠ 无效选择")


# ══════════════════════════════════════════════════════════════
# 主菜单
# ══════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="决策机联调 · 交互式调试台")
    ap.add_argument("--port", default="/dev/ttyX5",
                    help="X5 串口设备 (默认 /dev/ttyX5)")
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel",
                    help="底盘控制 Twist 话题 (默认 /cmd_vel，即 cmd_bridge 直接订阅的话题)"
                         "；如用 cmd_vel_guard 则设 /cmd_vel_raw")
    ap.add_argument("--nudge-vx", type=float, default=0.15,
                    help="微调前后速度 m/s (默认 0.15)")
    ap.add_argument("--nudge-vy", type=float, default=0.1,
                    help="微调左右速度 m/s (默认 0.1)")
    ap.add_argument("--nudge-w", type=float, default=0.6,
                    help="微调角速度 rad/s (默认 0.6)")
    ap.add_argument("--nudge-step-sec", type=float, default=0.2,
                    help="微调每步持续秒数 (默认 0.2)")
    ap.add_argument("--no-chassis", action="store_true",
                    help="完全禁用底盘联动（即使 ROS 可用）")
    args = ap.parse_args()

    # ── 初始化底盘控制器（可选）────────────────────────────
    chassis = None
    if not args.no_chassis:
        chassis = ChassisController(topic=args.cmd_vel_topic)

    print("╔" + "═" * 46 + "╗")
    print("║   决策机联调 · 交互式调试台 (debug_console)   ║")
    print("╚" + "═" * 46 + "╝")
    print("  提示：FSM + 底盘 + 定位请另开终端按 OPERATIONS §2.3 起好。")
    if chassis and chassis.ready:
        print(f"  底盘控制: OK（话题 {args.cmd_vel_topic}）")
    else:
        print(f"  底盘控制: 不可用（X5 移动帧仅打印，"
              f"[3] 底盘菜单不可用）")

    try:
        while True:
            print("\n" + "-" * 48)
            print("  [1] 测试导航（走路点，调 run_route）")
            print("  [2] 测试 X5 机械臂（串口发单条命令"
                  + (" + 底盘联动" if (chassis and chassis.ready) else "")
                  + "）")
            chassis_label = "测试底盘（手动发 Twist）" \
                if (chassis and chassis.ready) \
                else "测试底盘（需 ROS 环境）"
            print(f"  [3] {chassis_label}")
            print("  [0] 退出")
            sel = ask("  选择 > ")
            if sel == "1":
                nav_menu()
            elif sel == "2":
                x5_menu(args.port, chassis=chassis,
                        nudge_vx=args.nudge_vx,
                        nudge_vy=args.nudge_vy,
                        nudge_w=args.nudge_w,
                        nudge_step_sec=args.nudge_step_sec)
            elif sel == "3":
                if chassis:
                    chassis_menu(chassis)
                else:
                    print("  ⚠ 底盘控制器未初始化，请去掉 --no-chassis 重试")
            elif sel in ("0", "q", "quit", "exit"):
                print("再见。")
                return
            else:
                print("  ⚠ 请输入 1 / 2 / 3 / 0")
    finally:
        if chassis:
            chassis.close()


if __name__ == "__main__":
    main()
