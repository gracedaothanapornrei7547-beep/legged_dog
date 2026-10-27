#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_mission_node —— 简化决策机（v1.1）

流程（路点名称从 config/mission.yaml 的 route 映射读取）：
  1. 启动后手动输入两个颜色（如 red blue）
  2. 初始化 X5 机械臂
  3. 导航: start → grab（抓取点）
  4. 发抓取命令 → 等 X5 回复 s3=02(完成)
  5. 导航: cross（过减速带）→ place_<color1> → 发放置左托盘 → 等 s3=02
  6. 导航: place_<color2> → 发放置右托盘 → 等 s3=02
  7. 完赛

X5 通信模式对齐 debug_console.py：发串口命令 → 循环等回帧 → 收到 s3=02 后再继续。

配置文件: config/mission.yaml
  route:
    start:       "1"       # 起始点
    grab:        "22"      # 抓取点（gap12缝中站位）
    cross:       "2222"    # 过减速带点（zone12_approach）
    place_green: "1111"    # 食品区(绿) 放置点
    place_gray:  "2222"    # 工具区(灰) 放置点
    place_blue:  "3333"    # 仪器区(蓝) 放置点
    place_red:   "4444"    # 药品区(红) 放置点

用法：
  ros2 run simple_dicision simple_mission_node --ros-args -p dry_run:=True
  ros2 launch simple_dicision simple_mission.launch.py
"""
import math
import os
import queue
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


# ═══════════════════════════════════════════════════════════════
# 协议常量（对齐《主机B_协议对接文档.md》）
# ═══════════════════════════════════════════════════════════════
HEAD_STATE, TAIL_STATE = 0xAB, 0xBA
HEAD_DIGIT, TAIL_DIGIT = 0xCD, 0xDC

S1_INIT      = 0x01   # 初始化
S1_GRAB      = 0x04   # 抓取
S1_PLACE     = 0x05   # 放置

S2_NONE        = 0x00
S2_LEFT        = 0x01   # 抓左 / 放左
S2_PLACE_LEFT  = 0x01
S2_PLACE_RIGHT = 0x02

S3_RUNNING = 0x01   # 正在运行
S3_DONE    = 0x02   # 完成
S3_FAIL    = 0x03   # 失败

S1_NAMES = {0x01: "初始化", 0x04: "抓取", 0x05: "放置"}
S3_NAMES = {0x01: "运行中", 0x02: "完成", 0x03: "失败"}

# ── 颜色 → 归位区编号 ────────────────────────────────────
COLOR_TO_ZONE = {
    "green": 0, "gray": 1, "grey": 1, "blue": 2, "red": 3,
    "绿": 0,    "灰": 1,   "蓝": 2,    "红": 3,
    "0": 0, "1": 1, "2": 2, "3": 3,
}
ZONE_NAMES = ["green", "gray", "blue", "red"]

DEFAULT_WAYPOINTS_FILE = str(Path(os.path.expanduser("~"))
                             / ".simple_nav_fsm_waypoints.yaml")

# ── 默认路点映射（逻辑名 → 路点名）───────────────────────
DEFAULT_ROUTE = {
    "start":       "1",
    "grab":        "22",
    "cross":       "2222",
    "place_green": "1111",
    "place_gray":  "2222",
    "place_blue":  "3333",
    "place_red":   "4444",
}


# ═══════════════════════════════════════════════════════════════
# 帧解析器（对齐 debug_console.py _FrameParser）
# ═══════════════════════════════════════════════════════════════
class FrameParser:
    """逐字节增量解析 A→B 状态帧 + 数字帧。"""

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, b: int) -> Optional[dict]:
        b &= 0xFF
        if len(self.buf) == 0:
            if b in (HEAD_STATE, HEAD_DIGIT):
                self.buf.append(b)
            return None
        self.buf.append(b)
        head = self.buf[0]

        if head == HEAD_STATE:
            if len(self.buf) == 5:
                ok = (self.buf[4] == TAIL_STATE)
                r = None
                if ok:
                    r = {'kind': 'state', 'state1': self.buf[1],
                         'state2': self.buf[2], 'state3': self.buf[3]}
                else:
                    self.dropped += 1
                self.buf = bytearray()
                return r
            if len(self.buf) > 5:
                self.dropped += 1
                self.buf = bytearray()
            return None

        if head == HEAD_DIGIT:
            if len(self.buf) == 11:
                ok = (self.buf[10] == TAIL_DIGIT)
                r = None
                if ok:
                    r = {'kind': 'digit', 'answer': self.buf[1],
                         'box_zones': list(self.buf[2:10])}
                else:
                    self.dropped += 1
                self.buf = bytearray()
                return r
            if len(self.buf) > 11:
                self.dropped += 1
                self.buf = bytearray()
            return None

        self.buf = bytearray()
        return None


# ═══════════════════════════════════════════════════════════════
# 串口链路（对齐 debug_console.py _x5_send_one 模式）
# ═══════════════════════════════════════════════════════════════
class ArmLink:
    """串口收发：发命令帧 → 后台 RX 线程收帧 → events 队列供上层消费。"""

    def __init__(self, logger, port="/dev/ttyX5", baud=115200,
                 dry_run=False, dry_run_step_sec=0.3):
        self._log = logger
        self.port, self.baud = port, baud
        self.dry_run = dry_run
        self._dry_step = dry_run_step_sec

        self.events: "queue.Queue[dict]" = queue.Queue()
        self.parser = FrameParser()
        self._ser = None
        self._ser_lock = threading.Lock()       # ★ 保护 self._ser 读写
        self._rx_thread = None
        self._stop = threading.Event()
        self.tx_count = 0
        self.rx_frames = 0

    def open(self):
        if self.dry_run:
            self._log.warning(f"[arm] DRY-RUN 模式：不开串口 {self.port}")
            return

        import serial
        # ★ 重试打开：杀占用进程 → 打开 → 最多重试 3 次
        for attempt in range(1, 4):
            self._kill_others()
            time.sleep(0.3)                     # 等 kill 生效
            try:
                with self._ser_lock:
                    self._ser = serial.Serial(self.port, self.baud,
                                              bytesize=8, parity='N', stopbits=1,
                                              timeout=0.05)
            except Exception as e:
                self._log.error(f"[arm] 打开串口失败(第{attempt}次): {e}")
                if attempt < 3:
                    time.sleep(1.0)
                continue
            # 成功
            self._stop.clear()
            self._rx_err_cnt = 0
            self._rx_thread = threading.Thread(target=self._rx_loop,
                                               name="arm_rx", daemon=True)
            self._rx_thread.start()
            self._log.info(f"[arm] 串口已打开 {self.port} @ {self.baud} 8N1")
            return
        raise RuntimeError(f"串口 {self.port} 打开失败(重试3次)，"
                           f"请检查: 1)设备是否存在 ls {self.port}  "
                           f"2)是否被占用 lsof {self.port}")

    def _kill_others(self):
        """杀掉所有占用本串口的其他进程（当前进程除外）。"""
        import subprocess, signal
        try:
            out = subprocess.check_output(
                ["fuser", self.port], stderr=subprocess.STDOUT, timeout=3
            ).decode().strip()
        except Exception:
            return
        if not out:
            return
        my_pid = str(os.getpid())
        pids = [p for p in out.split() if p != my_pid]
        if pids:
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGTERM)   # 先优雅退出
                except Exception:
                    pass
            time.sleep(0.1)
            for pid in pids:
                try:
                    os.kill(int(pid), signal.SIGKILL)   # 再强杀
                except Exception:
                    pass
            self._log.warning(f"[arm] 已杀掉占用 {self.port} 的进程: {pids}")

    def close(self):
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.5)
            self._rx_thread = None
        with self._ser_lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                self._log.info("[arm] 串口已关闭")

    def send_cmd(self, s1: int, s2: int = 0x00):
        frame = bytes([HEAD_STATE, s1 & 0xFF, s2 & 0xFF, TAIL_STATE])
        self.tx_count += 1
        self._log.info(f"[arm] TX {frame.hex(' ').upper()}  "
                       f"({S1_NAMES.get(s1, hex(s1))}"
                       f"{f'/{s2:#04x}' if s2 else ''})")
        if self.dry_run:
            threading.Thread(target=self._dry_reply, args=(s1, s2),
                             daemon=True).start()
            return
        with self._ser_lock:
            if self._ser is None:
                raise RuntimeError("串口未打开")
            self._ser.write(frame)

    def get_event(self, timeout: float = 0.1) -> Optional[dict]:
        try:
            return self.events.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self):
        n = 0
        while True:
            try:
                self.events.get_nowait()
                n += 1
            except queue.Empty:
                break
        if n:
            self._log.warning(f"[arm] 丢弃 {n} 帧积压旧帧")

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(64)
            except Exception as e:
                self._rx_err_cnt += 1
                self._log.error(f"[arm] 串口读取异常(#{self._rx_err_cnt}): {e}")
                if self._rx_err_cnt >= 5:   # 连续5次失败 → 重连
                    self._log.warning("[arm] 连续读取失败，尝试重连串口...")
                    self._reconnect()
                    self._rx_err_cnt = 0
                else:
                    time.sleep(0.2)
                continue
            self._rx_err_cnt = 0            # 读成功 → 清零
            if not data:
                continue
            for b in data:
                f = self.parser.feed(b)
                if f:
                    self._on_frame(f)

    def _reconnect(self):
        """关闭旧串口句柄 → 杀占用 → 重新打开（带锁保护）。"""
        with self._ser_lock:
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None
        time.sleep(0.5)
        self._kill_others()    # ★ 运行期间被抢占也杀
        time.sleep(0.3)
        with self._ser_lock:
            try:
                import serial
                self._ser = serial.Serial(self.port, self.baud,
                                          bytesize=8, parity='N', stopbits=1,
                                          timeout=0.05)
                self._log.info(f"[arm] 串口重连成功 {self.port}")
            except Exception as e:
                self._log.error(f"[arm] 串口重连失败: {e}")

    def _on_frame(self, f: dict):
        f['stamp'] = time.monotonic()
        if f['kind'] == 'state':
            self._log.info(f"[arm] RX 状态帧 "
                           f"{S1_NAMES.get(f['state1'], hex(f['state1']))}"
                           f"/{f['state2']:#04x} → "
                           f"{S3_NAMES.get(f['state3'], hex(f['state3']))}")
        elif f['kind'] == 'digit':
            self._log.info(f"[arm] RX 数字帧 答案={f['answer']} "
                           f"箱编号={f['box_zones']}")
        self.rx_frames += 1
        self.events.put(f)

    def _dry_reply(self, s1: int, s2: int):
        time.sleep(self._dry_step * 0.3)
        fe = self.events
        fe.put({'kind': 'state', 'state1': s1, 'state2': s2,
                'state3': S3_RUNNING, 'stamp': time.monotonic()})
        time.sleep(self._dry_step)
        fe.put({'kind': 'state', 'state1': s1, 'state2': s2,
                'state3': S3_DONE, 'stamp': time.monotonic()})


# ═══════════════════════════════════════════════════════════════
# 导航客户端（路点查表 → navigate_to_pose → 阻塞等结果）
# ═══════════════════════════════════════════════════════════════
class NavClient:
    """阻塞式 navigate_to_pose action 客户端。"""

    def __init__(self, node, waypoints_file=DEFAULT_WAYPOINTS_FILE,
                 dry_run=False, dry_run_nav_sec=0.5):
        self._node = node
        self._log = node.get_logger()
        self.waypoints_file = os.path.expanduser(waypoints_file)
        self.dry_run = dry_run
        self._dry_sec = dry_run_nav_sec
        self.waypoints: Dict[str, dict] = {}
        self._active_handle = None
        self._cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose',
                                    callback_group=self._cb_group)
        self.reload()

    def reload(self):
        self.waypoints = {}
        p = Path(self.waypoints_file)
        if not p.exists():
            self._log.warning(f"[nav] 路点文件不存在: {p}")
            return
        data = yaml.safe_load(p.read_text()) or {}
        raw = data.get('waypoints', {}) or {}
        for name, wp in raw.items():
            if isinstance(wp, dict):
                self.waypoints[name] = {
                    'x': float(wp['x']), 'y': float(wp['y']),
                    'yaw': float(wp.get('yaw', 0.0)),
                    'final_heading': bool(wp.get('final_heading', True)),
                }
            elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                self.waypoints[name] = {
                    'x': float(wp[0]), 'y': float(wp[1]),
                    'yaw': float(wp[2]) if len(wp) > 2 else 0.0,
                    'final_heading': True,
                }
        self._log.info(f"[nav] 已加载 {len(self.waypoints)} 个路点 来自 {p}")

    def wait_server(self, timeout=10.0):
        if self.dry_run:
            return True
        ok = self._client.wait_for_server(timeout_sec=timeout)
        if not ok:
            self._log.error("[nav] navigate_to_pose action 不可用")
        return ok

    def cancel(self):
        gh = self._active_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self._log.warning(f"[nav] cancel 异常: {e}")

    def goto(self, name: str, timeout: float = 60.0) -> bool:
        wp = self.waypoints.get(name)
        if wp is None:
            if self.dry_run:
                self._log.warning(f"[nav][dry] 路点 '{name}' 未打点，仿真跳过")
                time.sleep(self._dry_sec)
                return True
            self._log.error(f"[nav] 路点 '{name}' 不存在于 {self.waypoints_file}")
            return False

        if self.dry_run:
            self._log.info(f"[nav][dry] goto {name} "
                           f"(x={wp['x']:.2f} y={wp['y']:.2f}) ... {self._dry_sec:.1f}s")
            time.sleep(self._dry_sec)
            return True

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        yaw = math.radians(wp['yaw'])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not wp['final_heading']:
            goal.behavior_tree = 'xy_only'

        self._log.info(f"[nav] goto {name}: x={wp['x']:.3f} y={wp['y']:.3f} "
                       f"yaw={wp['yaw']:.1f}° 超时={timeout:.0f}s")

        accept_evt = threading.Event()
        result_evt = threading.Event()
        box = {'handle': None, 'status': None}

        def _on_response(fut):
            gh = fut.result()
            box['handle'] = gh if gh.accepted else None
            accept_evt.set()
            if gh.accepted:
                gh.get_result_async().add_done_callback(_on_result)

        def _on_result(fut):
            box['status'] = fut.result().status
            result_evt.set()

        send_fut = self._client.send_goal_async(goal)
        send_fut.add_done_callback(_on_response)

        if not accept_evt.wait(timeout=5.0):
            self._log.error("[nav] 发 goal 后 5s 无响应")
            return False
        if box['handle'] is None:
            self._log.error("[nav] goal 被 FSM 拒绝")
            return False
        self._active_handle = box['handle']

        deadline = time.monotonic() + timeout
        while not result_evt.wait(timeout=0.2):
            if time.monotonic() > deadline:
                self._log.error(f"[nav] goto {name} 超时({timeout:.0f}s)，取消")
                self.cancel()
                result_evt.wait(timeout=3.0)
                self._active_handle = None
                return False
        self._active_handle = None

        ok = (box['status'] == GoalStatus.STATUS_SUCCEEDED)
        if ok:
            self._log.info(f"[nav] ✓ goto {name} 成功")
        else:
            self._log.error(f"[nav] ✗ goto {name} 失败 status={box['status']}")
        return ok


# ═══════════════════════════════════════════════════════════════
# 简化任务状态机
# ═══════════════════════════════════════════════════════════════
class SimpleMission:
    """
    最简流程（route 路点名称从配置文件读取）：
      S0 初始化 → 输入两色 → S3 进场 → S4 抓取 → S5 过带 → S6 放左→放右 → 完赛

    路点序列（逻辑名→实际路点名由 route 映射）：
      start → grab（抓取）→ cross（过减速带）→ place_<c1>（放左）→ place_<c2>（放右）
    """

    def __init__(self, node, arm: ArmLink, nav: NavClient,
                 route: Dict[str, str],
                 nav_timeout=60.0, grab_timeout=60.0, place_timeout=60.0,
                 arm_timeout=180.0, nav_retry=1):
        self.node = node
        self.arm = arm
        self.nav = nav
        self.route = route
        self._log = node.get_logger()
        self._nav_timeout = nav_timeout
        self._grab_timeout = grab_timeout
        self._place_timeout = place_timeout
        self._arm_timeout = arm_timeout
        self._nav_retry = nav_retry

        self._phase_pub = node.create_publisher(String, "~/phase", 10)
        self._abort = threading.Event()
        self._t0 = 0.0

        # 启动时打印路点映射，方便确认
        self._log.info(f"[mission] 路点映射: start={route['start']}  "
                       f"grab={route['grab']}  cross={route['cross']}  "
                       f"place_green={route['place_green']}  "
                       f"place_gray={route['place_gray']}  "
                       f"place_blue={route['place_blue']}  "
                       f"place_red={route['place_red']}")

    # ── 外部控制 ──────────────────────────────────────────────
    def request_abort(self):
        self._log.warning("[mission] 收到中止请求")
        self._abort.set()
        self.nav.cancel()

    def aborted(self) -> bool:
        return self._abort.is_set()

    # ── 主流程 ────────────────────────────────────────────────
    def run(self, left_color: str, right_color: str):
        """入口：传入两个颜色名（如 'red', 'blue'）。"""
        self._t0 = time.monotonic()
        try:
            lz = self._parse_color(left_color)
            rz = self._parse_color(right_color)
            self._log.info(f"[mission] 左侧={left_color}→区{lz}({ZONE_NAMES[lz]})  "
                           f"右侧={right_color}→区{rz}({ZONE_NAMES[rz]})")

            # S0: 打开串口 + 初始化 X5
            self._phase("S0_INIT")
            self.arm.open()
            if not self._run_arm(S1_INIT, S2_NONE,
                                 timeout=self._arm_timeout,
                                 label="初始化机械臂"):
                raise RuntimeError("机械臂初始化失败")

            # S3: 进场 → 抓取点
            self._phase("S3_ENTER")
            self._goto(self.route["start"])
            self._goto(self.route["grab"])

            # S4: 抓取（发一条命令，X5 自行完成左右抓取）
            self._phase("S4_GRAB")
            if not self._run_arm(S1_GRAB, S2_LEFT,
                                 timeout=self._grab_timeout,
                                 label="抓取(左右一起)"):
                self._log.warning("[mission] ⚠ 抓取失败(s3=03)，跳过，继续后续步骤")
            else:
                self._log.info(f"[mission] ✓ 抓取完成: 左托盘=区{lz} 右托盘=区{rz}")

            # S5: 过减速带
            self._phase("S5_CROSS")
            self._goto(self.route["cross"])

            # S6: 放左箱
            wp_left = self.route.get(f"place_{ZONE_NAMES[lz]}")
            if not wp_left:
                raise RuntimeError(f"zone {lz} 无对应放置路点")
            self._phase(f"S6_PLACE_left_{ZONE_NAMES[lz]}")
            self._goto(wp_left)
            if not self._run_arm(S1_PLACE, S2_PLACE_LEFT,
                                 timeout=self._place_timeout,
                                 label=f"放左托盘→区{lz}({ZONE_NAMES[lz]})"):
                self._log.warning(f"[mission] ⚠ 放置左箱失败(s3=03)，跳过")

            # S6: 放右箱
            wp_right = self.route.get(f"place_{ZONE_NAMES[rz]}")
            if not wp_right:
                raise RuntimeError(f"zone {rz} 无对应放置路点")
            self._phase(f"S6_PLACE_right_{ZONE_NAMES[rz]}")
            self._goto(wp_right)
            if not self._run_arm(S1_PLACE, S2_PLACE_RIGHT,
                                 timeout=self._place_timeout,
                                 label=f"放右托盘→区{rz}({ZONE_NAMES[rz]})"):
                self._log.warning(f"[mission] ⚠ 放置右箱失败(s3=03)，跳过")

            self._phase("DONE")
            self._log.info("[mission] ✅ 两箱放置完成，完赛！")

        except Exception as e:
            self._log.error(f"[mission] 任务失败: {e}")
        finally:
            self._summary()

    # ── 颜色解析 ──────────────────────────────────────────────
    def _parse_color(self, s: str) -> int:
        s = s.strip().lower()
        if s in COLOR_TO_ZONE:
            return COLOR_TO_ZONE[s]
        raise ValueError(f"无法识别的颜色: '{s}'，支持: {list(COLOR_TO_ZONE.keys())}")

    # ── 机械臂命令（对齐 debug_console.py _x5_send_one 模式）───
    def _run_arm(self, s1: int, s2: int, timeout: float,
                 label: str) -> bool:
        if self.aborted():
            return False
        self.arm.drain()

        self._log.info(f"[mission] ▶ {label}")
        self.arm.send_cmd(s1, s2)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.aborted():
                return False
            f = self.arm.get_event(timeout=0.1)
            if f is None:
                continue
            if f['kind'] != 'state':
                continue

            fs1, fs3 = f['state1'], f['state3']
            if fs1 != s1:
                self._log.warning(f"[mission] {label}: 串台回执 "
                                  f"s1={fs1:#04x}(期望{s1:#04x})，忽略")
                continue
            if fs3 == S3_RUNNING:
                continue
            if fs3 == S3_DONE:
                self._log.info(f"[mission] ✓ {label} 完成 (s3=02)")
                return True
            if fs3 == S3_FAIL:
                self._log.error(f"[mission] ✗ {label} X5 报失败 (s3=03)")
                return False

        self._log.error(f"[mission] ✗ {label} 超时 ({timeout:.0f}s)")
        return False

    # ── 导航（带重试）─────────────────────────────────────────
    def _goto(self, name: str):
        for k in range(1, self._nav_retry + 2):
            if self.aborted():
                raise RuntimeError("已中止")
            if self.nav.goto(name, timeout=self._nav_timeout):
                return
            self._log.warning(f"[mission] 导航 {name} 第{k}次失败")
        raise RuntimeError(f"导航 {name} 失败")

    # ── 工具方法 ──────────────────────────────────────────────
    def _phase(self, phase: str):
        self._log.info(f"[mission] 阶段: {phase}")
        self._phase_pub.publish(String(data=phase))

    def _summary(self):
        dt = time.monotonic() - self._t0
        self._log.info(f"[mission] === 统计 === 用时 {dt:.1f}s | "
                       f"串口 TX {self.arm.tx_count} RX {self.arm.rx_frames} "
                       f"丢帧 {self.arm.parser.dropped}")


# ═══════════════════════════════════════════════════════════════
# 配置文件加载
# ═══════════════════════════════════════════════════════════════
def load_route_config(path: str, logger) -> Dict[str, str]:
    """读取路点映射配置文件，返回 route 字典。"""
    p = Path(os.path.expanduser(path))
    if not p.exists():
        logger.warning(f"[config] 路点配置文件不存在: {p}，使用内置默认值")
        return dict(DEFAULT_ROUTE)

    try:
        data = yaml.safe_load(p.read_text()) or {}
        cfg = data.get('route', {}) or {}
    except Exception as e:
        logger.error(f"[config] 读取路点配置文件失败: {e}，使用内置默认值")
        return dict(DEFAULT_ROUTE)

    # 用默认值补全缺失的 key
    route = {}
    for key, default in DEFAULT_ROUTE.items():
        val = cfg.get(key, default)
        route[key] = str(val) if val is not None else default
        if key not in cfg:
            logger.warning(f"[config] 路点映射缺少 '{key}'，使用默认 '{default}'")

    logger.info(f"[config] 已加载路点映射: {len(route)} 项 来自 {p}")
    for k, v in route.items():
        logger.info(f"  {k}: '{v}'")
    return route


# ═══════════════════════════════════════════════════════════════
# ROS2 节点入口
# ═══════════════════════════════════════════════════════════════
class SimpleDirectorNode(Node):
    def __init__(self):
        super().__init__("simple_dicision")
        p = self._declare_params()
        log = self.get_logger()

        # 加载路点映射配置
        route = load_route_config(p["route_config"], log)

        self.arm = ArmLink(
            log,
            port=p["serial_port"], baud=p["serial_baud"],
            dry_run=p["dry_run"],
            dry_run_step_sec=p["dry_run_step_sec"],
        )
        self.nav = NavClient(
            self,
            waypoints_file=p["waypoints_file"],
            dry_run=p["dry_run"],
            dry_run_nav_sec=p["dry_run_nav_sec"],
        )
        self.mission = SimpleMission(
            self, self.arm, self.nav, route,
            nav_timeout=p["nav_timeout_sec"],
            grab_timeout=p["grab_timeout_sec"],
            place_timeout=p["place_timeout_sec"],
            arm_timeout=p["arm_timeout"],
            nav_retry=p["nav_retry"],
        )
        self._thread = None

        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/abort", self._srv_abort)

        if p["dry_run"]:
            log.warning("★★ DRY-RUN 模式：不开串口、导航仅仿真 ★★")

        # 读取颜色：优先 ROS 参数，参数为空则尝试 stdin
        self._left_color = p["left_color"].strip()
        self._right_color = p["right_color"].strip()
        if not self._left_color or not self._right_color:
            try:
                if not self._left_color:
                    self._left_color = input(
                        "请输入左侧箱子颜色 (green/gray/blue/red): ").strip()
                if not self._right_color:
                    self._right_color = input(
                        "请输入右侧箱子颜色 (green/gray/blue/red): ").strip()
            except (EOFError, KeyboardInterrupt):
                pass
        # 最终兜底
        if not self._left_color:
            self._left_color = "red"
        if not self._right_color:
            self._right_color = "blue"
        log.info(f"输入: 左={self._left_color} 右={self._right_color}")

        if p["auto_start"]:
            log.info(f"auto_start=true：{p['start_delay']:.1f}s 后自动开跑")
            self._start_timer = self.create_timer(
                max(p["start_delay"], 0.05), self._auto_start)
        else:
            log.info("auto_start=false：等待服务调用 "
                     "`ros2 service call /simple_dicision/start std_srvs/srv/Trigger`")

    # ── 启动 / 中止 ──────────────────────────────────────────
    def _auto_start(self):
        self._start_timer.cancel()
        self._launch()

    def _srv_start(self, req, resp):
        if self._thread and self._thread.is_alive():
            resp.success, resp.message = False, "任务已在运行"
            return resp
        self._launch()
        resp.success, resp.message = True, "任务已启动"
        return resp

    def _srv_abort(self, req, resp):
        self.mission.request_abort()
        resp.success, resp.message = True, "已请求中止"
        return resp

    def _launch(self):
        if self._thread and self._thread.is_alive():
            self.get_logger().warning("任务线程已在运行，忽略重复启动")
            return
        self._thread = threading.Thread(
            target=self.mission.run,
            args=(self._left_color, self._right_color),
            name="mission", daemon=True)
        self._thread.start()

    def shutdown(self):
        self.mission.request_abort()
        self.arm.close()

    # ── 参数 ─────────────────────────────────────────────────
    def _declare_params(self) -> dict:
        d = self.declare_parameter
        p = {}
        p["serial_port"] = d("serial_port", "/dev/ttyX5").value
        p["serial_baud"] = d("serial_baud", 115200).value
        p["nav_timeout_sec"] = float(d("nav_timeout_sec", 60.0).value)
        p["grab_timeout_sec"] = float(d("grab_timeout_sec", 60.0).value)
        p["place_timeout_sec"] = float(d("place_timeout_sec", 60.0).value)
        p["arm_timeout"] = float(d("arm_timeout", 180.0).value)
        p["nav_retry"] = int(d("nav_retry", 1).value)
        p["waypoints_file"] = d("waypoints_file", DEFAULT_WAYPOINTS_FILE).value
        p["route_config"] = d("route_config", "").value
        p["left_color"] = d("left_color", "").value
        p["right_color"] = d("right_color", "").value
        p["auto_start"] = bool(d("auto_start", True).value)
        p["start_delay"] = float(d("start_delay", 2.0).value)
        p["dry_run"] = bool(d("dry_run", False).value)
        p["dry_run_step_sec"] = float(d("dry_run_step_sec", 0.3).value)
        p["dry_run_nav_sec"] = float(d("dry_run_nav_sec", 0.5).value)
        return p


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDirectorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
