#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission —— 主状态机（v3.2）：串口指挥 X5 + navigate_to_pose 指挥导航

流程（《决策机完整说明 v3.2》§2）：
  S0 初始化 → S1 识别+扫描 → S2 规划(选一条异色缝) → S3~S6 单趟搬运 → 完赛

关键 v3.2 定案：
  * 只搬 back 排，选一条异色缝(异色→高分→平票最短)，只跑1趟搬2箱。
  * 编号统一：8箱/答案/归位区/place路点全用归位区编号 0-3(0xFF未检测)。
  * 放置=区编号比对托盘编号：区==左托盘→05 01(放左)，区==右托盘→05 02(放右)。
  * 【已移除微调】雷达定位精度已达标，抓/放不再"停-发-调-干"，A→B 只剩状态帧+数字帧。

线程：run() 跑在 worker 线程（阻塞式）；node 由 MultiThreadedExecutor 主线程 spin。
"""
import threading
import time
from typing import Dict, List, Optional

from std_msgs.msg import String

from . import arm_link as al
from .strategy import Strategy, Trip

# 规范路点（14个，v3.2 只搬 back）：仅在 strict_waypoints=True 时开跑前点名。
# 注：当前录制的是调试导航用的测试点位，非最终正式点位，故默认不强制（见 mission.yaml）。
CANONICAL_WAYPOINTS = [
    "entry",
    "gap01_pre", "gap12_pre", "gap23_pre",
    "gap01_back", "gap12_back", "gap23_back",
    "zone01_approach", "zone12_approach", "zone23_approach",
    "place_green", "place_gray", "place_blue", "place_red",
]


class TripFailed(Exception):
    pass


class Mission:
    def __init__(self, node, arm, nav, strategy: Strategy, p: Dict):
        self.node = node
        self.arm = arm
        self.nav = nav
        self.strategy = strategy
        self.p = p
        self._log = node.get_logger()

        # 识别结果
        self.answer: int = -1
        self.box_zones: List[int] = [al.ZONE_UNDETECTED] * 8
        self._digit_stamp = 0.0

        # 托盘内容（抓取时记录）：'left'/'right' → 归位区编号
        self.tray: Dict[str, int] = {}

        # 阶段发布
        self._phase_pub = node.create_publisher(String, "~/phase", 10)

        # 控制/统计
        self._abort = threading.Event()
        self._t0 = 0.0
        self.recovers = 0

    # ── 外部控制 ──────────────────────────────────────────────
    def request_abort(self):
        self._log.warning("[mission] 收到中止请求")
        self._abort.set()
        self.nav.cancel()

    def aborted(self) -> bool:
        return self._abort.is_set()

    # ── 主流程 ────────────────────────────────────────────────
    def run(self):
        self._t0 = time.monotonic()
        try:
            self._s0_init()
            self._s1_scan()
            trip = self._s2_plan()
            if trip is None:
                self._log.error("[mission] 无可用异色缝，任务终止")
                self._phase_set("NO_VALID_GAP")
            else:
                self._run_trip(trip)
        except TripFailed as e:
            self._log.error(f"[mission] 任务失败: {e}")
        except Exception as e:
            self._log.error(f"[mission] 未预期异常: {e}")
        finally:
            self._summary()

    # ── S0 初始化 ─────────────────────────────────────────────
    def _s0_init(self):
        self._phase_set("S0_INIT")
        if not self.nav.wait_server(timeout=self.p["nav_server_wait_sec"]):
            # 不做看门狗：等不到只告警，继续往下（FSM 可稍后起来）
            self._log.warning("[mission] 导航服务器未就绪（simple_nav_fsm 没起？），"
                              "不中止，继续执行")
        self.arm.open()
        missing = self.nav.missing(CANONICAL_WAYPOINTS)
        if missing:
            self._log.warning(f"[mission] 缺路点: {missing}")
            if self.p["strict_waypoints"] and not self.p["dry_run"]:
                raise TripFailed(f"strict_waypoints=True 且缺 {len(missing)} 个路点")
        if not self._run_arm(al.S1_INIT, 0,
                             timeout=self.p["arm_timeout"],
                             retries=1, label="初始化机械臂"):
            raise TripFailed("机械臂初始化失败")

    # ── S1 识别+扫描（启动区原地）─────────────────────────────
    def _s1_scan(self):
        self._phase_set("S1_SCAN")
        t_cmd = time.monotonic()
        if not self._run_arm(al.S1_RECOGNIZE, 0,
                             timeout=self.p["arm_timeout"],
                             retries=1, label="数字识别"):
            raise TripFailed("数字识别失败")
        self._await_digit(since=t_cmd)

        t_cmd = time.monotonic()
        if not self._run_arm(al.S1_SCAN, 0,
                             timeout=self.p["arm_timeout"],
                             retries=1, label="扫描8箱"):
            raise TripFailed("扫描8箱失败")
        self._await_digit(since=t_cmd)
        self._log.info(f"[mission] 识别结果: 答案(高分区)={self.answer} "
                       f"箱编号={self.box_zones}")

    # ── S2 规划（选一条异色缝）────────────────────────────────
    def _s2_plan(self) -> Optional[Trip]:
        self._phase_set("S2_PLAN")
        trip = self.strategy.plan_trip(self.answer, self.box_zones)
        return trip

    # ── S3~S6 单趟搬运 ────────────────────────────────────────
    def _run_trip(self, trip: Trip):
        self._log.info(f"[mission] === 开始搬运: {trip.gap} "
                       f"左idx{trip.left_idx}→区{trip.left_zone} "
                       f"右idx{trip.right_idx}→区{trip.right_zone} ===")

        # S3 进场
        self._phase_set(f"S3_ENTER_{trip.gap}")
        self._goto("entry")
        self._goto(trip.pre_point)
        self._goto(trip.grab_point)

        # S4 抓箱（只发左侧帧，X5 自行完成左右抓取后回 s3=02）
        self._phase_set(f"S4_GRAB_{trip.gap}")
        zone_of = {"left": trip.left_zone, "right": trip.right_zone}
        if not self._run_arm(al.S1_GRAB, al.S2_LEFT,
                             timeout=self.p["grab_timeout_sec"],
                             retries=self.p["grab_retry"],
                             label="抓取(左右一起)"):
            self._recover("抓取失败")
            raise TripFailed("抓取失败")
        self.tray["left"] = trip.left_zone
        self.tray["right"] = trip.right_zone
        self._log.info(f"[mission] 左托盘=区{trip.left_zone}"
                       f"({self.strategy.zone_name(trip.left_zone)}) "
                       f"右托盘=区{trip.right_zone}"
                       f"({self.strategy.zone_name(trip.right_zone)})")

        # S5 过带
        self._phase_set(f"S5_CROSS_{trip.gap}")
        self._goto(trip.approach_point)

        # S6 放箱（按写死偏好顺序去两个区，区色比对托盘发放左/放右）
        self._phase_set(f"S6_PLACE_{trip.gap}")
        for zone in trip.place_order:
            wp = self.strategy.waypoint_for_zone(zone)
            if wp is None:
                self._log.error(f"[mission] 区编号{zone}无对应路点，跳过")
                continue
            side = Strategy.place_side(zone, trip.left_zone, trip.right_zone)
            if side is None:
                self._log.error(f"[mission] 区{zone}与左右托盘都不匹配，跳过")
                continue
            self._goto(wp)
            s2 = al.S2_PLACE_LEFT if side == "left" else al.S2_PLACE_RIGHT
            if not self._run_arm(al.S1_PLACE, s2,
                                 timeout=self.p["place_timeout_sec"],
                                 retries=self.p["place_retry"],
                                 label=f"放{side}托盘→区{zone}"
                                       f"({self.strategy.zone_name(zone)})"):
                self._recover(f"放置区{zone}失败")
                raise TripFailed(f"放置区{zone}失败")

        self._log.info("[mission] ✅ 两箱放置完成，完赛！")
        self._phase_set("DONE")

    # ── 机械臂命令 + 事件循环 ────────────────────────────────
    def _run_arm(self, s1: int, s2: int, timeout: float,
                 retries: int, label: str) -> bool:
        """
        发一条命令并等到 s3=02(完成)。
        * s3=03/超时 → 重发，最多 retries 次
        * 数字帧随到随收
        （已移除微调：不再处理移动帧 / s3=04）
        """
        attempts = retries + 1
        for attempt in range(1, attempts + 1):
            if self.aborted():
                return False
            self.arm.drain()
            tag = f"{label} (第{attempt}/{attempts}次)"
            self._log.info(f"[mission] ▶ {tag}")
            self.arm.send_cmd(s1, s2)

            deadline = time.monotonic() + timeout
            failed = False
            while time.monotonic() < deadline:
                if self.aborted():
                    return False
                f = self.arm.get_event(timeout=0.1)
                if f is None:
                    continue
                kind = f["kind"]

                if kind == "digit":
                    self._take_digit(f)
                    continue

                if kind != "state":
                    continue

                fs1, fs3 = f["state1"], f["state3"]
                if fs1 != s1:
                    self._log.warning(f"[mission] {label}: 串台回执 "
                                      f"s1={fs1:#04x}(期望{s1:#04x})，忽略")
                    continue
                if fs3 == al.S3_RUNNING:
                    continue
                if fs3 == al.S3_DONE:
                    self._log.info(f"[mission] ✓ {label} 完成")
                    return True
                if fs3 == al.S3_FAIL:
                    self._log.error(f"[mission] ✗ {label} A 报失败(s3=03)")
                    failed = True
                    break
            if not failed:
                self._log.error(f"[mission] ✗ {label} 超时({timeout:.0f}s)")
        return False

    # ── 数字帧收取 ────────────────────────────────────────────
    def _take_digit(self, f: dict):
        self.answer = int(f["answer"])
        self.box_zones = list(f["box_zones"])
        self._digit_stamp = f.get("stamp", time.monotonic())

    def _await_digit(self, since: float, grace: float = 2.0):
        """等到命令发出(since)之后的数字帧，宽限 grace 秒，杜绝旧帧。"""
        if self._digit_stamp > since:
            return
        t_end = time.monotonic() + grace
        while time.monotonic() < t_end:
            f = self.arm.get_event(timeout=0.1)
            if f and f["kind"] == "digit":
                self._take_digit(f)
                return
            ld = self.arm.last_digit
            if ld and ld.get("stamp", 0) > since:
                self._take_digit(ld)
                return
        self._log.warning("[mission] 宽限期内未等到新数字帧")

    # ── 导航封装（带重试）────────────────────────────────────
    def _goto(self, name: str, timeout: Optional[float] = None):
        timeout = timeout or self.p["nav_timeout_sec"]
        tries = self.p["nav_retry"] + 1
        for k in range(1, tries + 1):
            if self.aborted():
                raise TripFailed("已中止")
            if self.nav.goto(name, timeout=timeout):
                return
            self._log.warning(f"[mission] 导航 {name} 第{k}/{tries}次失败")
        self._recover(f"导航 {name} 失败")
        raise TripFailed(f"导航 {name} 失败")

    # ── 恢复/预算/阶段/统计 ──────────────────────────────────
    def _recover(self, why: str):
        self.recovers += 1
        self._log.error(f"[mission] 恢复计数+1 ({self.recovers}/"
                        f"{self.p['max_recover']}): {why}")

    def _phase_set(self, phase: str):
        self._log.info(f"[mission] 阶段: {phase}")
        self._phase_pub.publish(String(data=phase))

    def _summary(self):
        dt = time.monotonic() - self._t0
        self._log.info(
            f"[mission] === 统计 === 用时 {dt:.1f}s | "
            f"恢复 {self.recovers} 次 | "
            f"串口 TX {self.arm.tx_count} RX {self.arm.rx_frames} "
            f"丢帧 {self.arm.parser.dropped}")
