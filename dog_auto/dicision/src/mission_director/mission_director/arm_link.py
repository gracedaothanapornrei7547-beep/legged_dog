#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_link —— 决策机(B) ↔ X5(A) 串口链路

职责（《主状态机.md》§7.2）：串口收发 / 协议编解码 / RX 线程 / dry_run。
协议定义完全对齐《主机B_协议对接文档.md》：
  B→A 状态帧(4B):  AB s1 s2 BA
  A→B 状态帧(5B):  AB s1 s2 s3 BA
  A→B 数字帧(11B): CD 答案 8箱色 DC
  （已移除微调：不再收发移动帧 EF..FE / s3=04）

对上层（mission）的接口：
  open()/close()          打开/关闭串口（dry_run 时为空操作）
  send_cmd(s1, s2)        发一条命令帧
  get_event(timeout)      按到达顺序取一帧（dict），超时返回 None
  drain()                 丢弃积压的旧帧（发新命令前调用）
  last_digit              最近一帧数字帧 {'answer','box_colors','stamp'}

线程模型：RX 线程持续读串口→逐字节喂 FrameParser→完整帧进 events 队列。
mission 在自己的线程里消费队列，本模块不做任何时序决策。
"""
import queue
import threading
import time
from typing import Optional

# ── 协议常量 ─────────────────────────────────────────────────────
HEAD_STATE, TAIL_STATE = 0xAB, 0xBA
HEAD_DIGIT, TAIL_DIGIT = 0xCD, 0xDC

# 状态1（主状态）
S1_INIT = 0x01        # 初始化
S1_RECOGNIZE = 0x02   # 数字识别
S1_SCAN = 0x03        # 扫描8箱
S1_GRAB = 0x04        # 抓取
S1_PLACE = 0x05       # 放置

# 状态2（子状态）
S2_NONE = 0x00
S2_LEFT = 0x01        # 抓取: 抓左边 → 左托盘
S2_RIGHT = 0x02       # 抓取: 抓右边 → 右托盘
S2_PLACE_LEFT = 0x01  # 放置(v3): 放左托盘的箱（原"放第1层"作废）
S2_PLACE_RIGHT = 0x02  # 放置(v3): 放右托盘的箱（原"放第2层"作废）

# 数字帧编号（v3.2 统一为归位区编号，颜色码作废）
ZONE_UNDETECTED = 0xFF  # 未检测

# 状态3（A 的运行状态）
S3_RUNNING = 0x01     # 正在运行
S3_DONE = 0x02        # 完成（成功）
S3_FAIL = 0x03        # 意外中断（失败）

S1_NAMES = {S1_INIT: "初始化", S1_RECOGNIZE: "数字识别", S1_SCAN: "扫描8箱",
            S1_GRAB: "抓取", S1_PLACE: "放置"}
S3_NAMES = {S3_RUNNING: "运行中", S3_DONE: "完成", S3_FAIL: "失败"}


def build_state_cmd(state1: int, state2: int = 0x00) -> bytes:
    """构造 B→A 状态帧（协议文档 §4.1）。"""
    return bytes([HEAD_STATE, state1 & 0xFF, state2 & 0xFF, TAIL_STATE])


class FrameParser:
    """
    字节流状态机：feed(byte) → 完整帧 dict 或 None。
    逻辑与《主机B_协议对接文档.md》§4.2 参考实现一致，仅补了丢帧计数。
    """

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0          # 帧尾校验失败/长度异常的丢弃次数
        self.noise_bytes = 0      # 非帧头起始的噪声字节数

    def feed(self, b: int) -> Optional[dict]:
        b &= 0xFF
        if len(self.buf) == 0:
            if b in (HEAD_STATE, HEAD_DIGIT):
                self.buf.append(b)
            else:
                self.noise_bytes += 1
            return None
        self.buf.append(b)
        head = self.buf[0]

        # 状态帧 A→B: AB s1 s2 s3 BA (5字节)
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

        # 数字帧: CD 答案 8箱编号 DC (11字节) —— v3.2 编号语义
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


class ArmLink:
    """串口链路。dry_run=True 时不开串口，用内置仿真回帧（本机跑流程用）。"""

    def __init__(self, logger,
                 port: str = "/dev/ttyX5",
                 baud: int = 115200,
                 dry_run: bool = False,
                 dry_run_answer: int = 2,
                 dry_run_box_zones=(0, 2, 1, 3, 2, 1, 0, 3),
                 dry_run_step_sec: float = 0.3):
        """
        v3.2（已移除微调）:
          - 数字帧 8箱/答案 都是归位区编号 0-3（0xFF=未检测）。
          - A→B 只剩状态帧 + 数字帧，不再有移动帧。
        dry_run_*: 仿真参数：答案(高分区编号)/8箱编号/每步延迟
        """
        self._log = logger
        self.port, self.baud = port, int(baud)
        self.dry_run = bool(dry_run)
        self._dry_answer = int(dry_run_answer)
        self._dry_zones = list(dry_run_box_zones)
        self._dry_step = float(dry_run_step_sec)

        self.events: "queue.Queue[dict]" = queue.Queue()
        self.last_digit: Optional[dict] = None
        self.parser = FrameParser()
        self._ser = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.tx_count = 0
        self.rx_frames = 0

    # ── 打开 / 关闭 ───────────────────────────────────────────
    def open(self):
        if self.dry_run:
            self._log.warning(f"[arm_link] DRY-RUN 模式：不开串口 {self.port}，"
                              f"用仿真回帧（答案={self._dry_answer} "
                              f"箱编号={self._dry_zones}）")
            return
        import serial  # 延迟导入：dry_run 的开发机可以没装 pyserial
        self._ser = serial.Serial(self.port, self.baud,
                                  bytesize=8, parity='N', stopbits=1,
                                  timeout=0.05)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop,
                                           name="arm_rx", daemon=True)
        self._rx_thread.start()
        self._log.info(f"[arm_link] 串口已打开 {self.port} @ {self.baud} 8N1")

    def close(self):
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._log.info("[arm_link] 串口已关闭")

    # ── 发送 ─────────────────────────────────────────────────
    def send_cmd(self, s1: int, s2: int = 0x00):
        frame = build_state_cmd(s1, s2)
        self.tx_count += 1
        self._log.info(f"[arm_link] TX {frame.hex(' ').upper()}  "
                       f"({S1_NAMES.get(s1, hex(s1))}"
                       f"{f'/{s2:#04x}' if s2 else ''})")
        if self.dry_run:
            threading.Thread(target=self._dry_reply, args=(s1, s2),
                             daemon=True).start()
            return
        if self._ser is None:
            raise RuntimeError("串口未打开（先调用 open()）")
        self._ser.write(frame)

    # ── 接收 ─────────────────────────────────────────────────
    def get_event(self, timeout: float = 0.1) -> Optional[dict]:
        """取一帧（按到达顺序）。超时返回 None。"""
        try:
            return self.events.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> int:
        """丢弃积压的旧帧（发新命令前调用，防止旧回执串台）。"""
        n = 0
        while True:
            try:
                self.events.get_nowait()
                n += 1
            except queue.Empty:
                break
        if n:
            self._log.warning(f"[arm_link] 丢弃 {n} 帧积压旧帧")
        return n

    # ── RX 线程 ──────────────────────────────────────────────
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(64)
            except Exception as e:
                self._log.error(f"[arm_link] 串口读取异常: {e}")
                time.sleep(0.2)
                continue
            if not data:
                continue
            before = self.parser.dropped
            for b in data:
                f = self.parser.feed(b)
                if f:
                    self._on_frame(f)
            if self.parser.dropped > before:
                self._log.warning(f"[arm_link] 帧校验失败，累计丢弃 "
                                  f"{self.parser.dropped} 帧")

    def _on_frame(self, f: dict):
        f['stamp'] = time.monotonic()
        if f['kind'] == 'digit':
            self.last_digit = f
            self._log.info(f"[arm_link] RX 数字帧 答案(高分区)={f['answer']} "
                           f"箱编号={f['box_zones']}")
        else:
            self._log.info(f"[arm_link] RX 状态帧 "
                           f"{S1_NAMES.get(f['state1'], hex(f['state1']))}"
                           f"/{f['state2']:#04x} → "
                           f"{S3_NAMES.get(f['state3'], hex(f['state3']))}")
        self.rx_frames += 1
        self.events.put(f)

    # ── dry_run 仿真：模拟 A 端回帧时序（协议文档 §三 流程）────
    def _dry_reply(self, s1: int, s2: int):
        def put(f: dict):
            f['stamp'] = time.monotonic()
            if f['kind'] == 'digit':
                self.last_digit = f
            self.events.put(f)

        time.sleep(self._dry_step * 0.3)
        put({'kind': 'state', 'state1': s1, 'state2': s2,
             'state3': S3_RUNNING})
        if s1 in (S1_RECOGNIZE, S1_SCAN):
            time.sleep(self._dry_step)
            answer = self._dry_answer if s1 == S1_RECOGNIZE else 0
            put({'kind': 'digit', 'answer': answer,
                 'box_zones': list(self._dry_zones)})
        time.sleep(self._dry_step)
        put({'kind': 'state', 'state1': s1, 'state2': s2, 'state3': S3_DONE})
