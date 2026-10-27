#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
strategy —— 选缝策略（v3.2）：答案 + 8箱编号 → 单趟计划

纯 Python 逻辑，不依赖 ROS，可直接单元测试（见 test/test_offline.py）。

依据《决策机完整说明 v3.2》：
  §0 编号统一：8箱字节/答案字节/归位区/place路点全用"归位区编号 0-3"
       0=食品(绿) 1=工具(灰) 2=仪器(蓝) 3=药品(红)，0xFF=未检测。
       颜色码 1-4 彻底作废。答案字节 = 结果%4 = 高分区编号。
  §4 选缝：只搬 back 排(idx0-3)，三缝各对应相邻两箱；
       规则1 异色硬性(同色缝弃) → 规则2 含高分优先 → 规则3 平票选路径最短。
  §5 放置区访问顺序：写死偏好方向(不算真实距离)——
       左缝gap01靠左优先 / 右缝gap23靠右优先 / 中缝gap12中间优先 / 平票先左后右。
  §6 放左右：到区后区编号==左托盘编号发05 01(放左)，==右托盘编号发05 02(放右)。

back 排箱位(§8)：idx0 idx1 idx2 idx3 = 列0 列1 列2 列3(靠减速带那排)。
front 排(idx4-7)不搬。
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

# ── 归位区编号 → 名字（下标即编号，可被参数 zone_names 覆盖）──────────
#   0=食品绿 1=工具灰 2=仪器蓝 3=药品红
DEFAULT_ZONE_NAMES: Tuple[str, ...] = ("green", "gray", "blue", "red")
UNDETECTED = 0xFF

# ── 缝 → back 排相邻两箱（§4，场地几何常量）───────────────────────
#   left_idx = 缝左侧箱(小列号)，right_idx = 缝右侧箱(大列号)
GAP_BACK: Dict[str, Tuple[int, int]] = {
    "gap01": (0, 1),
    "gap12": (1, 2),
    "gap23": (2, 3),
}
ALL_GAPS: Tuple[str, ...] = ("gap01", "gap12", "gap23")


@dataclass
class Trip:
    """本趟计划：狗站 <gap>_back，抓左右两箱，再去两个区放置。"""
    gap: str                 # 'gap01' / 'gap12' / 'gap23'
    left_idx: int            # 左箱箱位（→ 左托盘 AB 04 01 BA）
    right_idx: int           # 右箱箱位（→ 右托盘 AB 04 02 BA）
    left_zone: int           # 左箱归位区编号 0-3（= 左托盘编号）
    right_zone: int          # 右箱归位区编号 0-3（= 右托盘编号）
    high: bool = False       # 本趟是否含高分箱
    place_order: Tuple[int, int] = (0, 0)  # 两个区的访问顺序（§5 写死偏好）

    @property
    def grab_point(self) -> str:
        return f"{self.gap}_back"        # 缝中抓取点，如 gap01_back

    @property
    def pre_point(self) -> str:
        return f"{self.gap}_pre"         # 缝前对准点

    @property
    def approach_point(self) -> str:
        return f"zone{self.gap[3:]}_approach"   # gap01→zone01_approach


class Strategy:
    """答案 + 8箱编号 → 单趟计划（选缝 + 放置区顺序）。"""

    def __init__(self,
                 zone_names: Sequence[str] = DEFAULT_ZONE_NAMES,
                 logger=None):
        """
        zone_names: 归位区编号(0-3) → 名字，决定放置路点名 place_<name>。
                    默认 0绿 1灰 2蓝 3红（与图册图10 左→右一致）。
        """
        self.zone_names = list(zone_names)
        self._log = logger

    # ── 小工具 ────────────────────────────────────────────────
    def _info(self, m):
        if self._log:
            self._log.info(m)

    def _warn(self, m):
        if self._log:
            self._log.warning(m)

    def zone_name(self, zone: int) -> str:
        if 0 <= zone < len(self.zone_names):
            return self.zone_names[zone]
        return ""

    def waypoint_for_zone(self, zone: int) -> Optional[str]:
        name = self.zone_name(zone)
        return f"place_{name}" if name else None

    # ── §4 选缝 ───────────────────────────────────────────────
    def plan_trip(self, answer: int, box_zones: Sequence[int]) -> Optional[Trip]:
        """
        输入：answer（高分区编号 0-3），box_zones（8 箱归位区编号，0xFF=未检测）
        输出：单趟 Trip；若没有可用的异色缝则返回 None。
        只看 back 排 idx0-3。
        """
        zones = list(box_zones)
        if len(zones) != 8:
            self._warn(f"箱编号数量异常: {len(zones)} (期望8)")
            zones = (zones + [UNDETECTED] * 8)[:8]
        back = zones[:4]     # idx0-3

        high_zone = int(answer) if 0 <= answer <= 3 else -1
        if high_zone < 0:
            self._warn(f"答案 {answer} 不在 0-3，视为无高分箱")
        self._info(f"高分区编号={high_zone}"
                   f"({self.zone_name(high_zone) or '无'}) "
                   f"back排编号={back}")

        # 逐缝打分：异色才可用；含高分+1分
        candidates = []   # (gap, score, li, ri)
        for gap in ALL_GAPS:
            li, ri = GAP_BACK[gap]
            lz, rz = back[li], back[ri]
            if lz == UNDETECTED or rz == UNDETECTED:
                self._info(f"{gap}: 含未检测箱(0xFF) → 跳过")
                continue
            if lz == rz:
                self._info(f"{gap}: 两箱同色({lz}) → 舍弃")
                continue
            score = 0
            if high_zone >= 0 and (lz == high_zone or rz == high_zone):
                score = 1
            candidates.append((gap, score, li, ri))
            self._info(f"{gap}: 异色 ✓ 左idx{li}={lz}({self.zone_name(lz)}) "
                       f"右idx{ri}={rz}({self.zone_name(rz)}) 分{score}")

        if not candidates:
            self._warn("没有可用的异色缝（全同色/含未检测）→ 无法规划本趟")
            return None

        # 规则2 高分优先 → 规则3 平票选路径最短（缝顺序即路径长短代理，
        # gap 序号越小越靠入场侧，视为更短；以 ALL_GAPS 顺序兜底）
        candidates.sort(key=lambda c: (-c[1], ALL_GAPS.index(c[0])))
        gap, score, li, ri = candidates[0]
        lz, rz = back[li], back[ri]

        trip = Trip(gap=gap, left_idx=li, right_idx=ri,
                    left_zone=lz, right_zone=rz, high=bool(score))
        trip.place_order = self._place_order(gap, lz, rz)
        self._info(f"选定 {gap}: 左idx{li}→区{lz}({self.zone_name(lz)}) "
                   f"右idx{ri}→区{rz}({self.zone_name(rz)}) "
                   f"{'★高分' if score else ''} | "
                   f"放置顺序 区{trip.place_order[0]}→区{trip.place_order[1]}")
        return trip

    # ── §5 放置区访问顺序（写死偏好方向）──────────────────────
    def _place_order(self, gap: str, lz: int, rz: int) -> Tuple[int, int]:
        """
        两个目标区(编号 lz, rz)按选缝的偏好方向排序，返回(先去, 后去)。
        归位区编号 0绿 1灰 2蓝 3红，左→右 = 0<1<2<3。
          左缝gap01 → 靠左优先(编号小的先)
          右缝gap23 → 靠右优先(编号大的先)
          中缝gap12 → 中间优先(1/2 先于 0/3)；平票(都中/都边)先左后右
        """
        a, b = lz, rz
        if gap == "gap01":
            return (a, b) if a < b else (b, a)          # 小编号先
        if gap == "gap23":
            return (a, b) if a > b else (b, a)          # 大编号先
        # gap12 中间优先：给每个区算"是否中间区"(1/2=True)
        def mid(z):
            return z in (1, 2)
        if mid(a) and not mid(b):
            return (a, b)
        if mid(b) and not mid(a):
            return (b, a)
        # 都中间 或 都靠边 → 先左后右(编号小的先)
        return (a, b) if a < b else (b, a)

    # ── §6 放左右：区编号 → 放哪侧托盘 ────────────────────────
    @staticmethod
    def place_side(zone: int, left_zone: int, right_zone: int) -> Optional[str]:
        """
        到某区(编号 zone)，比对左右托盘编号，返回 'left'/'right'（发05 01/05 02）。
        两托盘编号不同，必有且仅有一侧匹配；都不匹配返回 None（异常）。
        """
        if zone == left_zone:
            return "left"
        if zone == right_zone:
            return "right"
        return None
