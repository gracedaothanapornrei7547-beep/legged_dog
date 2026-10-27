#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线自测（v3.2，不依赖 ROS/串口/硬件）。

跑法：
    cd ~/dog_auto/dicision/src/mission_director
    python3 test/test_offline.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mission_director.arm_link import (  # noqa: E402
    FrameParser, build_state_cmd,
    S2_PLACE_LEFT, S2_PLACE_RIGHT, ZONE_UNDETECTED)
from mission_director.strategy import Strategy, GAP_BACK  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ✓ {name}")
    else:
        FAIL += 1
        print(f"  ✗ {name}  {detail}")


def feed_all(p, data):
    out = []
    for b in data:
        r = p.feed(b)
        if r:
            out.append(r)
    return out


def test_parser():
    print("[1] 协议编解码（v3.2）")
    p = FrameParser()

    r = feed_all(p, bytes([0xAB, 0x04, 0x02, 0x02, 0xBA]))
    check("状态帧 抓右完成", r == [{'kind': 'state', 'state1': 4,
                                    'state2': 2, 'state3': 2}], str(r))

    # 数字帧: 答案(高分区)=2, 8箱编号
    r = feed_all(p, bytes([0xCD, 0x02, 0, 2, 1, 3, 2, 1, 0, 3, 0xDC]))
    check("数字帧 答案2+8箱编号",
          r and r[0]['kind'] == 'digit' and r[0]['answer'] == 2
          and r[0]['box_zones'] == [0, 2, 1, 3, 2, 1, 0, 3], str(r))

    # 数字帧含未检测 0xFF
    r = feed_all(p, bytes([0xCD, 0x00, 0xFF, 2, 1, 3, 0, 1, 2, 3, 0xDC]))
    check("数字帧 含未检测0xFF",
          r and r[0]['box_zones'][0] == ZONE_UNDETECTED, str(r))

    # 命令帧 + 坏帧重同步
    check("命令帧 抓右", build_state_cmd(0x04, 0x02).hex() == "ab0402ba")
    check("放左托盘命令", build_state_cmd(0x05, S2_PLACE_LEFT).hex() == "ab0501ba")
    check("放右托盘命令", build_state_cmd(0x05, S2_PLACE_RIGHT).hex() == "ab0502ba")
    bad = bytes([0xAB, 0x01, 0x00, 0x01, 0xFF])
    good = bytes([0xAB, 0x01, 0x00, 0x02, 0xBA])
    r = feed_all(p, bad + good)
    check("坏帧丢弃后重同步", r == [{'kind': 'state', 'state1': 1,
                                     'state2': 0, 'state3': 2}]
          and p.dropped >= 1, f"r={r} dropped={p.dropped}")


def test_select_gap():
    print("[2] 选缝：异色→高分→平票最短")
    s = Strategy()
    # 答案=2(蓝高分), back=[0绿,2蓝,1灰,3红]
    t = s.plan_trip(answer=2, box_zones=[0, 2, 1, 3, 2, 1, 0, 3])
    # gap01=绿0+蓝2异色含高分, gap12=蓝2+灰1异色含高分, gap23=灰1+红3异色无高分
    # 高分优先→gap01/gap12, 平票取序号小→gap01
    check("选 gap01(异色+高分+最短)", t is not None and t.gap == "gap01", str(t))
    check("左idx0→区0 右idx1→区2",
          t.left_zone == 0 and t.right_zone == 2, str(t))
    check("标记高分", t.high)

    # 同色缝要跳过: back=[0,0,1,3] → gap01同色(0,0)弃
    t = s.plan_trip(answer=-1, box_zones=[0, 0, 1, 3, 0, 0, 0, 0])
    check("gap01同色被弃→选gap12", t is not None and t.gap == "gap12",
          str(t.gap if t else None))

    # 全同色/无异色 → None
    t = s.plan_trip(answer=-1, box_zones=[0, 0, 0, 0, 0, 0, 0, 0])
    check("全同色→无可用缝(None)", t is None)

    # 含未检测: back=[0,0xFF,1,3] → gap01含FF弃、gap12含FF弃 → gap23
    t = s.plan_trip(answer=-1, box_zones=[0, ZONE_UNDETECTED, 1, 3, 0, 0, 0, 0])
    check("含0xFF的缝被弃→选gap23", t is not None and t.gap == "gap23",
          str(t.gap if t else None))


def test_place_order():
    print("[3] 放置区访问顺序（写死偏好方向）")
    s = Strategy()
    # 左缝靠左优先: gap01+{区3,区2} → 先2(小)后3
    t = s.plan_trip(answer=-1, box_zones=[3, 2, 0, 1, 0, 0, 0, 0])
    # gap01=区3+区2异色 → 选它(序号最小), 靠左优先→(2,3)
    check("gap01靠左优先 区3区2→先2后3",
          t.gap == "gap01" and t.place_order == (2, 3), str(t.place_order))

    # 右缝靠右优先: 构造只有 gap23 可选且为 {区1,区0}
    # back=[0,0,1,0]→gap01(0,0)弃 gap12(0,1)异色 gap23(1,0)异色
    # 无高分→平票取序号小 gap12. 需要单独测 gap23 的 order:
    check("gap23靠右优先 区1区0→先1后0",
          s._place_order("gap23", 1, 0) == (1, 0))
    check("gap23靠右优先 区0区3→先3后0",
          s._place_order("gap23", 0, 3) == (3, 0))

    # 中缝中间优先
    check("gap12 区2(中)区0(边)→先2后0",
          s._place_order("gap12", 2, 0) == (2, 0))
    check("gap12 区1区2都中→先左后右(先1)",
          s._place_order("gap12", 2, 1) == (1, 2))
    check("gap12 区0区3都边→先左后右(先0)",
          s._place_order("gap12", 3, 0) == (0, 3))


def test_place_side():
    print("[4] 放左/放右：区编号比对托盘编号")
    check("区==左托盘→left",
          Strategy.place_side(zone=0, left_zone=0, right_zone=2) == "left")
    check("区==右托盘→right",
          Strategy.place_side(zone=2, left_zone=0, right_zone=2) == "right")
    check("都不匹配→None",
          Strategy.place_side(zone=1, left_zone=0, right_zone=2) is None)


def test_gap_back():
    print("[5] 缝-箱映射常量自洽")
    check("gap01→idx0,1", GAP_BACK["gap01"] == (0, 1))
    check("gap12→idx1,2", GAP_BACK["gap12"] == (1, 2))
    check("gap23→idx2,3", GAP_BACK["gap23"] == (2, 3))


if __name__ == "__main__":
    test_parser()
    test_select_gap()
    test_place_order()
    test_place_side()
    test_gap_back()
    print(f"\n结果: {PASS} 通过 / {FAIL} 失败")
    sys.exit(1 if FAIL else 0)
