#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_client —— 导航接口：路点查表 → navigate_to_pose goal → 阻塞等结果

对接 simple_nav_fsm（已验证跑通）：
  * action:  navigate_to_pose (nav2_msgs/action/NavigateToPose)
  * 路点文件: ~/.simple_nav_fsm_waypoints.yaml（record_waypoint 打点产物，
    §0.6.2 要求"直接复用这份已有的 yaml"）
    实际格式（record_waypoint 写出）:
        waypoints:
          entry: {x: 1.2345, y: 0.5678, yaw: 90.0, final_heading: true}
    兼容《主状态机.md》§7.4 的占位三元列表写法 [x, y, yaw]。
  * goal.behavior_tree = 'xy_only' → 不做终点朝向（fsm_node 的选项位）

线程模型：本类的 goto() 由 mission 线程调用（阻塞式）；
节点本身由 MultiThreadedExecutor 在主线程 spin，action 回调经
ReentrantCallbackGroup 执行，用 threading.Event 交接结果——
mission 线程里绝不 spin，避免和执行器打架。
"""
import math
import os
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

DEFAULT_WAYPOINTS_FILE = str(Path(os.path.expanduser("~"))
                             / ".simple_nav_fsm_waypoints.yaml")

_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


class NavClient:
    """路点导航客户端（阻塞式 goto，一次一个 goal，符合 FSM 用法）。"""

    def __init__(self, node,
                 waypoints_file: str = DEFAULT_WAYPOINTS_FILE,
                 dry_run: bool = False,
                 dry_run_nav_sec: float = 0.5):
        self._node = node
        self._log = node.get_logger()
        self.waypoints_file = os.path.expanduser(waypoints_file)
        self.dry_run = bool(dry_run)
        self._dry_sec = float(dry_run_nav_sec)
        self.waypoints: Dict[str, dict] = {}
        self.last_feedback_dist: Optional[float] = None
        self._active_handle = None
        self._cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose',
                                    callback_group=self._cb_group)
        self.reload()

    # ── 路点表 ───────────────────────────────────────────────
    def reload(self):
        """读取路点文件（可在打点后热重载）。"""
        self.waypoints = {}
        p = Path(self.waypoints_file)
        if not p.exists():
            self._log.warning(f"[nav] 路点文件不存在: {p} "
                              f"(先用 record_waypoint 打点)")
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
                # 兼容 §7.4 占位格式 [x, y, yaw(度)]
                self.waypoints[name] = {
                    'x': float(wp[0]), 'y': float(wp[1]),
                    'yaw': float(wp[2]) if len(wp) > 2 else 0.0,
                    'final_heading': True,
                }
        self._log.info(f"[nav] 已加载 {len(self.waypoints)} 个路点 "
                       f"来自 {p}")

    def has(self, name: str) -> bool:
        return name in self.waypoints

    def missing(self, names: List[str]) -> List[str]:
        return [n for n in names if n not in self.waypoints]

    # ── action server 就绪 ───────────────────────────────────
    def wait_server(self, timeout: float = 10.0) -> bool:
        if self.dry_run:
            return True
        ok = self._client.wait_for_server(timeout_sec=timeout)
        if not ok:
            self._log.error("[nav] navigate_to_pose action 不可用 —— "
                            "simple_nav_fsm 是否已启动？")
        return ok

    # ── 核心：阻塞式 goto ─────────────────────────────────────
    def goto(self, name: str, timeout: float = 30.0) -> bool:
        """
        导航到路点 name。成功返回 True。
        失败（拒绝/中止/超时/路点缺失）返回 False，超时会主动 cancel。
        """
        wp = self.waypoints.get(name)
        if wp is None:
            if self.dry_run:
                self._log.warning(f"[nav][dry] 路点 '{name}' 未打点，"
                                  f"仿真直接视为到达")
                time.sleep(self._dry_sec)
                return True
            self._log.error(f"[nav] 路点 '{name}' 不存在于 "
                            f"{self.waypoints_file}")
            return False

        if self.dry_run:
            self._log.info(f"[nav][dry] goto {name} "
                           f"(x={wp['x']:.2f} y={wp['y']:.2f} "
                           f"yaw={wp['yaw']:.0f}°) ... {self._dry_sec:.1f}s")
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
            goal.behavior_tree = 'xy_only'   # fsm_node 约定：不做终点朝向

        self._log.info(f"[nav] goto {name}: x={wp['x']:.3f} y={wp['y']:.3f} "
                       f"yaw={wp['yaw']:.1f}° "
                       f"终点朝向={'否' if not wp['final_heading'] else '是'} "
                       f"超时={timeout:.0f}s")
        self.last_feedback_dist = None

        accept_evt, result_evt = threading.Event(), threading.Event()
        box = {'handle': None, 'status': None}

        def _fb(msg):
            self.last_feedback_dist = msg.feedback.distance_remaining

        def _on_response(fut):
            gh = fut.result()
            box['handle'] = gh if gh.accepted else None
            accept_evt.set()
            if gh.accepted:
                gh.get_result_async().add_done_callback(_on_result)

        def _on_result(fut):
            box['status'] = fut.result().status
            result_evt.set()

        send_fut = self._client.send_goal_async(goal, feedback_callback=_fb)
        send_fut.add_done_callback(_on_response)

        if not accept_evt.wait(timeout=5.0):
            self._log.error("[nav] 发 goal 后 5s 无响应（FSM 没在跑？）")
            return False
        if box['handle'] is None:
            self._log.error("[nav] goal 被 FSM 拒绝")
            return False
        self._active_handle = box['handle']

        deadline = time.monotonic() + timeout
        while not result_evt.wait(timeout=0.2):
            if time.monotonic() > deadline:
                self._log.error(f"[nav] goto {name} 超时({timeout:.0f}s)，"
                                f"取消 goal")
                self.cancel()
                result_evt.wait(timeout=3.0)   # 等 cancel 落地
                self._active_handle = None
                return False
        self._active_handle = None

        status = box['status']
        ok = (status == GoalStatus.STATUS_SUCCEEDED)
        name_s = _STATUS_NAMES.get(status, str(status))
        if ok:
            self._log.info(f"[nav] goto {name} 成功")
        else:
            self._log.error(f"[nav] goto {name} 失败 status={name_s}")
        return ok

    def cancel(self):
        """取消当前 goal（超时/急停时用）。"""
        gh = self._active_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self._log.warning(f"[nav] cancel 异常: {e}")
