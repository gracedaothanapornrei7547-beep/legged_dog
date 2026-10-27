#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_director_node —— 节点入口 + 参数声明

组装：ArmLink + NavClient + Strategy → Mission，主流程在后台线程顺序执行，
主线程用 MultiThreadedExecutor spin（action 回调 / 微调定时器都跑在这）。

控制接口：
  参数 auto_start=true  → 起来 start_delay 秒后自动开跑（比赛用）
  服务 ~/start (std_srvs/Trigger)  → 手动开跑（auto_start=false 时）
  服务 ~/abort (std_srvs/Trigger)  → 中止：取消导航、微调归零、流程收尾回家跳过
话题：
  ~/phase (std_msgs/String)        → 当前阶段（S0_INIT / TRIP1_gap01_back / ...）
"""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .arm_link import ArmLink
from .mission import Mission
from .nav_client import DEFAULT_WAYPOINTS_FILE, NavClient
from .strategy import Strategy


class MissionDirector(Node):
    def __init__(self):
        super().__init__("mission_director")
        p = self._declare_params()
        log = self.get_logger()

        arm = ArmLink(
            log,
            port=p["serial_port"], baud=p["serial_baud"],
            dry_run=p["dry_run"],
            dry_run_answer=p["dry_run_answer"],
            dry_run_box_zones=p["dry_run_box_zones"],
            dry_run_step_sec=p["dry_run_step_sec"],
        )
        nav = NavClient(
            self,
            waypoints_file=p["waypoints_file"],
            dry_run=p["dry_run"],
            dry_run_nav_sec=p["dry_run_nav_sec"],
        )
        strategy = Strategy(
            zone_names=p["zone_names"],
            logger=log,
        )
        self.mission = Mission(self, arm, nav, strategy, p)
        self._arm = arm
        self._thread = None

        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/abort", self._srv_abort)

        if p["dry_run"]:
            log.warning("★★ DRY-RUN 模式：不开串口、导航仅仿真 ★★")
        if p["auto_start"]:
            log.info(f"auto_start=true：{p['start_delay']:.1f}s 后开跑 "
                     f"(手动模式请设 auto_start:=false 并调 ~/start)")
            self._start_timer = self.create_timer(
                max(p["start_delay"], 0.05), self._auto_start)
        else:
            log.info("auto_start=false：等待服务调用 "
                     "`ros2 service call /mission_director/start "
                     "std_srvs/srv/Trigger`")

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
            target=self.mission.run, name="mission", daemon=True)
        self._thread.start()

    def shutdown(self):
        self.mission.request_abort()
        self._arm.close()

    # ── 参数声明（命名对齐《主状态机.md》§7.6）──────────────────
    def _declare_params(self) -> dict:
        d = self.declare_parameter
        p = {}
        # ---- 串口 ----
        p["serial_port"] = d("serial_port", "/dev/ttyX5").value
        p["serial_baud"] = d("serial_baud", 115200).value
        # ---- 协议可配置项（v3.2 归位区编号）----
        #   0=食品绿 1=工具灰 2=仪器蓝 3=药品红
        p["zone_names"] = list(d(
            "zone_names", ["green", "gray", "blue", "red"]).value)
        # ---- 抓取顺序 ----
        p["grab_first"] = d("grab_first", "right").value
        # ---- 时间/超时 ----
        p["nav_timeout_sec"] = float(d("nav_timeout_sec", 30.0).value)
        p["grab_timeout_sec"] = float(d("grab_timeout_sec", 40.0).value)
        p["place_timeout_sec"] = float(d("place_timeout_sec", 40.0).value)
        p["arm_timeout"] = float(d("arm_timeout", 180.0).value)
        p["nav_server_wait_sec"] = float(d("nav_server_wait_sec", 180.0).value)
        # ---- 重试/恢复 ----
        p["nav_retry"] = int(d("nav_retry", 1).value)
        p["grab_retry"] = int(d("grab_retry", 1).value)
        p["place_retry"] = int(d("place_retry", 1).value)
        p["max_recover"] = int(d("max_recover", 3).value)
        # ---- 路点 / 导航 ----
        p["waypoints_file"] = d(
            "waypoints_file", DEFAULT_WAYPOINTS_FILE).value
        p["strict_waypoints"] = bool(d("strict_waypoints", True).value)
        # ---- 触发 / 仿真 ----
        p["auto_start"] = bool(d("auto_start", True).value)
        p["start_delay"] = float(d("start_delay", 2.0).value)
        p["dry_run"] = bool(d("dry_run", False).value)
        p["dry_run_answer"] = int(d("dry_run_answer", 2).value)
        p["dry_run_box_zones"] = [int(v) for v in d(
            "dry_run_box_zones", [0, 2, 1, 3, 2, 1, 0, 3]).value]
        p["dry_run_step_sec"] = float(d("dry_run_step_sec", 0.3).value)
        p["dry_run_nav_sec"] = float(d("dry_run_nav_sec", 0.5).value)
        return p


def main(args=None):
    rclpy.init(args=args)
    node = MissionDirector()
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
