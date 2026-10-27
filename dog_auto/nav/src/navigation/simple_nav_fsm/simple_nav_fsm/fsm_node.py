#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_nav_fsm —— Y → X → Yaw 逐轴顺序校准的极简导航状态机
（替代 Nav2 的 planner_server / bt_navigator / controller_server / costmap 这一层）

三阶段逐轴校准：
  * CALIBRATE_Y: vy 逼近 gy，vx/wz 被动维持阶段起始 x 和 yaw
  * CALIBRATE_X: vx 逼近 gx，vy/wz 被动维持阶段起始 y 和 yaw
  * CALIBRATE_YAW: wz 逼近 gyaw，vx/vy 被动维持阶段起始 x 和 y
  * 全闭环判定：误差在容差内"连续 N 帧"才算到位，没有任何开环时长标定
  * 主动轴大增益快速逼近，被动轴小增益冻结漂移
  * 角度差全部走 wrap-around 归一化，跨 ±180° 不会炸出假误差
  * TF 显式检查新鲜度，拿不到新鲜数据 → SAFETY_STOP，绝不拿旧数据继续算
  * 失败看门狗：状态超时 + 无进展检测
  * SAFETY_STOP 恢复后一律回 CALIBRATE_Y 重来，不带陈旧状态继续跑
  * 对外接口复用 nav2_msgs/action/NavigateToPose：上层"搬箱子"逻辑零迁移成本；
    goal.behavior_tree 字段被借用为选项位："xy_only" = 不做终点朝向
"""
import math
import threading
import time
from collections import deque
from enum import Enum

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import SetParametersResult
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


def norm_ang(a: float) -> float:
    """角度归一化到 (-pi, pi]。所有角度差必须先过这里。"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q) -> float:
    """四元数 -> yaw（平面机器人只关心 yaw）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class NavState(Enum):
    IDLE = 0
    CALIBRATE_Y = 1
    CALIBRATE_X = 2
    CALIBRATE_YAW = 3
    SAFETY_STOP = 4


# 所有参数集中声明，支持 `ros2 param set /simple_nav_fsm <name> <value>` 在线调
# 默认值为哨兵/小车平台；狗平台请用 config/params_dog.yaml
PARAM_DEFAULTS = {
    # ---- 框架 / 接口 ----
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'cmd_vel_topic': '/cmd_vel_raw',   # ★发到 guard 输入侧, 不是 guard 输出话题 /cmd_vel
    'estop_topic': '/nav_estop',          # std_msgs/Bool, true = 急停
    'control_rate': 30.0,                 # Hz 状态机主循环
    'tf_timeout': 0.5,                    # s TF 新鲜度阈值（按 map->odom 实际发布率调）
    'default_final_heading': True,        # goal 未指明时是否做终点朝向
    # ---- 容差 / 到位判定 ----
    'xy_tol': 0.04,                       # m
    'yaw_tol_deg': 4.0,                   # deg，必须大于 trot 抖动带 (±2~3°)
    'settle_frames': 8,                   # 连续 N 帧在容差内才算到位
    # ---- 主动轴增益（逼近目标）----
    'kv': 0.8,                            # vx/vy 主增益（比例减速）
    'v_max': 0.4,                         # m/s
    'v_min': 0.06,                        # m/s 防止在容差边界外无限龟速蹭
    'vy_max': 0.12,                       # m/s, Phase 2 (Y) 主动侧移上限
    'vy_min': 0.06,                       # m/s
    'kp_rotate': 1.8,                     # Phase 3 (YAW) 主动旋转增益
    'wz_max': 0.8,                        # rad/s
    'wz_min': 0.15,                       # rad/s 克服步态死区的最小转速
    # ---- 被动轴纠偏增益（冻结维持）----
    'k_hold_pos': 0.5,                    # xy 位置被动纠偏增益 (<< kv)
    'k_hold_yaw': 0.5,                    # yaw 被动纠偏增益 (<< kp_rotate)
    'vx_hold_max': 0.1,                   # m/s, vx 被动纠偏上限
    'vy_hold_max': 0.08,                  # m/s, vy 被动纠偏上限
    'wz_hold_max': 0.25,                  # rad/s, wz 被动纠偏上限
    # ---- 指令斜坡（加速度限幅，别猛拽步态）----
    'acc_x': 0.5,
    'acc_y': 0.5,
    'acc_wz': 2.0,
    # ---- 失败看门狗 ----
    'progress_window': 3.0,               # s 无进展检测窗口
    'min_xy_progress': 0.05,              # m/窗口
    'min_yaw_progress_deg': 3.0,          # deg/窗口
    'x_timeout': 30.0,                    # s, CALIBRATE_X 超时
    'y_timeout': 30.0,                    # s, CALIBRATE_Y 超时
    'yaw_timeout': 25.0,                  # s, CALIBRATE_YAW 超时
    # ---- SAFETY_STOP ----
    'recover_frames': 15,                 # 条件解除连续 N 帧才恢复（防抖）
    'safety_abort_timeout': 20.0,         # s 安全停太久不恢复 -> 报失败给上层
}


class SimpleNavFsm(Node):

    def __init__(self):
        super().__init__('simple_nav_fsm')

        self.p = {}
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
            self.p[name] = self.get_parameter(name).value
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self._estop = False
        self.create_subscription(Bool, self.p['estop_topic'], self._estop_cb, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._goal_lock = threading.Lock()
        self._execute_lock = threading.Lock()   # 串行化：同一时刻只跑一个 goal
        self._current_handle = None
        self._preempt = False
        self._live_state = 'IDLE'

        # 指令斜坡记忆
        self._last_cmd = [0.0, 0.0, 0.0]
        self._last_cmd_t = time.monotonic()

        self._server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            handle_accepted_callback=self._accepted_cb,
            callback_group=ReentrantCallbackGroup())

        self.create_timer(1.0, self._idle_state_pub)
        self.get_logger().info(
            'simple_nav_fsm 就绪 | action: navigate_to_pose (nav2_msgs/NavigateToPose) | '
            f"cmd: {self.p['cmd_vel_topic']} | estop: {self.p['estop_topic']}")

    # ------------------------------------------------------------------ 基础
    def _on_params(self, params):
        for prm in params:
            if prm.name in self.p:
                self.p[prm.name] = prm.value
                self.get_logger().info(f'参数在线更新: {prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    def _estop_cb(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().warn('收到外部急停 -> SAFETY_STOP')
        if (not msg.data) and self._estop:
            self.get_logger().info('外部急停解除')
        self._estop = bool(msg.data)

    def _idle_state_pub(self):
        if self._current_handle is None:
            self.state_pub.publish(String(data='IDLE'))

    def get_pose(self):
        """map 系下 (x, y, yaw)。TF 拿不到或过旧一律返回 None——不拿旧数据继续算。"""
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.p['map_frame'], self.p['base_frame'], Time())
        except TransformException as e:
            self.get_logger().warn(f'TF 不可用: {e}', throttle_duration_sec=2.0)
            return None
        age = (self.get_clock().now() - Time.from_msg(tfm.header.stamp)).nanoseconds * 1e-9
        if age > float(self.p['tf_timeout']):
            self.get_logger().warn(
                f"TF 过旧 ({age:.2f}s > {self.p['tf_timeout']}s)", throttle_duration_sec=2.0)
            return None
        t = tfm.transform.translation
        return t.x, t.y, yaw_of(tfm.transform.rotation)

    def send_cmd(self, vx, vy, wz):
        """带加速度限幅的速度指令。body 系：vx 前进 / vy 左 / wz 逆时针。"""
        now = time.monotonic()
        dt = clamp(now - self._last_cmd_t, 1e-3, 0.2)
        p = self.p
        vx = clamp(vx, self._last_cmd[0] - float(p['acc_x']) * dt,
                   self._last_cmd[0] + float(p['acc_x']) * dt)
        vy = clamp(vy, self._last_cmd[1] - float(p['acc_y']) * dt,
                   self._last_cmd[1] + float(p['acc_y']) * dt)
        wz = clamp(wz, self._last_cmd[2] - float(p['acc_wz']) * dt,
                   self._last_cmd[2] + float(p['acc_wz']) * dt)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)
        self._last_cmd = [vx, vy, wz]
        self._last_cmd_t = now

    def send_stop(self):
        """立即置零（不走斜坡），并复位斜坡记忆。状态退出/急停/goal 结束都调它。"""
        self.cmd_pub.publish(Twist())
        self._last_cmd = [0.0, 0.0, 0.0]
        self._last_cmd_t = time.monotonic()

    # ------------------------------------------------------------ action 骨架
    def _goal_cb(self, request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _accepted_cb(self, goal_handle):
        # 抢占语义（与 Nav2 一致）：新 goal 到来 -> 旧 goal 中止，新 goal 接管
        with self._goal_lock:
            if self._current_handle is not None:
                self.get_logger().warn('新 goal 抢占当前 goal')
                self._preempt = True
        goal_handle.execute()

    def _execute_cb(self, goal_handle):
        with self._execute_lock:
            with self._goal_lock:
                self._current_handle = goal_handle
                self._preempt = False
            try:
                return self._run(goal_handle)
            finally:
                self.send_stop()
                with self._goal_lock:
                    if self._current_handle is goal_handle:
                        self._current_handle = None
                self._live_state = 'IDLE'

    # ------------------------------------------------------------ 状态机主体
    def _run(self, gh):
        p = self.p
        log = self.get_logger()
        req = gh.request

        # ---- goal 解析 ----
        frame = req.pose.header.frame_id or p['map_frame']
        if frame != p['map_frame']:
            log.error(f"goal frame '{frame}' != map_frame '{p['map_frame']}'，"
                      '请在 map 系下发 goal')
            gh.abort()
            return NavigateToPose.Result()
        gx = req.pose.pose.position.x
        gy = req.pose.pose.position.y
        gyaw = yaw_of(req.pose.pose.orientation)

        bt = (req.behavior_tree or '').strip()
        if bt == 'xy_only':
            need_final = False
        elif bt == 'with_final_heading':
            need_final = True
        else:
            need_final = bool(p['default_final_heading'])

        log.info(f'新 goal: x={gx:.3f} y={gy:.3f} yaw={math.degrees(gyaw):.1f} deg | '
                 f'终点朝向: {"要" if need_final else "不要"}')

        # ---- 运行时状态 ----
        start_stamp = self.get_clock().now()
        state = NavState.CALIBRATE_Y
        state_t0 = time.monotonic()
        settle = 0
        safety_ok = 0
        safety_reason = ''
        prog = deque()                       # (t, x, y, yaw) 进展看门狗采样，状态切换即清空
        # 各阶段冻结参考值（阶段开始时记录）
        x_ref = y_ref = yaw_ref = 0.0
        tick = 0
        dbg = ''

        def enter(new_state, why=''):
            nonlocal state, state_t0, settle, safety_ok
            log.info(f'{state.name} -> {new_state.name}' + (f'  ({why})' if why else ''))
            state = new_state
            state_t0 = time.monotonic()
            settle = 0
            safety_ok = 0
            prog.clear()
            self.send_stop()                 # 切状态先归零一拍，杜绝残留指令

        def finish(kind, msg):
            self.send_stop()
            if kind == 'success':
                log.info(f'goal 完成: {msg}')
                gh.succeed()
            elif kind == 'canceled':
                log.info(f'goal 取消: {msg}')
                gh.canceled()
            else:
                log.error(f'goal 失败: {msg}')
                gh.abort()
            return NavigateToPose.Result()

        rate_dt = 1.0 / max(float(p['control_rate']), 1.0)

        # ---- 主循环：读定位 -> 算误差 -> 判定/出指令 ----
        while rclpy.ok():
            if gh.is_cancel_requested:
                return finish('canceled', '上层取消')
            if self._preempt:
                return finish('abort', '被新 goal 抢占')

            pose = self.get_pose()
            estopped = self._estop

            # TF 丢失/过旧 或 外部急停 -> 强制切 SAFETY_STOP（优先级最高）
            if state != NavState.SAFETY_STOP and (pose is None or estopped):
                safety_reason = '外部急停' if estopped else 'TF 丢失/过旧'
                enter(NavState.SAFETY_STOP, safety_reason)

            x = y = yaw = None
            if pose is not None:
                x, y, yaw = pose

            now_mono = time.monotonic()
            in_state = now_mono - state_t0
            xy_tol = float(p['xy_tol'])
            yaw_tol = math.radians(float(p['yaw_tol_deg']))

            if state == NavState.SAFETY_STOP:
                self.send_stop()             # 按控制频率持续发零速
                dbg = f'SAFETY_STOP({safety_reason}) {in_state:.1f}s'
                if pose is not None and not estopped:
                    safety_ok += 1
                    if safety_ok >= int(p['recover_frames']):
                        # 恢复后回 CALIBRATE_Y 重新开始
                        enter(NavState.CALIBRATE_Y, '安全条件解除，重新校准')
                else:
                    safety_ok = 0
                if in_state > float(p['safety_abort_timeout']):
                    return finish('abort', f'SAFETY_STOP 超时未恢复（{safety_reason}）')

            else:
                # 非 SAFETY 分支保证 pose 非 None
                prog.append((now_mono, x, y, yaw))
                while len(prog) > 2 and prog[1][0] <= now_mono - float(p['progress_window']):
                    prog.popleft()

                # ============================================================
                # Phase 1: CALIBRATE_Y —— vy 主动逼近 gy，vx/wz 被动维持 x_ref/yaw_ref
                # ============================================================
                if state == NavState.CALIBRATE_Y:
                    # 被动轴冻结阶段开始时的当前位姿，未对齐前不跟踪goal
                    if settle == 0 and len(prog) <= 1:
                        y_ref, x_ref, yaw_ref = y, x, yaw
                        log.info(f'CALIBRATE_Y 冻结参考: y_ref={y_ref:.3f} x_ref={x_ref:.3f} '
                                 f'yaw_ref={math.degrees(yaw_ref):.1f}deg')
                    err_y = gy - y
                    err_x = x_ref - x
                    err_yaw = norm_ang(yaw - yaw_ref)   # ★yaw方向取反
                    dbg = (f'CALIBRATE_Y err_y={err_y:+.3f} '
                           f'hold_x={err_x:+.3f} hold_yaw={math.degrees(err_yaw):+.1f}deg')

                    if abs(err_y) <= xy_tol:
                        settle += 1
                        self.send_cmd(0.0, 0.0, 0.0)
                        if settle >= int(p['settle_frames']):
                            enter(NavState.CALIBRATE_X, f'y 就位 (残差 {err_y:+.3f}m)')
                    else:
                        settle = 0
                        # 主动: vy 逼近 gy
                        vy = clamp(float(p['kv']) * err_y, -float(p['vy_max']), float(p['vy_max']))
                        if abs(vy) < float(p['vy_min']):
                            vy = math.copysign(float(p['vy_min']), err_y)
                        # 被动: vx 维持 x_ref（err_x = x_ref - x，同号拉回）
                        vx = clamp(float(p['k_hold_pos']) * err_x,
                                   -float(p['vx_hold_max']), float(p['vx_hold_max']))
                        # 被动: wz 维持 yaw_ref
                        wz = clamp(float(p['k_hold_yaw']) * err_yaw,
                                   -float(p['wz_hold_max']), float(p['wz_hold_max']))
                        self.send_cmd(vx, vy, wz)
                        # 无进展看门狗
                        if in_state > float(p['progress_window']) and len(prog) >= 2:
                            span = now_mono - prog[0][0]
                            moved = abs(y - prog[0][2])
                            if span >= float(p['progress_window']) and \
                               moved < float(p['min_xy_progress']):
                                return finish('abort', 'CALIBRATE_Y 无进展——机器人可能被阻挡')
                        if in_state > float(p['y_timeout']):
                            return finish('abort', f'CALIBRATE_Y 超时 ({in_state:.1f}s)')

                # ============================================================
                # Phase 2: CALIBRATE_X —— vx 主动逼近 gx，vy/wz 被动维持 y_ref/yaw_ref
                # ============================================================
                elif state == NavState.CALIBRATE_X:
                    # 被动轴冻结阶段开始时的当前位姿，未对齐前不跟踪goal
                    if settle == 0 and len(prog) <= 1:
                        x_ref, y_ref, yaw_ref = x, y, yaw
                        log.info(f'CALIBRATE_X 冻结参考: x_ref={x_ref:.3f} '
                                 f'yaw_ref={math.degrees(yaw_ref):.1f}deg')
                    err_x = gx - x
                    err_y = y_ref - y
                    err_yaw = norm_ang(yaw - yaw_ref)   # ★yaw方向取反
                    dbg = (f'CALIBRATE_X err_x={err_x:+.3f} '
                           f'hold_y={err_y:+.3f} hold_yaw={math.degrees(err_yaw):+.1f}deg')

                    if abs(err_x) <= xy_tol:
                        settle += 1
                        self.send_cmd(0.0, 0.0, 0.0)
                        if settle >= int(p['settle_frames']):
                            if need_final:
                                enter(NavState.CALIBRATE_YAW, f'x 就位 (残差 {err_x:+.3f}m)')
                            else:
                                return finish('success',
                                              f'xy 就位 (x残差 {abs(gx - x):.3f}m, '
                                              f'y残差 {abs(gy - y):.3f}m)，无需终点朝向')
                    else:
                        settle = 0
                        # 主动: vx 逼近 gx
                        vx = clamp(float(p['kv']) * err_x, -float(p['v_max']), float(p['v_max']))
                        if abs(vx) < float(p['v_min']):
                            vx = math.copysign(float(p['v_min']), err_x)
                        # 被动: vy 维持 y_ref（err_y = y_ref - y，同号拉回）
                        vy = clamp(float(p['k_hold_pos']) * err_y,
                                   -float(p['vy_hold_max']), float(p['vy_hold_max']))
                        # 被动: wz 维持 yaw_ref
                        wz = clamp(float(p['k_hold_yaw']) * err_yaw,
                                   -float(p['wz_hold_max']), float(p['wz_hold_max']))
                        self.send_cmd(vx, vy, wz)
                        if in_state > float(p['progress_window']) and len(prog) >= 2:
                            span = now_mono - prog[0][0]
                            moved = abs(x - prog[0][1])
                            if span >= float(p['progress_window']) and \
                               moved < float(p['min_xy_progress']):
                                return finish('abort', 'CALIBRATE_X 无进展——机器人可能被阻挡')
                        if in_state > float(p['x_timeout']):
                            return finish('abort', f'CALIBRATE_X 超时 ({in_state:.1f}s)')

                # ============================================================
                # Phase 3: CALIBRATE_YAW —— wz 主动逼近 gyaw，vx/vy 被动维持 x_ref/y_ref
                # ============================================================
                elif state == NavState.CALIBRATE_YAW:
                    # 被动轴冻结阶段开始时的当前位姿，未对齐前不跟踪goal
                    if settle == 0 and len(prog) <= 1:
                        x_ref, y_ref = x, y
                        log.info(f'CALIBRATE_YAW 冻结参考: x_ref={x_ref:.3f} y_ref={y_ref:.3f}')
                    err_yaw = norm_ang(yaw - gyaw)   # ★yaw方向取反试验
                    err_x = x_ref - x
                    err_y = y_ref - y
                    dbg = (f'CALIBRATE_YAW err_yaw={math.degrees(err_yaw):+.1f}deg '
                           f'hold_x={err_x:+.3f} hold_y={err_y:+.3f}')

                    if abs(err_yaw) <= yaw_tol:
                        settle += 1
                        self.send_cmd(0.0, 0.0, 0.0)
                        if settle >= int(p['settle_frames']):
                            return finish('success',
                                          f'到位: x残差 {abs(gx - x):.3f}m, '
                                          f'y残差 {abs(gy - y):.3f}m, '
                                          f'yaw残差 {math.degrees(err_yaw):+.1f}deg')
                    else:
                        settle = 0
                        # 主动: wz 逼近 gyaw
                        wz = clamp(float(p['kp_rotate']) * err_yaw,
                                   -float(p['wz_max']), float(p['wz_max']))
                        if abs(err_yaw) > yaw_tol and abs(wz) < float(p['wz_min']):
                            wz = math.copysign(float(p['wz_min']), err_yaw)
                        # 被动: vx 维持 x_ref（err_x = x_ref - x，同号拉回）
                        vx = clamp(float(p['k_hold_pos']) * err_x,
                                   -float(p['vx_hold_max']), float(p['vx_hold_max']))
                        # 被动: vy 维持 y_ref（err_y = y_ref - y，同号拉回）
                        vy = clamp(float(p['k_hold_pos']) * err_y,
                                   -float(p['vy_hold_max']), float(p['vy_hold_max']))
                        self.send_cmd(vx, vy, wz)
                        # 无进展看门狗
                        if in_state > float(p['progress_window']) and len(prog) >= 2:
                            span = now_mono - prog[0][0]
                            dyaw = abs(norm_ang(yaw - prog[0][3]))
                            if span >= float(p['progress_window']) and \
                               dyaw < math.radians(float(p['min_yaw_progress_deg'])):
                                return finish('abort',
                                    'CALIBRATE_YAW 无进展——检查 yaw 控制权/是否被卡住')
                        if in_state > float(p['yaw_timeout']):
                            return finish('abort', f'CALIBRATE_YAW 超时 ({in_state:.1f}s)')

            # ---- 反馈（当前状态/剩余误差）与调试话题 ----
            self._live_state = dbg or state.name
            if tick % max(1, int(float(p['control_rate']) / 10.0)) == 0:
                self.state_pub.publish(String(data=f'{dbg or state.name}'))
            if tick % max(1, int(float(p['control_rate']) / 5.0)) == 0 and pose is not None:
                fb = NavigateToPose.Feedback()
                fb.current_pose.header.frame_id = p['map_frame']
                fb.current_pose.header.stamp = self.get_clock().now().to_msg()
                fb.current_pose.pose.position.x = x
                fb.current_pose.pose.position.y = y
                fb.current_pose.pose.orientation.z = math.sin(yaw / 2.0)
                fb.current_pose.pose.orientation.w = math.cos(yaw / 2.0)
                fb.distance_remaining = float(math.hypot(gx - x, gy - y))
                fb.navigation_time = (self.get_clock().now() - start_stamp).to_msg()
                fb.number_of_recoveries = 0
                gh.publish_feedback(fb)

            tick += 1
            time.sleep(rate_dt)

        return finish('abort', 'rclpy 已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavFsm()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
