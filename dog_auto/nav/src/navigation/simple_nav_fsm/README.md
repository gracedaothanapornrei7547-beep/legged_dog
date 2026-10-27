# simple_nav_fsm

直线 + 原地转向的极简导航状态机，**替代 Nav2 的 planner_server / bt_navigator / controller_server / costmap 层**。

定位（point_lio + small_gicp_relocalization）和底层狗控制（dog_control / shm）完全不动。

## 核心思想：将 Nav2 导航简化为状态机

| Nav2 组件 | simple_nav_fsm 替代方案 |
|---|---|
| `planner_server` (Theta* 全局规划) | 直线连接: path = 起点→终点线段 |
| `bt_navigator` (行为树) | 四状态 FSM: ROTATE_TO_HEADING → TRANSLATE → ROTATE_TO_FINAL |
| `controller_server` (OmniPidPursuitController) | PID 直行 + 横向纠偏, 双增益(主动转 vs 被动拉回) |
| 两层 `costmap` (global + local) | **无 costmap** — 不避障, 安全性由发 goal 上层负责 |
| `velocity_smoother` | 内置指令斜坡 (加速度限幅) |
| `behavior_server` (BackUpFreeSpace) | 过冲防护: 转身重瞄补走 (max_reaim 限次) |

### 四状态 FSM

```
IDLE ──(收到 goal)──▶ ROTATE_TO_HEADING ──(朝向就位)──▶ TRANSLATE ──(到点)──▶ ROTATE_TO_FINAL ──(朝向就位)──▶ IDLE
  ▲                        │                              │                      │
  │                        │                              │                      │
  └────(失败/抢占)─────────┴──(过冲/失稳→重瞄)────────────┘──(位置漂出→重瞄)────┘
  
  任意状态 ──(TF丢失/急停)──▶ SAFETY_STOP ──(恢复)──▶ ROTATE_TO_HEADING (重瞄当前goal)
```

### 关键设计决策

- **全闭环判定**: 误差在容差内"连续 N 帧"才算到位, 没有任何开环时长标定
- **双增益**: 主动转向 (`kp_rotate`) 与直行被动纠偏 (`kp_hold`) 分离, 后者明显更小
- **角度差 wrap-around 归一化**: 跨 ±180° 不会炸出假误差
- **TF 新鲜度显式检查**: 拿不到新鲜数据 → `SAFETY_STOP`, 绝不拿旧数据继续算
- **TRANSLATE 固定朝向**: 进入直行那一刻冻结 path_yaw, vy 闭环横向误差 (狗支持侧移)
- **过冲防护**: 沿线投影越过终点 → 转身重瞄补走 (≤ max_reaim 次)
- **失败看门狗**: 状态超时 + 无进展检测 (转向无进展会提示 `body_posture` 没让出 yaw 控制权)
- **接口兼容**: 直接复用 `nav2_msgs/action/NavigateToPose`, 上层代码零迁移

## 在 dog_auto 中的位置

```
dog_auto/nav/src/navigation/simple_nav_fsm/
├── package.xml
├── setup.py / setup.cfg
├── README.md
├── config/
│   ├── params.yaml          # 哨兵/小车平台参数 (v_max=0.4, wz_max=0.8)
│   └── params_dog.yaml      # ★ 狗平台参数 (匹配 cmd_vel_guard: v_max=0.15, wz_max=0.3)
├── launch/
│   └── simple_nav.launch.py
└── simple_nav_fsm/
    ├── __init__.py
    ├── fsm_node.py           # 状态机核心
    ├── record_waypoint.py    # 打点工具
    └── run_route.py          # 跑路线工具
```

## 构建

```bash
# 在 nav workspace 中
cd ~/dog_auto/nav
colcon build --packages-select simple_nav_fsm
source install/setup.bash
```

## 启动

### 前置条件

```bash
# 终端1: 狗底盘
ros2 launch nav_bringup dog_bringup.launch.py

# 终端2: 定位 (point_lio + small_gicp_relocalization + map_server)
ros2 launch nav_bringup navigation.launch.py map_name:=arena
```

然后**停止 Nav2 层** (把 controller_server / planner_server / bt_navigator / behavior_server kill 掉)，或直接用不含 Nav2 的定位 launch (如有)。

### 启动状态机

```bash
# 终端3: 替换 Nav2, 启动状态机导航
ros2 launch simple_nav_fsm simple_nav.launch.py

# 如果狗已载货可切负重参数:
ros2 launch simple_nav_fsm simple_nav.launch.py params_file:=/path/to/params_loaded.yaml
```

## 使用

### 手发单个 goal (调试)

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, \
   orientation: {z: 0.0, w: 1.0}}}}" --feedback
```

### 打点

```bash
# 把狗推到目标位置停稳:
ros2 run simple_nav_fsm record_waypoint pickup_a
ros2 run simple_nav_fsm record_waypoint corner_1 --xy-only   # 中途点不做终点朝向
```

打点工具会:
- 采 1 秒 map→base_footprint TF (约 30 帧)
- 取均值
- 方差超标 (刚挪完/TF 还在跳) → **拒绝写入**并提示原因
- 写入 `~/.simple_nav_fsm_waypoints.yaml`

确认无误可加 `--force`。

### 跑路线

先编辑 `~/.simple_nav_fsm_waypoints.yaml` 里的 `routes` 段:

```yaml
routes:
  demo: [pickup_a, corner_1, place_a, home]
```

```bash
ros2 run simple_nav_fsm run_route demo
# 或直接给名字序列:
ros2 run simple_nav_fsm run_route pickup_a corner_1 place_a home
```

## 在线调参

所有参数支持运行中修改：

```bash
ros2 param set /simple_nav_fsm kv 0.6
ros2 param set /simple_nav_fsm ky 1.0
ros2 topic echo /simple_nav_fsm/state     # 实时看状态/各项误差
```

## 狗 vs 哨兵: 参数差异

| 参数 | 狗 (params_dog.yaml) | 哨兵/小车 (params.yaml) | 原因 |
|------|---------------------|------------------------|------|
| `v_max` | 0.15 | 0.4 | 匹配 cmd_vel_guard vx_max |
| `vy_max` | 0.1 | 0.12 | 匹配 cmd_vel_guard vy_max |
| `wz_max` | 0.3 | 0.8 | 匹配 cmd_vel_guard w_max |
| `kp_rotate` | 1.5 | 1.8 | 狗惯性小 |
| `wz_hold_max` | 0.2 | 0.25 | 保守纠偏 |
| `ky` | 1.0 | 0.8 | 狗横向移动能力弱, 加强纠偏 |
| `acc_x/y` | 0.3 | 0.5 | 狗保护 |
| `acc_wz` | 1.5 | 2.0 | 狗保护 |
| `xy_tol` | 0.05 | 0.04 | 狗步态抖动大 |
| `yaw_tol_deg` | 5.0 | 4.0 | 防止边界横跳 |
| `translate_timeout_per_m` | 25s | 15s | 狗慢, 0.15m/s 时 1m≈7s |

## 验证顺序

0. **地基: yaw 控制权。** 不跑状态机, 直接:
   `ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.15}}"`
   同时 `ros2 run tf2_ros tf2_echo map base_footprint` 看 yaw 是否变化。
   转不动 = body_posture 没让权, 先解决它。

1. **纯转向。** `record_waypoint here` 记下当前位置, 手改 yaw 后对它发 goal。

2. **纯直行。** 发正前方 2m、同朝向的 goal。观察 `e_y` 是否收敛在 ±2cm 内。

3. **安全分支。** 测急停 + 杀重定位测 TF 丢失 + 用手挡狗测"直行无进展"看门狗。

4. **负载重跑。** 抱箱重复 1~3。需要不同增益时用 `params_loaded.yaml`。

5. **串联。** `run_route demo` 跑完整搬箱路径。

## 行为约定

- **goal 一律 map 系**: 其它 frame 直接 abort
- **终点朝向开关**: behavior_tree="xy_only" 不做终点朝向; behavior_tree="with_final_heading" 强制做
- **抢占**: 新 goal 中止当前 goal
- **失败**: 所有 abort 带明确原因 (包括"body_posture 是否让出 yaw 控制权"这类提示)
- **SAFETY_STOP**: 持续发零速; 条件解除需连续 `recover_frames` 帧; 恢复后重瞄当前 goal
- **回退 Nav2**: 接口就是 `navigate_to_pose`, 上层零改动

## 已知取舍

- **不避障**: 安全性由发 goal 的上层负责; 本节点只保证被挡住时**明确报失败**
- **不倒车**: 过冲通过"转身重瞄补走"处理, 全程只有前进 + 侧移 + 原地转
- **Python 实现**: 30Hz 控制循环足够; 若上更高频率逐函数翻译成 rclcpp 即可
