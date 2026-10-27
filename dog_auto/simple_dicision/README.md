# simple_dicision · 简化决策机 —— 使用说明

> 最简版决策机：**手动输入颜色 → gap12 抓取 → 顺序放置**。
> 对标 debug_console.py 的 X5 通信模式：发串口命令 → 等 s3=02(完成) → 再发导航 goal，严格串行。

## 文件结构

```
simple_dicision/
├── package.xml
├── setup.py / setup.cfg
├── resource/simple_dicision
├── config/
│   └── mission.yaml                           ← ★ 路点映射配置（按实际打点名称改）
├── launch/simple_mission.launch.py             ← launch 文件
├── README.md                                   ← 本文件
└── simple_dicision/
    ├── __init__.py
    └── simple_mission_node.py                  ← 全部逻辑（自包含）
```

---

## 1. 流程总览

```
stdin 输入两色 → S0 初始化 X5 → S3 进场 → S4 抓取 → S5 过减速带 → S6 放左 → S6 放右 → 完赛
```

| 阶段 | 动作 | 通信 |
|------|------|------|
| 启动 | `input()` 提示输入左侧/右侧箱子颜色 | stdin |
| S0_INIT | 初始化机械臂 | `AB 01 00 BA` → 等 s3=02 |
| S3_ENTER | `start` → `grab`（抓取点，gap12 缝中站位） | navigate_to_pose |
| S4_GRAB | X5 抓取左右两箱 | `AB 04 01 BA` → 等 s3=02 |
| S5_CROSS | `cross`（过减速带点） | navigate_to_pose |
| S6_PLACE_left | 走到左箱对应放置区 → 放左托盘 | goto `place_<color1>` → `AB 05 01 BA` → 等 s3=02 |
| S6_PLACE_right | 走到右箱对应放置区 → 放右托盘 | goto `place_<color2>` → `AB 05 02 BA` → 等 s3=02 |
| DONE | 统计输出 | — |

**规则**：
- 第一个输入 = 左侧箱子 = 放左托盘 (`05 01`)
- 第二个输入 = 右侧箱子 = 放右托盘 (`05 02`)
- 路点名称全部从 `config/mission.yaml` 读取，不硬编码

---

## 2. 配置文件：路点映射

`config/mission.yaml` 中的 `route` 段将逻辑名映射到 `~/.simple_nav_fsm_waypoints.yaml` 中已打点的**实际路点名**：

```yaml
route:
  start:       "1"        # 起始点（启动区）
  grab:        "22"       # 抓取点（gap12 缝中站位）
  cross:       "2222"     # 过减速带点

  place_green: "1111"     # 食品区(绿) 放置点 — zone 0
  place_gray:  "2222"     # 工具区(灰) 放置点 — zone 1
  place_blue:  "3333"     # 仪器区(蓝) 放置点 — zone 2
  place_red:   "4444"     # 药品区(红) 放置点 — zone 3
```

| 逻辑名 | 含义 | 备注 |
|--------|------|------|
| `start` | 起点 | 进场起始位，狗从启动区出发的第一站 |
| `grab` | 抓取点 | gap12 缝中站位，狗居中停稳后 X5 抓取 |
| `cross` | 过减速带点 | 越过减速带后的安全位，再分别去各放置区 |
| `place_green` | 食品区(绿) zone 0 | 归位区编号 0 |
| `place_gray` | 工具区(灰) zone 1 | 归位区编号 1 |
| `place_blue` | 仪器区(蓝) zone 2 | 归位区编号 2 |
| `place_red` | 药品区(红) zone 3 | 归位区编号 3 |

**使用方法**：
1. 先用 `ros2 run simple_nav_fsm record_waypoint <点名>` 在场地打好 7 个点
2. 将 `config/mission.yaml` 中的值改为你实际打的点名（如 `"1"` / `"22"` / `"2222"` 等）
3. launch 时会自动加载这个配置文件

也可指定自定义配置文件路径：
```bash
ros2 launch simple_dicision simple_mission.launch.py route_config:=/path/to/my_mission.yaml
```

---

## 3. 前置条件

### 3.1 编译

```bash
cd ~/dog_auto
colcon build --packages-select simple_dicision --symlink-install
source install/setup.bash
```

### 3.2 依赖终端（另开）

按顺序启动（决策机最晚）：

```bash
# 终端① 狗底盘
ros2 launch nav_bringup dog_bringup.launch.py

# 终端② 定位
ros2 launch nav_bringup dog_navigation.launch.py map_name:=arena run_nav2:=False

# 终端③ 导航 FSM
ros2 launch simple_nav_fsm simple_nav.launch.py
```

### 3.3 路点要求

在 `~/.simple_nav_fsm_waypoints.yaml` 中至少打好 `config/mission.yaml` 中配置的 7 个路点。

---

## 4. 跑法

### 4.1 无实物彩排（dry_run）

```bash
ros2 run simple_dicision simple_mission_node --ros-args -p dry_run:=True
```

启动后提示输入颜色：
```
请输入左侧箱子颜色 (green/gray/blue/red): red
请输入右侧箱子颜色 (green/gray/blue/red): blue
```

然后观察日志走完全流程（每步约 0.3~0.5s 仿真延迟）。

### 4.2 实机（launch）

```bash
# 手动扳机
ros2 launch simple_dicision simple_mission.launch.py auto_start:=False
ros2 service call /simple_dicision/start std_srvs/srv/Trigger

# 自动开跑
ros2 launch simple_dicision simple_mission.launch.py auto_start:=True
```

### 4.3 急停

```bash
ros2 service call /simple_dicision/abort std_srvs/srv/Trigger
```

### 4.4 旁观当前阶段

```bash
ros2 topic echo /simple_dicision/phase
```

---

## 5. 参数参考

### 串口

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyX5` | ⚠ 插上后 `ls /dev/ttyUSB*` 确认真名 |
| `serial_baud` | `115200` | 波特率，双方约定，不要动 |

### 超时

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `nav_timeout_sec` | `30.0` | 单段导航超时秒数 |
| `grab_timeout_sec` | `60.0` | 抓取超时秒数 |
| `place_timeout_sec` | `60.0` | 单次放置超时秒数 |
| `arm_timeout` | `180.0` | 初始化超时秒数 |

### 路点

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `route_config` | `config/mission.yaml` | ★ 路点映射配置文件（launch 自动定位） |
| `waypoints_file` | `~/.simple_nav_fsm_waypoints.yaml` | 打点数据文件路径 |

### 控制

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `auto_start` | `True` | 是否启动后自动开跑 |
| `start_delay` | `2.0` | auto_start 延迟秒数 |
| `nav_retry` | `1` | 导航失败重试次数 |

### 仿真

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dry_run` | `False` | True=不开串口/导航假装到点 |
| `dry_run_step_sec` | `0.3` | 仿真中机械臂每步耗时 |
| `dry_run_nav_sec` | `0.5` | 仿真中每段导航耗时 |

---

## 6. 颜色输入支持

| 输入 | 归位区编号 | 放置路点 key |
|------|-----------|-------------|
| `green` / `绿` / `0` | 0 | `place_green` |
| `gray` / `grey` / `灰` / `1` | 1 | `place_gray` |
| `blue` / `蓝` / `2` | 2 | `place_blue` |
| `red` / `红` / `3` | 3 | `place_red` |

---

## 7. X5 通信协议

### B→A 状态帧（4 字节）

```
AB s1 s2 BA
```

| 命令 | s1 | s2 | 帧 |
|------|----|----|-----|
| 初始化 | `01` | `00` | `AB 01 00 BA` |
| 抓取 | `04` | `01` | `AB 04 01 BA` |
| 放置(左托盘) | `05` | `01` | `AB 05 01 BA` |
| 放置(右托盘) | `05` | `02` | `AB 05 02 BA` |

### A→B 状态帧（5 字节）

```
AB s1 s2 s3 BA
```

| s3 | 含义 |
|----|------|
| `01` | 运行中 |
| `02` | 完成 ✓ |
| `03` | 失败 ✗ |

### 通信模式（与 debug_console.py 一致）

```
B 发命令帧 → 循环读 A 回帧 → s3=02 继续下一步 / s3=03 报失败 / 超时报超时
```

---

## 8. 故障速查

| 症状 | 大概率原因 | 处置 |
|------|-----------|------|
| TX 有日志、RX 死寂 | 线没交叉 / 没共地 / 设备号错 | 查线序；改 `serial_port` |
| 发命令 X5 无反应 | X5 没上电/没就绪 | 等 X5 就绪后重跑 |
| 报"路点不存在" | 打点名与 config/mission.yaml 不一致 | 核对 YAML 中的路点名和 waypoints 文件 |
| 导航超时 | 定位没起 / FSM 没起 | 查终端②③ |
| 狗一 launch 就窜 | auto_start=true | 调试用 `auto_start:=False` |
| 放置错托盘 | 输入顺序反了 | 第一个=左托盘，第二个=右托盘 |
| 颜色不识 | 输入了不支持的颜色名 | 用 green/gray/blue/red 或 绿/灰/蓝/红 |
| 配置没生效 | config/mission.yaml 路径不对 | launch 自动定位，直接 ros2 run 需手动指定 `route_config` |
