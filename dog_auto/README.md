# dog_auto — 任务赛全自主导航与决策系统

## 快速编译与运行

### 1. 环境初始化
```bash
source setup-env.sh
```

### 2. 编译各工作区
```bash
# 导航工作区
cd nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 实时底盘桥与运控工作区
cd ../legged_nav
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 3. 三终端启动发车
```bash
# 终端 1：底盘实时运控与跨域桥 (cmd_bridge + nav_quad)
ros2 launch nav_bringup dog_bringup.launch.py

# 终端 2：全自主导航核心 (Point-LIO + GICP 重定位 + Nav2)
ros2 launch nav_bringup dog_navigation.launch.py map_name:=arena

# 终端 3：任务决策教练机 (FSM 发车)
ros2 launch mission_director mission.launch.py
```
