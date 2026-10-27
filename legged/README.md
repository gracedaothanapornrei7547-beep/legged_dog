# legged — 障碍赛 500Hz 纯 VMC 运控系统

## 快速编译与运行

### 1. CAN 总线初始化 (1Mbps)
```bash
# 启动 4 路独立 CAN 通道 (can3, can5, can6, can7 绑定 灵足 RS02)
sudo scripts/setup-can.sh
```

### 2. 编译工程
```bash
colcon build --symlink-install --packages-select parallel_vmc
source install/setup.bash
```

### 3. 运行 500Hz 纯 VMC 运控主节点
```bash
# PREEMPT_RT 实时调度运行
ros2 run parallel_vmc trot_quad
```
