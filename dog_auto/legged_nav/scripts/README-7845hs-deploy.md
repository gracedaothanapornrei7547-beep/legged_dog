# 7845HS 任务赛目标机 —— 部署清单（scripts/ 本地速查）

> 把狗底盘从 N150 mini 搬到 7845HS 时照这个走。深度背景见 `~/dog_auto/readme.md` 的 §1.6 / §6.4-6.6。
> 本目录脚本分两类：**现有(N150 起就有)** 和 **新增(本次 7845HS 适配)**。

## 现有脚本（保持不变，逻辑通用）
- `setup-can.sh` —— 幂等把 can0/1/6/7 拉 UP（bitrate 1M，已 UP 绝不 cycle，防赤乌适配器 >6 次 down/up 死锁）。
- `can-watchdog.sh` + `can-watchdog.service` —— 后台保 CAN 一直 UP。
- `can-reset.sh` —— `nav_quad` 启动时 `sudo` 调用，rmmod/modprobe gs_usb 彻底复位 + 调 `lock-can-by-usb.sh` 锁名。
- `lock-can-by-usb.sh` + `70-candlelight-can.rules` —— 按 USB 物理口把接口锁成固定 can 名。**⚠ 内含 N150 的 USB 口，搬机必改（见下）。**
- `install-can-up.sh` —— 装 watchdog service + sudoers 免密。
- `can-reset-nopasswd` / `rc-nopasswd` —— sudoers 片段。**⚠ 内含旧机 `rc` 用户 + `/home/rc/...` 路径，搬机必改。**

## 新增脚本（7845HS 适配）
- `setup-rt-7845hs.sh` —— 实时调优：隔离 nav_quad 占的 CPU1/2/3(+SMT 兄弟)、装 rtprio/memlock limits、performance governor、关 irqbalance、打印 GRUB cmdline。
- `detect-can-usb.sh` —— 只读探测本机 can↔USB 物理口映射，对比 N150、生成可粘贴的 lock 表/udev 骨架，解决“USB 口路径换机会变”。

## 部署顺序（在 7845HS 上）

1. **装依赖**（见 readme §6.4）：ROS2 Humble + rosdep + Nav2/PCL/Eigen/OpenMP/TBB、Livox-SDK2、`can-utils`。
2. **CAN 物理口适配（最关键）**：
   ```bash
   ./detect-can-usb.sh          # 看当前 can↔USB 口，对比 N150
   ```
   若提示 USB 口变了 → 用 `function_can_test` 逐条 bus 靠电机 id 认腿（FR=1/2/3 FL=4/5/6 BL=7/8/9 BR=10/11/12），
   据此改 `lock-can-by-usb.sh` 的 `USB_TO_CAN`、`70-candlelight-can.rules`、以及 **`hardware_manager.hpp` 的 `LEG_CAN_CHANNEL`（权威）**，然后 `colcon build`。
3. **改 sudoers 路径/用户**：把 `can-reset-nopasswd` / `rc-nopasswd` 里的 `rc` 和 `/home/rc/Desktop/legged_control_lite/...` 换成 7845HS 实际用户与本仓库路径（`whoami` / `readlink -f scripts/can-reset.sh`）。
4. **装 CAN 服务**：
   ```bash
   sudo ./install-can-up.sh     # watchdog + 免密（先确认第 3 步路径已改）
   ```
5. **实时调优**：
   ```bash
   sudo ./setup-rt-7845hs.sh            # 运行时项 + 打印 GRUB 建议
   # 按提示把 isolcpus=... 加进 /etc/default/grub → sudo update-grub → reboot
   ```
6. **网络**：接雷达网口配静态 IP `192.168.1.50`，`ping 192.168.1.146` 通（`MID360_config.json`）。
7. **验证**：
   - `./detect-can-usb.sh` 4 条腿口对；`function_can_test <leg> private step` 逐腿点动对得上。
   - `cat /sys/devices/system/cpu/isolated` 显示隔离核；`ps -eLo psr,rtprio,comm | grep nav_quad` 亲和/优先级对。
   - 先 `cmd_bridge` 再 `nav_quad`；`shm_feed 0.1 0 0` 让狗低速前进；最后接 nav 全栈。

## 搬机必改清单（一句话）
- USB→CAN 映射（`lock-can-by-usb.sh` / `70-candlelight-can.rules` / `LEG_CAN_CHANNEL`）
- sudoers 用户与路径（`*-nopasswd`）
- RT 隔离核 GRUB（`setup-rt-7845hs.sh` 打印）
- 雷达网口静态 IP
