// nav_cmd_shm.hpp —— nav→狗 共享内存命令布局(单一真源)
// =============================================================================
//   狗端(ShmCommandSource)读、ROS2 bridge 节点写, 两边 #include 同一个布局, 绝不漂。
//   只含 POD 结构 + 名字常量, 零依赖(bridge 不必沾狗控制代码)。
//
//   seqlock: 写端写前 seq++(变奇=写中), 写完 seq++(变偶=稳定)。
//            读端读到 seq 偶且读前后一致 = 拿到一致快照, 永不阻塞。
//   看门狗:  读端比 write_t_s 晚 > timeout(默认0.3s) → 判过期 → 归零站立。
// =============================================================================

#pragma once

#include <cstdint>

namespace motor_test_cpp {
namespace task {

// POSIX 共享内存名(= /dev/shm/legged_nav_cmd)。
inline constexpr const char* NAV_CMD_SHM_NAME = "/legged_nav_cmd";

struct NavCmdShm {
    volatile uint32_t seq       = 0;    // seqlock 序号(奇=写中, 偶=稳定)
    uint32_t          _pad      = 0;
    double            write_t_s = 0.0;  // nav 写入时刻(CLOCK_MONOTONIC 秒)
    double            vx        = 0.0;  // (m/s) 前后,  nav 侧已限幅
    double            vy        = 0.0;  // (m/s) 侧向,  nav 侧已限幅
    double            omega     = 0.0;  // (rad/s) 自转, nav 侧已限幅
    uint32_t          gait_enabled = 0; // 1=走 0=站
    uint32_t          _pad2     = 0;
};

}  // namespace task
}  // namespace motor_test_cpp
