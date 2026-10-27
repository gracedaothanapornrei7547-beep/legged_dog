/**
 * @file command_source.cpp
 * @brief 自主导航底盘运动指令输入接口与共享内存 (SHM) 消费器实现
 */

#include "motor_test/task/command_source.hpp"

#include <atomic>
#include <cstdio>
#include <ctime>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace motor_test_cpp {
namespace task {

namespace {

/**
 * @brief 获取 Linux 单调时钟高精度时间戳 (秒)
 */
inline double mono_now_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

}  // namespace

ShmCommandSource::ShmCommandSource(double step_per_v, double fixed_step_h,
                                   double fixed_stance_z, double timeout_s)
    : step_per_v_(step_per_v), fixed_step_h_(fixed_step_h),
      fixed_stance_z_(fixed_stance_z), timeout_s_(timeout_s) {
    // 以只读模式连接共享内存区域，保证与 ROS2 上层通信节点的进程级安全隔离
    fd_ = shm_open(NAV_CMD_SHM_NAME, O_RDONLY, 0666);
    if (fd_ >= 0) {
        void* p = mmap(nullptr, sizeof(NavCmdShm), PROT_READ, MAP_SHARED, fd_, 0);
        if (p != MAP_FAILED) {
            shm_ = static_cast<NavCmdShm*>(p);
        } else {
            ::close(fd_);
            fd_ = -1;
        }
    }
}

ShmCommandSource::~ShmCommandSource() {
    if (shm_) {
        ::munmap(shm_, sizeof(NavCmdShm));
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

ChassisCommand ShmCommandSource::read() {
    ChassisCommand c;
    c.step_h   = fixed_step_h_;
    c.stance_z = fixed_stance_z_;
    if (!shm_) {
        return c;
    }

    // Seqlock 无锁一致性读取循环: 确保读取前后序列号一致且为偶数 (写端未处于更新状态)
    NavCmdShm snap{};
    bool ok = false;
    for (int tries = 0; tries < 8 && !ok; ++tries) {
        const uint32_t s1 = shm_->seq;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s1 & 1u) continue;
        snap.write_t_s    = shm_->write_t_s;
        snap.vx           = shm_->vx;
        snap.vy           = shm_->vy;
        snap.omega        = shm_->omega;
        snap.gait_enabled = shm_->gait_enabled;
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t s2 = shm_->seq;
        if (s1 == s2) {
            ok = true;
        }
    }
    if (!ok) {
        return c;
    }

    // 软件看门狗校验: 若写入时间戳与当前系统时间差超过阈值，判定指令失效并进入安全保护
    if (mono_now_s() - snap.write_t_s > timeout_s_) {
        return c;
    }

    // 速度指令线性映射为足端运动位移与转向角速度
    c.fresh        = true;
    c.gait_enabled = (snap.gait_enabled != 0);
    c.step_x       = snap.vx * step_per_v_;
    c.step_y       = snap.vy * step_per_v_;
    c.yawrate      = snap.omega;
    return c;
}

}  // namespace task
}  // namespace motor_test_cpp
