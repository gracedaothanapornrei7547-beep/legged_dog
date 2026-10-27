// run_config.hpp —— 整机一次运行的全部配置（纯代码默认；改默认 → 重编 → 上电）
// =============================================================================
// 所属模块: parallel::interface (task 层)
// 职责: 聚合各子系统 *Config（gait/wbc/hardware...）+ 运行级参数（主循环时序/日志/CAN），
//       main 函数构造该配置并传递给控制环，负责集中聚合与初始化各子模块参数。
//       调参 = 改对应模块 *Config 的默认值 → colcon build → 上电。
// =============================================================================

#pragma once

#include <string>

#include "parallel/planning/gait.hpp"
#include "parallel/planning/raibert.hpp"
#include "parallel/kinematics/baseline.hpp"
#include "parallel/dynamics/body_posture.hpp"
#include "parallel/dynamics/force_alloc.hpp"
#include "parallel/dynamics/force_control.hpp"
#include "parallel/dynamics/wbc.hpp"
#include "parallel/planning/subs_gait.hpp"        // SbusMap（高层通道映射；低层接收在 hardware/sbus_reader.hpp）

namespace parallel {
namespace interface {

// 整机运行配置：子系统 *Config 的聚合 + 运行级参数。
struct RunConfig {
    // ── 各子系统配置（默认值在各自模块 hpp 里改）──
    gait::GaitConfig                gait{};
    baseline::BaselineConfig        baseline{};
    force_control::ForceCtrlConfig  force_ctrl{};
    body_posture::BodyPostureConfig body_posture{};   // .enabled = 闭环 wrench 总开关
    force_alloc::ForceAllocConfig   force_alloc{};     // .swing_support_max 在此
    raibert::RaibertConfig          raibert{};
    sbus::SbusMap                   sbus{};            // .enabled = 遥控总开关 + 通道映射
    wbc::WbcConfig                  wbc{};              // 分层 WBC: grasp pinv + 摩擦锥投影
    bool        use_wbc = true;                        // true=WBC 替代 heuristic force_alloc

    // ── 用户实时运动指令（body 系；SBUS 每拍写，松手 → 0）──
    double vcmd_x_body = 0.0;
    double vcmd_y_body = 0.0;
    double yawrate_cmd = 0.0;

    // ── 主循环时序（不属于任何控制子系统）──
    double rate_hz    = 500.0;     // 控制 + CAN 发送频率（500Hz 给 CAN 总线/电机喘息）
    double startup_s  = 3.0;       // 缓启动 ramp 时长
    // 单腿测试: -1=全部4腿正常; 0=FL/1=FR/2=BL/3=BR(只驱动该腿，另3腿 CAN 未连=不动)。
    // 悬空单腿调试用: connect失败放行 + preflight跳过 + 跳arm/读位姿。
    int    only_leg   = -1;        // 4 腿正常
    // 缓启动后持续 trot，直到 Ctrl+C/急停。无"走 N 秒转站立"逻辑。
    // 松手/失帧/step 全 0 → is_stand_mode 原地站立（遥控控制）。

    // ── 日志（逐帧二进制遥测 + 文本重定向；空 = main 里默认成 log/trot_<时间戳>.{bin,log}，不覆盖）──
    std::string telem_log;                 // 逐帧 .bin（后台线程落盘，控制环零格式化；tools/parse_telem.py 解析）
    std::string log_file;                  // 文本 stdout（横幅/0.5s 状态/收尾/✓done）重定向 .log（可 tail -f）
    bool        print_to_terminal = false; // true = 文本打回终端（本地/sim 调试用）

    // ── CAN 启动自动 reset（rmmod/modprobe + 清 TEC，从 TEC=0 干净状态开跑，避免残留 PASSIVE）──
    bool        can_reset_on_start = true;
};

}  // namespace interface
}  // namespace parallel
