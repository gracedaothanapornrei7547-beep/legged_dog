// subs_gait.hpp —— 遥控(SBUS)「通道 ↔ 步态指令」映射 + 单帧解析 + banner
// =============================================================================
//   2026-06-09 从 hardware/sbus_reader.hpp 拆出。分工：
//     - 低层 SBUS 串口接收 / 解 25 字节帧 / FrameSnapshot → 仍在 hardware/sbus_reader.hpp
//       (SbusReader / FrameSnapshot / find_sbus_port / SBUS_* 常量)。
//     - 本文件 = 高层「手柄通道 → 步态指令」: 通道映射表 SbusMap、单帧解析 parse_command、
//       启动 banner。命名空间仍是 motor_test_cpp::sbus(调用处 sbus:: 不变)。
//   依赖 FrameSnapshot → 包含 sbus_reader.hpp。改通道映射只看本文件。
// =============================================================================
#pragma once

#include <string>

#include "motor_test/driver/sbus_reader.hpp"   // FrameSnapshot + namespace sbus + SBUS_* 常量

namespace motor_test_cpp {
namespace sbus {

// ============================================================================
// 遥控通道映射 + 指令解析 + CLI/banner —— ★SBUS 全部封装在 sbus 模块（不散到 cli_io）
// ============================================================================
//
// SbusMap：★想重映射看这一张表，一眼对应「动作 ↔ 通道」。
//   每个动作 = 哪个通道(idx 0~15) + 方向(sign ±1) + 满量程幅度。
//   通道号用 sbus_monitor 拨摇杆/开关看哪个 ch 变；改下面 SbusMap 默认值 → 重编生效（CLI 已删）。
//
// ── 当前通道表 (idx = SBUS 通道 0~15；归一化值 ∈ [-1,1] 中心=0；摇杆死区 deadzone=0.5) ─────
//   ch 0  ch_turn      摇杆·自转    左 / 中 / 右 → yawrate = -1.0 / 0 / +1.0 rad/s（三态, 足端差动真转）
//   ch 2  ch_forward   摇杆·前后    ★比例       → step_x  = ±0.20×(推杆深 0.1~1.0)×(ch8 档)  （过±100死区才动）
//   ch 3  ch_lateral   摇杆·侧移    ★比例       → step_y  = ±0.15×(推杆深 0.1~1.0)×(ch9 档)  （sign=-1 已翻向）
//   ch 5  ch_att_ab    2 段开关      raw > 1700  → 姿态力环 A/B: 顶=B(关 mx/my 力环) / 中=A(基线), 见 readme ①
//   ch 6  ch_posture   2 段开关 SWC  归一化 >0.30 → step_h = 高姿 0.05 / 低姿 0.03  （抬腿高度）
//   ch 7  ch_height    3 段开关      低 / 中 / 高 → stance_z = -0.20 / -0.23 / -0.26（身体站高 20/23/26cm）
//   ch 8  ch_scale_x   旋钮 VRA      连续        → step_x × 0.3(最小)~1.0(最大) 线性
//   ch 9  ch_scale_y   旋钮 VRB      连续        → step_y × 0.3(最小)~1.0(最大) 线性
//   ch 4  ch_jump      3 段          raw<600=1000档 / 中=不跳 / raw>1400=2000档 → 配 ch5 选 3 套跳跃起跳(dog_control)
//                                    ★(ch5,ch4): (1000,1000)沙石地 (1000,2000)下楼梯小跳 (2000,1000)木桥B
//   其余 ch 1 / 10~15：未用。
//
//   ★ch0 自转是【三态·非比例】(|归一化|<0.5 当 0, 推过半即满量程)；★ch2/3 前后/侧移是【比例】
//     (raw 过中心±100 死区后, 随推杆深度线性给步幅 0.1→1.0×max, 死区内当 0)。
//   ★起步使能 = ch0(过三态死区) / ch2 / ch3(过比例死区) 任一动作；只拨姿态开关(ch6) 或 转比例旋钮(ch8/9) 不会起步 → 维持站立。
//   ★失帧 / 超时(>timeout_s=0.3s) / 三摇杆全回中(松手) → step 全 0 → 站立（failsafe，安全）。
//   ★松手收停 = 主控环在 home_s(0.35s) 内把 step 平滑 slew→0（smooth_stop），不是瞬间僵停。
// ────────────────────────────────────────────────────────────────────────────────────────
struct SbusMap {
    // ── 总开关 ──
    bool        enabled   = true;            // ★遥控总开关:开后每拍读手柄→改 step_x/y/h+yawrate；
                                             //   无接收机/失帧 → failsafe 归零 → 站立（安全）
    // ── 串口 / 信号 ──
    // port = "auto" → 启动时扫描所有 /dev/ttyUSB*,自动识别 SBUS 端口和极性。
    //                 拔插换号也能自适应。要锁死可填具体路径 "/dev/ttyUSB0"。
    std::string port      = "auto";
    bool        invert    = false;           // 信号翻转(auto 模式下会被探测覆盖)
    double      timeout_s = 0.3;             // 超此无新帧→failsafe归零站立
    double      deadzone  = 0.5;             // 摇杆死区 |归一化|<dz 视为0

    //   通道 ch0 · 自转摇杆 → yawrate(自转角速度)。最小 −1.0 rad/s / 中位 0 / 最大 +1.0 rad/s
    // （足端差动真转；方向反翻 gait.yaw_turn_scale）
    int ch_turn    = 0;  double sign_turn    = 1.0;  double max_yawrate = 1.0;

    //   ★ch0 自转摇杆仍是三态量：推过死区(|归一化|>deadzone=0.5)即满量程, 非比例。
    //   ★ch2/ch3(前后/侧移)已改【比例】(见下 stick_* 字段): 推杆越深步幅越大。

    //   通道 ch2 · 前后摇杆 → step_x(前后步长)。 满量程 ±0.20 m   （比例: ×推杆深度 0.1~1.0 ×ch8 旋钮）
    int ch_forward = 2;  double sign_forward = +1.0;  double max_step_x  = 0.2;
    //   通道 ch3 · 左右摇杆 → step_y(横移步长)。 满量程 ±0.15 m   （比例: ×推杆深度 0.1~1.0 ×ch9 旋钮；sign=−1 已翻向）
    int ch_lateral = 3;  double sign_lateral = -1.0;  double max_step_y  = 0.15;

    // ── ★2026-06-20 ch2/ch3 摇杆改【比例】(原三态) ──
    //   原行为: 推过死区即满 ±max_step(三态)。新行为: raw 过中心 ±stick_deadzone_raw(防误按)后,
    //   按推杆深度线性给步幅比例 ∈ [stick_scale_min, stick_scale_max] = [0.1, 1.0] → 连续控速,
    //   再 ×max_step_x/y ×ch8/ch9 旋钮档。死区内(|raw-992|≤100)输出 0(站立)。仅 ch2/ch3, ch0 不变。
    int    stick_deadzone_raw = 100;     // 摇杆比例死区(raw counts; SBUS 满跨度 ~819, 100≈0.12 归一化)
    double stick_scale_min    = 0.1;     // 刚过死区时的步幅比例(最小可控步)
    double stick_scale_max    = 1.0;     // 推到底时的步幅比例(满步)

    // ── ★2026-06-21 ch5 重启用 = 姿态力环 A/B 实验开关(原跳楼梯模式 pitch 已置0=空操作, 接管) ──
    //   打到顶(raw>att_ab_raw_min) → B: dog_control 关 body_posture roll/pitch 力矩环(mx/my=硬腿
    //     0.7Hz 自激振源), 姿态交 terrain_adapt 腿长 + 每腿 kp_stance_z 弹簧 + capture_k_roll。
    //   回中(raw≤) → A: 基线力环 ON。同场拨开关 = 同电池/地面 A/B(避 run-to-run 噪声)。见 readme ①。
    //   失帧不翻(main 只 fresh 帧更新); raw 值域 ~172..1811, 1700≈顶死。
    int ch_att_ab     = 5; int att_ab_raw_min = 1700;

    // ── 姿态开关 SWC：2 段开关(无中间档)，选抬腿高度 step_h ──

    //   通道 ch6 · SWC 二段开关 → step_h(swing 抬腿高度)。归一化 >0.30 = 高姿 / ≤0.30 = 低姿
    int    ch_posture  = 6;     double posture_thresh = 0.23;
    //   step_h 取值二选一：高姿 = 最大 0.05 m / 低姿 = 最小 0.03 m
    double step_h_high = 0.03;  double step_h_low     = 0.06;  // 降40%适配 cycle=0.4

    // ── ★2026-06-20 ch7 三段开关 → 身体站高 stance_z(身高) ──
    //   三档身高: 低 0.20m / 中 0.23m / 高 0.26m。stance_z = 足端 z(body 系, 负值; |z|≈身高)。
    //   归一化三分: <-0.5=低 / 中位=中 / >0.5=高 (方向反了就互换 height_low/high)。
    //   ★与步态使能解耦(站着也能调高); 失帧保持上次(身高绝不归 0→塌); slew 限速防阶跃。
    //   ★所有用站高的下游(trot bz / jump stand_z 同步 / stair base_z / 起站姿)全读 a.gait.stance_z
    //     → 改这里只改三档值即可, 全工程自动对齐。
    int    ch_height       = 7;
    double height_low      = -0.20;   // 低档身高 20cm (★浅, 配大 step_x 时留意 th2 余量)
    double height_mid      = -0.23;   // 中档身高 23cm (= gait 默认 / 失帧安全值)
    double height_high     = -0.26;   // 高档身高 26cm
    double height_slew_mps = 0.12;    // 身高切换限速(m/s), 防足端 z 阶跃 lurch (3cm≈0.25s)

    // ── 步长比例旋钮 VRA/VRB：★2026-06-12 改连续比例(原3档)，乘到 step_x / step_y 上 ──

    //   通道 ch8 · VRA 旋钮 → step_x 比例系数。连续：最小 ×0.3 → 最大 ×1.0 (线性, 中位≈×0.65)
    int ch_scale_x = 8;
    //   通道 ch9 · VRB 旋钮 → step_y 比例系数。连续：最小 ×0.3 → 最大 ×1.0 (同上)
    int ch_scale_y = 9;

    // ── 松杆平滑收停(★用户要的"松手让四脚稳稳回中立"，而非瞬间归零僵住) ──
    //   按住时即时响应；松杆那刻把 step_x/y/h+yaw 在 home_s 秒内 slew→0 → 步态自然收尾、
    //   足端落回中立站姿。slew 到 0 后 step 全 0 → is_stand_mode 站立。
    bool   smooth_stop = true;    // false=旧行为(松手瞬间归零)
    double home_s      = 0.35;    // 松杆收停 / 足端回正时长(s)
};

// 一帧手柄 → 控制指令（套用 SbusMap；失帧/松手 → 全 0 → 站立）。
struct SbusCmd {
    bool   fresh = false, gait_enabled = false;
    double step_x = 0.0, step_y = 0.0, yawrate = 0.0, step_h = 0.0;
    double stance_z = -0.23; // ★ch7 三段身高(仅 fresh 帧有效; 调用方失帧保持上次, 不归 0)
};
SbusCmd parse_command(const FrameSnapshot& f, double now_s,
                      const SbusMap& map, double default_step_h);

// 启动 banner 文本（多行，含完整映射；map.enabled 决定 on/off），cli_io::print_banner 调用。
std::string banner(const SbusMap& map);

}  // namespace sbus
}  // namespace motor_test_cpp
