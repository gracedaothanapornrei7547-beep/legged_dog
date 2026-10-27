// subs_gait.hpp —— 遥控(SBUS)「通道 → 步态指令」映射 + 单帧解析 + banner
// =============================================================================
// 所属模块: parallel::sbus (planning 流水线)
// 职责: 高层「手柄通道 → 步态指令」: 通道映射表 SbusMap、单帧解析 parse_command、
//       启动 banner。低层 SBUS 串口接收在 hardware/sbus_reader.hpp。
//       2026-06-09 从 hardware/sbus_reader.hpp 拆出。改通道映射只看本文件。
// =============================================================================
#pragma once

#include <string>

#include "parallel/driver/sbus_reader.hpp"   // FrameSnapshot + namespace sbus + SBUS_* 常量

namespace parallel {
namespace sbus {

// ============================================================================
// 遥控通道映射 + 指令解析 + 启动横幅封装
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
//   ch 5  ch_att_ab    2 段开关      配 ch4 选跳跃预设 → (ch5,ch4)定跳哪套; 走路时无作用
//   ch 6  ch_posture   2 段开关 SWC  归一化 >0.30 → step_h = 高姿 0.05 / 低姿 0.03  （抬腿高度）
//   ch 7  ch_height    3 段开关      低(1000)=20cm / 中(1500)=23cm / 高(2000)=★电机复位(不改身高)
//   ch 8  ch_scale_x   旋钮 VRA      连续        → step_x × 0.3(最小)~1.0(最大) 线性
//   ch 9  ch_scale_y   旋钮 VRB      连续        → step_y × 0.3(最小)~1.0(最大) 线性
//   ch 4  ch_jump      3 段          raw<600=1000档 / 中=不跳 / raw>1400=2000档 → 配 ch5 选 3 套跳跃起跳(dog_control)
//                                    ★(ch5,ch4): (1000,1000)上跳楼梯 (1000,2000)下楼梯小跳 (2000,1000)沙石地 (2000,2000)木桥B
//   其余 ch 1 / 10~15：未用。
//
//   ★ch0/2/3 全部为比例控制: raw 过中心±100 死区后随推杆深度线性给步幅 0.1→1.0×max，死区内当 0。
//   ★起步使能 = ch0/ch2/ch3 任一过比例死区；只拨姿态开关(ch6)或转旋钮(ch8/9)不触发起步。
//   ★失帧/超时(>timeout_s=0.3s)/三摇杆全回中(松手) → step 全 0 → 站立（failsafe）。
//   ★松手收停 = 主控环在 home_s(0.35s) 内把 step 平滑 slew→0（smooth_stop）。
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

    //   通道 ch0 · 自转摇杆 → yawrate(自转角速度)。比例: 推杆越深转越快，最大 ±max_yawrate rad/s
    // （足端差动真转；方向反翻 gait.yaw_turn_scale）
    int ch_turn    = 0;  double sign_turn    = +1.0;  double max_yawrate = 2.0;

    //   ★ch0/ch2/ch3 均为比例: 推杆越深步幅/转速越大(ch0), 详见 stick_* 字段。

    //   通道 ch2 · 前后摇杆 → step_x(前后步长)。 满量程 ±0.20 m   （比例: ×推杆深度 0.1~1.0 ×ch8 旋钮）
    int ch_forward = 2;  double sign_forward = +1.0;  double max_step_x  = 0.25;
    //   通道 ch3 · 左右摇杆 → step_y(横移步长)。 满量程 ±0.15 m   （比例: ×推杆深度 0.1~1.0 ×ch9 旋钮；sign=−1 已翻向）
    int ch_lateral = 3;  double sign_lateral = -1.0;  double max_step_y  = 0.15;

    // ── ch0/ch2/ch3 摇杆比例控制 ──
    //   raw 过中心 ±stick_deadzone_raw(防误按)后按推杆深度线性给比例 ∈ [stick_scale_min, stick_scale_max]，
    //   再 ×max_step_x/y ×ch8/ch9 旋钮档。死区内(|raw-992|≤100)输出 0(站立)。
    int    stick_deadzone_raw = 100;     // 摇杆比例死区(raw counts; SBUS 满跨度 ~819, 100≈0.12 归一化)
    double stick_scale_min    = 0.1;     // 刚过死区时的步幅比例
    double stick_scale_max    = 1.0;     // 推到底时的步幅比例

    // ── ch5 = 跳跃选组(配 ch4) ──
    int ch_att_ab     = 5; int att_ab_raw_min = 1700;

    // ── ch6 pitch 闭环开关: 推到顶(>1750)→kp_pitch=0 kd_pitch=0(上楼梯/崎岖时 pitch 环不插手) ──
    int    ch_pitch     = 6;     // ch6: >1750→kp_pitch=0 kd_pitch=0(走路/跳跃均生效)
    double step_h_fixed = 0.06;  // 抬腿高度固定 6cm

    // ── ch7 三段开关: 低(1000)=20cm / 中(1500)=23cm / 高(2000)=电机复位 ──
    //   stance_z = 足端 z(body 系, 负值; |z|≈身高)。
    //   归一化三分: <-0.5=低 / 中位=中 / >0.5=高(2000档→dog_control跳过身高更新,专做复位)。
    //   与步态使能解耦(站着也能调高); 失帧保持上次(身高不归 0→塌); slew 限速防阶跃。
    //   所有用站高的下游(trot bz / jump stand_z 同步 / 起站姿)全读 a.gait.stance_z → 自动对齐。
    int    ch_height       = 7;
    double height_low      = -0.20;   // 低档(1000) 20cm
    double height_mid      = -0.23;   // 中档(1500) 23cm (= gait 默认 / 失帧安全值)
    double height_high     = -0.26;   // 2000档=电机复位, 此值仅兼容 parse_command 不生效
    double height_slew_mps = 0.12;    // 身高切换限速(m/s), 防足端 z 阶跃

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

// 启动横幅文本（包含通道完整映射关系；map.enabled 决定开启与否），供 main 函数启动时调用。
std::string banner(const SbusMap& map);

}  // namespace sbus
}  // namespace parallel
