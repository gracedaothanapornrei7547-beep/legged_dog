// leg_calibration.cpp —— 与 Python PER_LEG_FIXED_PARAMS 对照的标定表
//
// 这些数值是实物测量结果，**改之前必须在实物上验证**。
// 与 parallel/config/robot.py 中 PER_LEG_FIXED_PARAMS 完全一致。

#include "parallel/kinematics/leg_calibration.hpp"

#include <stdexcept>

namespace parallel {
namespace robot {

LegCalibration make_default_calibration(int leg_idx) {
    LegCalibration c;
    c.leg_idx = leg_idx;
    c.math_deadzone_th3 = DEADZONE_TH3_RAD;
    c.math_deadzone_th1 = DEADZONE_TH1_RAD;
    c.math_deadzone_th2 = DEADZONE_TH2_RAD;
    c.stand_z_body = DEFAULT_STAND_Z_BODY;
    c.branch = LEG_BRANCH;

    switch (leg_idx) {
        case LEG_FR:  // 前右 (CAN: 见 task::LEG_CAN_CHANNEL)
            c.motor_id_th3 = 1;
            c.motor_id_th2 = 3;
            c.motor_id_th1 = 2;
            c.dir_th3 = +1.0;
            c.dir_th1 = -1.0;
            c.dir_th2 = +1.0;
            c.x_sign  = +1.0;
            c.y_sign  = +1.0;
            c.z_sign  = -1.0;
            c.stand_y_body = -0.02;
            break;

        case LEG_FL:  // 前左 (CAN: 见 task::LEG_CAN_CHANNEL)
            c.motor_id_th3 = 4;
            c.motor_id_th2 = 6;
            c.motor_id_th1 = 5;
            c.dir_th3 = -1.0;
            c.dir_th1 = +1.0;
            c.dir_th2 = -1.0;
            c.x_sign  = +1.0;
            c.y_sign  = -1.0;
            c.z_sign  = -1.0;
            c.stand_y_body = +0.02;
            break;

        case LEG_BL:  // 后左 (CAN: 见 task::LEG_CAN_CHANNEL; 适配器 serial 002600215742570420353230)
            c.motor_id_th3 = 7;
            c.motor_id_th2 = 9;
            c.motor_id_th1 = 8;
            c.dir_th3 = +1.0;
            c.dir_th1 = -1.0;
            c.dir_th2 = +1.0;
            c.x_sign  = -1.0;  // ★ 后腿 hip 绕 z 转 180°
            c.y_sign  = -1.0;
            c.z_sign  = -1.0;
            c.stand_y_body = +0.02;
            break;

        case LEG_BR:  // 后右 (CAN: 见 task::LEG_CAN_CHANNEL; 适配器 serial 003F00245641571220333033)
            c.motor_id_th3 = 11;
            c.motor_id_th2 = 12;
            c.motor_id_th1 = 10;
            c.dir_th3 = -1.0;
            c.dir_th1 = +1.0;
            c.dir_th2 = -1.0;
            c.x_sign  = -1.0;
            c.y_sign  = +1.0;
            c.z_sign  = -1.0;
            c.stand_y_body = -0.02;
            break;

        default:
            throw std::out_of_range("leg_idx out of range [0, 3]");
    }
    return c;
}

// 全局只读表：进程启动时初始化一次。线程安全（C++11 静态局部初始化）。
const LegCalibration& get_calibration(int leg_idx) {
    static const std::array<LegCalibration, N_LEGS> TABLE = {
        make_default_calibration(LEG_FL),
        make_default_calibration(LEG_FR),
        make_default_calibration(LEG_BL),
        make_default_calibration(LEG_BR),
    };
    if (leg_idx < 0 || leg_idx >= N_LEGS) {
        throw std::out_of_range("get_calibration: leg_idx out of range");
    }
    return TABLE[leg_idx];
}

const LegCalibration& get_calibration(std::string_view leg_name) {
    int idx = leg_index(leg_name);
    if (idx < 0) {
        throw std::invalid_argument(
            std::string{"get_calibration: unknown leg name "} + std::string{leg_name});
    }
    return get_calibration(idx);
}

}  // namespace robot
}  // namespace parallel
