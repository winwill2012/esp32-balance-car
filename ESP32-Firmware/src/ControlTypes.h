#ifndef ESP32_BALANCE_CAR_CONTROL_TYPES_H
#define ESP32_BALANCE_CAR_CONTROL_TYPES_H

struct ControlParameters {
    // 直立环 PD：角度误差（度）和 X 轴角速度共同输出基础 PWM。
    // BLE 协议中分别对应 KP、KD。
    float angleKp = 90.0f;
    float angleKd = 2.0f;

    // 速度环 P：速度误差 -> 目标倾角（度），BLE 协议中对应 KV。
    // 当前速度单位是“每个速度采样周期内的编码器脉冲数”，不是 m/s。
    float speedKp = 0.58f;

    // 小车机械平衡零点（度）。控制角度 = IMU 原始角度 - angleOffset。
    float angleOffset = 0.0f;

    // 遥控目标速度的最大变化率，单位为“速度单位/秒”，用于抑制急加减速。
    float speedSlew = 35.0f;

    // 电机从静止到可靠起转所需的最小 PWM，左右轮独立标定。
    int leftMotorDeadZone = 164;
    int rightMotorDeadZone = 164;
};

struct MotionCommand {
    // 平移目标速度：正负方向取决于编码器和电机安装方向。
    float targetSpeed = 0.0f;

    // 左右轮差动 PWM；正值使左轮增加、右轮减小。
    float turnPwm = 0.0f;

    // 断开 BLE、进入校准等状态时立即清除运动意图。
    void clear() {
        targetSpeed = 0.0f;
        turnPwm = 0.0f;
    }
};

#endif // ESP32_BALANCE_CAR_CONTROL_TYPES_H
