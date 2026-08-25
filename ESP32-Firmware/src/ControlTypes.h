#ifndef ESP32_BALANCE_CAR_CONTROL_TYPES_H
#define ESP32_BALANCE_CAR_CONTROL_TYPES_H

struct ControlParameters {
    // 直立环 PID：D 项使用 X 轴陀螺仪角速度，三个增益均可由 BLE 调整。
    float angleKp = 90.0f;
    float angleKi = 0.0f;
    float angleKd = 4.0f;

    // 速度环 PID：速度误差 -> 目标倾角（度）。
    // 当前速度单位是“每个速度采样周期内的编码器脉冲数”，不是 m/s。
    float speedKp = 0.77f;
    float speedKi = 0.0f;
    float speedKd = 0.0f;

    // 转向环 PID：目标/实测偏航角速度 -> 左右轮差动 PWM。
    float turnKp = 2.3f;
    float turnKi = 0.2f;
    float turnKd = 0.0f;

    // 小车机械平衡零点（度）。控制角度 = IMU 原始角度 - angleOffset。
    float angleOffset = 0.0f;

    // 遥控目标速度的最大变化率，单位为“速度单位/秒”，用于抑制急加减速。
    float speedSlew = 35.0f;

};

struct MotionCommand {
    // 平移目标速度：正负方向取决于编码器和电机安装方向。
    float targetSpeed = 0.0f;

    // 蓝牙目标偏航角速度（°/s）；正值表示右转、负值表示左转。
    float targetYawRate = 0.0f;

    // 最近一次 TRN 指令的接收时间。超时后转向目标自动归零。
    uint32_t lastTurnCommandMs = 0;

    // 断开 BLE、进入校准等状态时立即清除运动意图。
    void clear() {
        targetSpeed = 0.0f;
        targetYawRate = 0.0f;
        lastTurnCommandMs = 0;
    }
};

#endif // ESP32_BALANCE_CAR_CONTROL_TYPES_H
