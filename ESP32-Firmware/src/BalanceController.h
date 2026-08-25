#ifndef ESP32_BALANCE_CAR_BALANCE_CONTROLLER_H
#define ESP32_BALANCE_CAR_BALANCE_CONTROLLER_H

#include "BatteryMonitor.h"
#include "BleConfigService.h"
#include "ControlTypes.h"
#include "EncoderPair.h"
#include "MotorDriver.h"
#include "Mpu6050.h"
#include "PidController.h"
#include "StatusLed.h"

/**
 * 平衡车控制器总协调类。
 *
 * 控制链：目标速度 -> 目标倾角 -> 电机 PWM。
 * 该类拥有全部硬件模块，并保证校准、正常控制和安全停机不会并发写电机。
 */
class BalanceController {
public:
    BalanceController();

    void begin();

private:
    static void controlTaskEntry(void *context);

    static void statusLedTaskEntry(void *context);

    void runControlLoop();

    void runStatusLedLoop() const;

    void updateOuterLoops(uint32_t nowMs);

    void resetMotionState();

    void loadGyroOffsets();

    void saveGyroOffsets();

    void runGyroCalibration();

    ControlParameters parameters_;
    MotionCommand motionCommand_;
    PidController anglePid_;
    PidController speedPid_;
    PidController turnPid_;

    Mpu6050 mpu6050_;
    bool mpu6050Ready_ = false;
    EncoderPair encoders_;
    MotorDriver motors_;
    StatusLed statusLed_;
    BleConfigService ble_;

    // 速度相关量单位均为“编码器脉冲/速度采样周期”。
    float targetSpeed_ = 0.0f;
    float speed_ = 0.0f;
    float leftSpeed_ = 0.0f;
    float rightSpeed_ = 0.0f;
    float speedReference_ = 0.0f;
    float targetAngle_ = 0.0f;
    float targetYawRate_ = 0.0f;
    float turnOutput_ = 0.0f;

    uint32_t lastOuterLoopUpdateMs_ = 0;
    uint32_t lastControlUpdateUs_ = 0;
};

#endif
