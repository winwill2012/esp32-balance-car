#include "BalanceController.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <cmath>

#include "Pins.h"

// 能够容忍的小车最大倾斜角度，超过这个角度，认为小车不可恢复，直接关闭电机
constexpr float kMaximumLeanAngle = 60.0f;
// 速度环最多只能要求内环倾斜 30 度，防止大速度误差产生危险目标角。
constexpr float kMaximumTargetAngle = 30.0f;
// 速度环计算周期，单位ms
constexpr uint32_t kSpeedUpdatePeriodMs = 20;
// BLE 与串口遥测不参与控制，降低到 10 Hz 以减少通知和打印对控制周期的干扰。
constexpr uint32_t kTelemetryPeriodMs = 100;
// 小程序以 150 ms 心跳发送遥控命令；超过此时间没有收到 TRN 时自动回中。
constexpr uint32_t kTurnCommandTimeoutMs = 450;
// 小程序直接下发目标偏航角速度，单位为 °/s，固件不再进行比例换算。
constexpr float kMaximumTargetYawRate = 300.0f;
// PID 修正量和“前馈 + 修正”总输出分别限幅；总输出略高于原始 300 PWM，
// 用于加速起转，同时避免瞬间叠加到电机的 10 位满量程。
constexpr float kMaximumTurnOutput = 300.0f;
constexpr float kMaximumMixedTurnOutput = 450.0f;
// 三个积分限幅的单位分别为“度·秒”“速度单位·秒”和“度”。
constexpr float kAngleIntegralLimit = 10.0f;
constexpr float kSpeedIntegralLimit = 20.0f;
constexpr float kTurnIntegralLimit = 90.0f;
// 当前安装方向下 MPU6050 的 gyroZ 为负表示向右转，将其归一化为“右转为正”。
constexpr float kYawRateSign = -1.0f;
// 方向归一化后的约定：向同一平移方向转动时 LSP/RSP 必须同号。
// 若手推小车前进时某一侧遥测符号相反，只把对应常量改为 -1。
constexpr int kLeftEncoderSign = 1;
constexpr int kRightEncoderSign = 1;

// 尚未执行陀螺仪校准时使用的兜底值；成功校准后由 NVS 中的数据覆盖。
constexpr float kDefaultGyroXOffset = -0.47f;
constexpr float kDefaultGyroYOffset = 0.55f;
constexpr float kDefaultGyroZOffset = -1.42f;

BalanceController::BalanceController()
    : anglePid_(90.0f, 0.0f, 4.0f, kAngleIntegralLimit, kMaximumPwm),
      speedPid_(-0.77f, 0.0f, 0.0f, kSpeedIntegralLimit, kMaximumTargetAngle),
      turnPid_(2.3f, 0.2f, 0.0f, kTurnIntegralLimit, kMaximumTurnOutput),
      mpu6050_(Wire) {
}

void BalanceController::begin() {
    // 先初始化执行器并确保 PWM 为 0，再启动传感器和控制任务，防止上电误动作。
    motors_.begin();
    encoders_.begin();
    BatteryMonitor::begin();
    StatusLed::begin();

    Wire.begin(kMpuSda, kMpuScl, 400000);
    mpu6050Ready_ = mpu6050_.begin();
    if (!mpu6050Ready_) {
        Serial.println("[IMU] MPU6050 not found; motors will remain stopped");
    }
    loadGyroOffsets();
    ble_.begin(parameters_, motionCommand_);

    lastOuterLoopUpdateMs_ = millis();
    lastControlUpdateUs_ = micros();
    // LED 和控制使用独立 FreeRTOS 任务；任务入口接收 this，避免使用全局控制状态。
    xTaskCreate(statusLedTaskEntry, "StatusLed", 2048, this, 1, nullptr);
    xTaskCreate(controlTaskEntry, "BalanceControl", 4096, this, 1, nullptr);
}

void BalanceController::controlTaskEntry(void *context) {
    static_cast<BalanceController *>(context)->runControlLoop();
}

void BalanceController::statusLedTaskEntry(void *context) {
    static_cast<BalanceController *>(context)->runStatusLedLoop();
}

void BalanceController::runControlLoop() {
    // 上一次向BLE汇报状态的时间
    uint32_t lastTelemetryMs = 0;
    // 上一次向BLE汇报小车摔倒状态的时间
    uint32_t lastFallenTelemetryMs = 0;
    uint32_t lastImuRetryMs = 0;

    while (true) {
        if (!mpu6050Ready_) {
            motors_.stop();
            const uint32_t now = millis();
            if (now - lastImuRetryMs >= 1000) {
                lastImuRetryMs = now;
                mpu6050Ready_ = mpu6050_.begin();
                Serial.printf("MPU6050初始化%s\n", mpu6050Ready_ ? "成功" : "失败");
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // BLE 回调只设置请求标志。耗时校准必须在控制任务中串行执行，
        // 这样校准期间不会有另一个控制流程同时向电机写 PWM。
        if (ble_.consumeGyroCalibrationRequest()) {
            runGyroCalibration();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (!mpu6050_.update()) {
            // I2C 读取异常时禁止使用旧姿态继续驱动；下一循环会重新初始化传感器。
            mpu6050Ready_ = false;
            motors_.stop();
            continue;
        }
        const float rawAngle = mpu6050_.angleX(); // 获取俯仰角 Pitch
        // angleOffset 表示机械结构真正直立时 IMU 的读数，而不是陀螺仪零偏。
        const float angle = rawAngle - parameters_.angleOffset;
        const uint32_t controlNowUs = micros();
        const float controlDt = constrain(
            (controlNowUs - lastControlUpdateUs_) * 0.000001f,
            0.001f,
            0.05f
        );
        lastControlUpdateUs_ = controlNowUs;

        // 倾倒后立即关闭电机，并清除速度/位置历史，避免扶起瞬间使用旧误差猛冲。
        if (fabsf(rawAngle) > kMaximumLeanAngle) {
            targetSpeed_ = 0.0f;
            targetAngle_ = 0.0f;
            resetMotionState();
            motors_.stop();

            const uint32_t now = millis();
            if (now - lastFallenTelemetryMs >= kTelemetryPeriodMs) {
                lastFallenTelemetryMs = now;
                ble_.notifyStatus(rawAngle, BatteryMonitor::readPercent(), 0, 0.0f, 0.0f);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t now = millis();
        // 编码器速度噪声较大，外环固定约 20 ms 更新一次；直立环则尽可能快地运行。
        if (now - lastOuterLoopUpdateMs_ >= kSpeedUpdatePeriodMs) {
            updateOuterLoops(now);
        }

        // 最内层直立 PD：
        //   P 项纠正目标角与实际角的偏差；
        //   D 项直接使用陀螺仪角速度抑制快速摆动，避免再对角度做差分放大噪声。
        anglePid_.setKp(parameters_.angleKp);
        anglePid_.setKi(parameters_.angleKi);
        anglePid_.setKd(parameters_.angleKd);
        anglePid_.setTarget(targetAngle_);
        anglePid_.setActual(angle);
        // 误差 = 目标角 - 实际角，因此误差变化率约为 -gyroX。
        // 这保持原控制律 Kp * angleError - Kd * gyroX 不变。
        anglePid_.updateWithDerivative(-mpu6050_.gyroX(), controlDt);
        const int balancePwm = static_cast<int>(anglePid_.output());

        // 转向采用“直接差动 PWM 前馈 + 偏航角速度 PID 修正”：
        // 前馈保留原来快速原地转向的力度，PID 只补偿实际转速与目标转速的误差。
        // 没有新鲜蓝牙命令时前馈和目标均为 0，转向环继续抑制原地自转。
        const bool turnCommandFresh =
                motionCommand_.lastTurnCommandMs != 0 &&
                now - motionCommand_.lastTurnCommandMs <= kTurnCommandTimeoutMs;
        targetYawRate_ = turnCommandFresh
                             ? constrain(
                                 motionCommand_.targetYawRate,
                                 -kMaximumTargetYawRate,
                                 kMaximumTargetYawRate
                             )
                             : 0.0f;
        // 保留数值为 1:1 的转向前馈以维持快速起转；PID 的目标值本身不做转换。
        const float turnFeedForward = targetYawRate_;
        const float yawRate = kYawRateSign * mpu6050_.gyroZ();
        turnPid_.setTarget(targetYawRate_);
        turnPid_.setActual(yawRate);
        turnPid_.setKp(parameters_.turnKp);
        turnPid_.setKi(parameters_.turnKi);
        turnPid_.setKd(parameters_.turnKd);
        turnPid_.update(controlDt);
        const float turnCorrection = turnPid_.output();
        turnOutput_ = constrain(
            turnFeedForward + turnCorrection,
            -kMaximumMixedTurnOutput,
            kMaximumMixedTurnOutput
        );

        motors_.drive(balancePwm, turnOutput_);

        now = millis();
        if (now - lastTelemetryMs >= kTelemetryPeriodMs) {
            lastTelemetryMs = now;
            ble_.notifyStatus(
                rawAngle,
                BatteryMonitor::readPercent(),
                balancePwm,
                leftSpeed_,
                rightSpeed_
            );
            Serial.printf(
                "angle: %.2f(raw %.2f) target: %.2f speed: %.2f/%.2f "
                "pwm: %d yaw: %.1f/%.1f turnFF/PID/out: %.1f/%.1f/%.1f\n",
                angle,
                rawAngle,
                targetAngle_,
                speed_,
                speedReference_,
                balancePwm,
                yawRate,
                targetYawRate_,
                turnFeedForward,
                turnCorrection,
                turnOutput_
            );
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void BalanceController::updateOuterLoops(const uint32_t nowMs) {
    // dt 仅用于限制蓝牙下发的目标速度的变化率。编码器速度仍按每次采样的脉冲数表示，
    // 因此修改 kSpeedUpdatePeriodMs 后必须重新调整 speedKp。
    const float dt = (nowMs - lastOuterLoopUpdateMs_) * 0.001f;
    lastOuterLoopUpdateMs_ = nowMs;

    // 原子地取出并清零本周期脉冲，随后统一左右编码器的正方向。
    int32_t leftCount = 0;
    int32_t rightCount = 0;
    encoders_.readAndReset(leftCount, rightCount);
    leftCount *= kLeftEncoderSign;
    rightCount *= kRightEncoderSign;
    // 一阶低通滤波抑制编码器量化抖动。0.7 保留历史，0.3 引入本次测量。
    leftSpeed_ = 0.7f * leftSpeed_ + 0.3f * leftCount;
    rightSpeed_ = 0.7f * rightSpeed_ + 0.3f * rightCount;
    speed_ = 0.5f * (leftSpeed_ + rightSpeed_);
    // 遥控速度先经过变化率限制，避免目标速度突变导致小车猛然倾斜。
    const float maximumSpeedStep = parameters_.speedSlew * dt;
    const float speedCommandError =
            motionCommand_.targetSpeed - targetSpeed_;
    targetSpeed_ += constrain(
        speedCommandError,
        -maximumSpeedStep,
        maximumSpeedStep
    );

    // 速度 P 环把目标速度与实测速度之差转换为目标倾角。
    // 前面的负号由当前 IMU 正方向和电机安装方向决定：若推车后的回正方向完全相反，
    // 应先核对编码器符号和电机方向，而不是盲目改变 PID 增益符号。
    speedReference_ = targetSpeed_;
    speedPid_.setKp(-parameters_.speedKp);
    speedPid_.setKi(-parameters_.speedKi);
    speedPid_.setKd(-parameters_.speedKd);
    speedPid_.setTarget(speedReference_);
    speedPid_.setActual(speed_);
    speedPid_.update(dt);
    targetAngle_ = speedPid_.output();
}

void BalanceController::resetMotionState() {
    int32_t ignoredLeft = 0;
    int32_t ignoredRight = 0;
    // 丢弃倾倒或校准期间积累的脉冲，否则恢复控制后的第一次速度计算会出现尖峰。
    encoders_.readAndReset(ignoredLeft, ignoredRight);

    leftSpeed_ = 0.0f;
    rightSpeed_ = 0.0f;
    speed_ = 0.0f;
    speedReference_ = 0.0f;
    targetAngle_ = 0.0f;
    targetYawRate_ = 0.0f;
    turnOutput_ = 0.0f;
    anglePid_.reset();
    speedPid_.reset();
    turnPid_.reset();
    lastOuterLoopUpdateMs_ = millis();
    lastControlUpdateUs_ = micros();
}

void BalanceController::loadGyroOffsets() {
    // Preferences 的 true 表示只读。没有 NVS 记录时 getFloat 返回代码中的默认偏移。
    Preferences preferences;
    preferences.begin("gyro", true);
    const float offsetX =
            preferences.getFloat("ox", kDefaultGyroXOffset);
    const float offsetY =
            preferences.getFloat("oy", kDefaultGyroYOffset);
    const float offsetZ =
            preferences.getFloat("oz", kDefaultGyroZOffset);
    preferences.end();

    mpu6050_.setGyroOffsets(offsetX, offsetY, offsetZ);
    Serial.printf(
        "[GYRO] offsets ox=%.3f oy=%.3f oz=%.3f\n",
        offsetX,
        offsetY,
        offsetZ
    );
}

void BalanceController::saveGyroOffsets() {
    Preferences preferences;
    preferences.begin("gyro", false);
    preferences.putFloat("ox", mpu6050_.gyroXOffset());
    preferences.putFloat("oy", mpu6050_.gyroYOffset());
    preferences.putFloat("oz", mpu6050_.gyroZOffset());
    preferences.end();
}

void BalanceController::runGyroCalibration() {
    // 校准时车体必须静止，且电机必须停止；任何运动都会被当成陀螺仪零偏写入 NVS。
    motionCommand_.clear();
    targetSpeed_ = 0.0f;
    targetAngle_ = 0.0f;
    motors_.stop();

    if (!mpu6050_.calibrateGyro()) {
        mpu6050Ready_ = false;
        Serial.println("[GYRO] calibration failed because of an I2C error");
        resetMotionState();
        return;
    }
    saveGyroOffsets();
    resetMotionState();
    ble_.notifyCalibrationDone(
        mpu6050_.gyroXOffset(),
        mpu6050_.gyroYOffset(),
        mpu6050_.gyroZOffset()
    );
}

void BalanceController::runStatusLedLoop() const {
    bool blinkOn = false;
    uint32_t lastBlinkMs = 0;
    float breathPhase = 0.0f;

    while (true) {
        if (ble_.isConnected()) {
            constexpr float kBreathStep = 0.06f;
            constexpr float kTwoPi = 6.2831853f;
            // 已连接：使用正弦曲线生成平滑呼吸效果。
            breathPhase += kBreathStep;
            if (breathPhase > kTwoPi) {
                breathPhase -= kTwoPi;
            }
            const float wave = 0.5f * (1.0f + sinf(breathPhase));
            StatusLed::setDuty(static_cast<int>(20.0f + wave * 235.0f));
        } else {
            constexpr uint32_t kBlinkIntervalMs = 350;
            // 未连接：固定周期闪烁，提示设备仍在广播并等待连接。
            const uint32_t now = millis();
            if (now - lastBlinkMs >= kBlinkIntervalMs) {
                lastBlinkMs = now;
                blinkOn = !blinkOn;
                StatusLed::setDuty(
                    blinkOn ? kMaximumPwm : 0
                );
            }
            breathPhase = 0.0f;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
