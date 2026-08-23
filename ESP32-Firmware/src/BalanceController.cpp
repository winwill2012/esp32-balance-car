#include "BalanceController.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <cmath>

#include "Pins.h"

// 安全和限幅参数：角度单位为度，速度单位为“脉冲/速度采样周期”。
// 倾倒判断使用 IMU 原始角度，避免错误的机械零点设置屏蔽倾倒保护。
constexpr float kMaximumLeanAngle = 60.0f;
// 速度环最多只能要求内环倾斜 30 度，防止大速度误差产生危险目标角。
constexpr float kMaximumTargetAngle = 30.0f;
// 速度环计算周期，单位ms
constexpr uint32_t kSpeedUpdatePeriodMs = 20;
// BLE 与串口遥测不参与控制，降低到 10 Hz 以减少通知和打印对控制周期的干扰。
constexpr uint32_t kTelemetryPeriodMs = 100;
// 方向归一化后的约定：向同一平移方向转动时 LSP/RSP 必须同号。
// 若手推小车前进时某一侧遥测符号相反，只把对应常量改为 -1。
constexpr int kLeftEncoderSign = 1;
constexpr int kRightEncoderSign = 1;

// 尚未执行陀螺仪校准时使用的兜底值；成功校准后由 NVS 中的数据覆盖。
constexpr float kDefaultGyroXOffset = -0.47f;
constexpr float kDefaultGyroYOffset = 0.55f;
constexpr float kDefaultGyroZOffset = -1.42f;

BalanceController::BalanceController()
    : imu_(Wire) {
}

void BalanceController::begin() {
    // 先初始化执行器并确保 PWM 为 0，再启动传感器和控制任务，防止上电误动作。
    motors_.begin();
    encoders_.begin();
    battery_.begin();
    statusLed_.begin();

    Wire.begin(kMpuSda, kMpuScl);
    Wire.setClock(400000);
    imuReady_ = imu_.begin();
    if (!imuReady_) {
        Serial.println("[IMU] MPU6050 not found; motors will remain stopped");
    }
    loadGyroOffsets();
    loadDeadZones();
    ble_.begin(parameters_, motionCommand_);

    lastSpeedUpdateMs_ = millis();
    // LED 和控制使用独立 FreeRTOS 任务；任务入口接收 this，避免使用全局控制状态。
    xTaskCreate(
        statusLedTaskEntry,
        "StatusLed",
        2048,
        this,
        1,
        nullptr
    );
    xTaskCreate(
        controlTaskEntry,
        "BalanceControl",
        4096,
        this,
        1,
        nullptr
    );
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
        if (!imuReady_) {
            motors_.stop();
            const uint32_t now = millis();
            if (now - lastImuRetryMs >= 1000) {
                lastImuRetryMs = now;
                imuReady_ = imu_.begin();
                Serial.printf(
                    "[IMU] initialization %s\n",
                    imuReady_ ? "succeeded" : "failed"
                );
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
        // 电机死区检测
        if (ble_.consumeDeadZoneRequest()) {
            runDeadZoneDetection();
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (!imu_.update()) {
            // I2C 读取异常时禁止使用旧姿态继续驱动；下一循环会重新初始化传感器。
            imuReady_ = false;
            motors_.stop();
            continue;
        }
        const float rawAngle = imu_.angleX();  // 获取俯仰角 Pitch
        // angleOffset 表示机械结构真正直立时 IMU 的读数，而不是陀螺仪零偏。
        const float angle = rawAngle - parameters_.angleOffset;

        // 倾倒后立即关闭电机，并清除速度/位置历史，避免扶起瞬间使用旧误差猛冲。
        if (fabsf(rawAngle) > kMaximumLeanAngle) {
            targetSpeed_ = 0.0f;
            targetAngle_ = 0.0f;
            resetMotionState();
            motors_.stop();

            const uint32_t now = millis();
            if (now - lastFallenTelemetryMs >= kTelemetryPeriodMs) {
                lastFallenTelemetryMs = now;
                ble_.notifyStatus(
                    rawAngle,
                    battery_.readPercent(),
                    0,
                    0.0f,
                    0.0f
                );
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t now = millis();
        // 编码器速度噪声较大，外环固定约 20 ms 更新一次；直立环则尽可能快地运行。
        if (now - lastSpeedUpdateMs_ >= kSpeedUpdatePeriodMs) {
            updateOuterLoops(now);
        }

        // 最内层直立 PD：
        //   P 项纠正目标角与实际角的偏差；
        //   D 项直接使用陀螺仪角速度抑制快速摆动，避免再对角度做差分放大噪声。
        const float angleError = targetAngle_ - angle;
        const int balancePwm = static_cast<int>(
            parameters_.angleKp * angleError -
            parameters_.angleKd * imu_.gyroX()
        );
        motors_.drive(
            balancePwm,
            motionCommand_.turnPwm,
            parameters_.leftMotorDeadZone,
            parameters_.rightMotorDeadZone
        );

        now = millis();
        if (now - lastTelemetryMs >= kTelemetryPeriodMs) {
            lastTelemetryMs = now;
            ble_.notifyStatus(
                rawAngle,
                battery_.readPercent(),
                balancePwm,
                leftSpeed_,
                rightSpeed_
            );
            Serial.printf(
                "angle: %.2f(raw %.2f) target: %.2f speed: %.2f/%.2f "
                "pwm: %d turn: %.1f\n",
                angle,
                rawAngle,
                targetAngle_,
                speed_,
                speedReference_,
                balancePwm,
                motionCommand_.turnPwm
            );
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void BalanceController::updateOuterLoops(const uint32_t nowMs) {
    // dt 仅用于限制蓝牙下发的目标速度的变化率。编码器速度仍按每次采样的脉冲数表示，
    // 因此修改 kSpeedUpdatePeriodMs 后必须重新调整 speedKp。
    const float dt = (nowMs - lastSpeedUpdateMs_) * 0.001f;
    lastSpeedUpdateMs_ = nowMs;

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
    const float speedError = speedReference_ - speed_;
    targetAngle_ = constrain(
        -parameters_.speedKp * speedError,
        -kMaximumTargetAngle,
        kMaximumTargetAngle
    );
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
    lastSpeedUpdateMs_ = millis();
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

    imu_.setGyroOffsets(offsetX, offsetY, offsetZ);
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
    preferences.putFloat("ox", imu_.gyroXOffset());
    preferences.putFloat("oy", imu_.gyroYOffset());
    preferences.putFloat("oz", imu_.gyroZOffset());
    preferences.end();
}

void BalanceController::runGyroCalibration() {
    // 校准时车体必须静止，且电机必须停止；任何运动都会被当成陀螺仪零偏写入 NVS。
    motionCommand_.clear();
    targetSpeed_ = 0.0f;
    targetAngle_ = 0.0f;
    motors_.stop();

    if (!imu_.calibrateGyro()) {
        imuReady_ = false;
        Serial.println("[GYRO] calibration failed because of an I2C error");
        resetMotionState();
        return;
    }
    saveGyroOffsets();
    resetMotionState();
    ble_.notifyCalibrationDone(
        imu_.gyroXOffset(),
        imu_.gyroYOffset(),
        imu_.gyroZOffset()
    );
}

void BalanceController::loadDeadZones() {
    Preferences preferences;
    preferences.begin("motor", true);
    parameters_.leftMotorDeadZone = preferences.getInt(
        "ldz",
        parameters_.leftMotorDeadZone
    );
    parameters_.rightMotorDeadZone = preferences.getInt(
        "rdz",
        parameters_.rightMotorDeadZone
    );
    preferences.end();
    Serial.printf(
        "[DZ] loaded left=%d right=%d\n",
        parameters_.leftMotorDeadZone,
        parameters_.rightMotorDeadZone
    );
}

void BalanceController::saveDeadZones() const {
    Preferences preferences;
    preferences.begin("motor", false);
    preferences.putInt("ldz", parameters_.leftMotorDeadZone);
    preferences.putInt("rdz", parameters_.rightMotorDeadZone);
    preferences.end();
}

bool BalanceController::detectDeadZoneOnce(
    const int round,
    int &leftResult,
    int &rightResult
) {
    // 标定前必须让车轮悬空。算法从 0 逐级增加原始 PWM，
    // 当 100 ms 内累计至少 10 个编码器脉冲时，认为对应电机已经可靠起转。
    int foundLeft = -1;
    int foundRight = -1;
    Serial.printf("[DZ] round %d started\n", round);

    for (int pwm = 0; pwm <= kMaximumPwm; ++pwm) {
        int32_t leftCount = 0;
        int32_t rightCount = 0;
        // 每一级测试前清除旧脉冲；driveRaw 不叠加已有死区，否则测不到真实起转点。
        encoders_.readAndReset(leftCount, rightCount);
        motors_.driveRaw(pwm, pwm);
        vTaskDelay(pdMS_TO_TICKS(100));
        encoders_.readAndReset(leftCount, rightCount);

        if (foundLeft < 0 && abs(leftCount) >= 10) {
            foundLeft = pwm;
        }
        if (foundRight < 0 && abs(rightCount) >= 10) {
            foundRight = pwm;
        }
        if (foundLeft >= 0 && foundRight >= 0) {
            break;
        }
    }

    motors_.stop();
    leftResult = foundLeft;
    rightResult = foundRight;
    Serial.printf(
        "[DZ] round %d finished left=%d right=%d\n",
        round,
        foundLeft,
        foundRight
    );
    return foundLeft >= 0 && foundRight >= 0;
}

void BalanceController::runDeadZoneDetection() {
    // 多次测量取均值，减小摩擦、齿轮啮合位置和电池电压造成的单次误差。
    constexpr int kDetectionRounds = 3;
    const int previousLeft = parameters_.leftMotorDeadZone;
    const int previousRight = parameters_.rightMotorDeadZone;

    motionCommand_.clear();
    targetSpeed_ = 0.0f;
    targetAngle_ = 0.0f;
    motors_.stop();

    int leftSum = 0;
    int rightSum = 0;
    int successfulRounds = 0;
    for (int round = 1; round <= kDetectionRounds; ++round) {
        int leftResult = -1;
        int rightResult = -1;
        if (detectDeadZoneOnce(round, leftResult, rightResult)) {
            leftSum += leftResult;
            rightSum += rightResult;
            ++successfulRounds;
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    if (successfulRounds > 0) {
        // 加上 successfulRounds / 2 实现整数四舍五入，而不是直接向下取整。
        parameters_.leftMotorDeadZone =
                (leftSum + successfulRounds / 2) / successfulRounds;
        parameters_.rightMotorDeadZone =
                (rightSum + successfulRounds / 2) / successfulRounds;
        saveDeadZones();
    } else {
        // 检测完全失败时保留原值，避免把无效的 -1 写入控制参数。
        parameters_.leftMotorDeadZone =
                previousLeft > 0 ? previousLeft : 164;
        parameters_.rightMotorDeadZone =
                previousRight > 0 ? previousRight : 164;
    }

    ble_.notifyDeadZoneDone(
        parameters_.leftMotorDeadZone,
        parameters_.rightMotorDeadZone
    );
    resetMotionState();
}

void BalanceController::runStatusLedLoop() {
    bool blinkOn = false;
    uint32_t lastBlinkMs = 0;
    float breathPhase = 0.0f;
    constexpr uint32_t kBlinkIntervalMs = 350;
    constexpr float kBreathStep = 0.06f;
    constexpr float kTwoPi = 6.2831853f;

    while (true) {
        if (ble_.isConnected()) {
            // 已连接：使用正弦曲线生成平滑呼吸效果。
            breathPhase += kBreathStep;
            if (breathPhase > kTwoPi) {
                breathPhase -= kTwoPi;
            }
            const float wave = 0.5f * (1.0f + sinf(breathPhase));
            statusLed_.setDuty(static_cast<int>(20.0f + wave * 235.0f));
        } else {
            // 未连接：固定周期闪烁，提示设备仍在广播并等待连接。
            const uint32_t now = millis();
            if (now - lastBlinkMs >= kBlinkIntervalMs) {
                lastBlinkMs = now;
                blinkOn = !blinkOn;
                statusLed_.setDuty(
                    blinkOn ? kMaximumPwm : 0
                );
            }
            breathPhase = 0.0f;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
