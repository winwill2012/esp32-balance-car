#include "MotorDriver.h"
#include "Pins.h"

// 平衡点附近的极小控制量主要来自传感器噪声，不驱动电机。
constexpr int kMotorCommandDeadband = 20;
// 固定的小幅摩擦补偿：不检测死区、不跟踪电机运动状态，也不加入启动冲击。
// 补偿随控制量从 0 平滑增加，在 100 PWM 时达到最大 50 PWM。
constexpr int kMaximumFrictionCompensation = 30;
constexpr int kFrictionCompensationBlendRange = 80;

void MotorDriver::begin() {
    // 每个电机由两路 PWM 驱动 H 桥：一侧输出 PWM，另一侧保持 0 来选择方向。
    pinMode(kLeftMotorIn1, OUTPUT);
    pinMode(kLeftMotorIn2, OUTPUT);
    pinMode(kRightMotorIn1, OUTPUT);
    pinMode(kRightMotorIn2, OUTPUT);

    ledcSetup(kLeftMotorIn1Channel, kMotorPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kLeftMotorIn1, kLeftMotorIn1Channel);
    ledcSetup(kLeftMotorIn2Channel, kMotorPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kLeftMotorIn2, kLeftMotorIn2Channel);
    ledcSetup(kRightMotorIn1Channel, kMotorPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kRightMotorIn1, kRightMotorIn1Channel);
    ledcSetup(kRightMotorIn2Channel, kMotorPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kRightMotorIn2, kRightMotorIn2Channel);

    stop();
}

void MotorDriver::drive(const int balancePwm, const float turnPwm) {
    // 差动混控约定：turnPwm > 0 时左轮增加、右轮减小，使车体向右转。
    // 最终输出先限幅，防止平衡 PWM 与转向 PWM 相加后超过 LEDC 范围。
    constexpr int maxPwm = kMaximumPwm;
    leftPwm_ = constrain(balancePwm + static_cast<int>(turnPwm), -maxPwm, maxPwm);
    rightPwm_ = constrain(balancePwm - static_cast<int>(turnPwm), -maxPwm, maxPwm);

    const int leftOutput = applySmoothFrictionCompensation(leftPwm_);
    const int rightOutput = applySmoothFrictionCompensation(rightPwm_);
    writeChannel(kLeftMotorIn1Channel, kLeftMotorIn2Channel, leftOutput);
    writeChannel(kRightMotorIn1Channel, kRightMotorIn2Channel, rightOutput);
}

void MotorDriver::stop() {
    leftPwm_ = 0;
    rightPwm_ = 0;
    ledcWrite(kLeftMotorIn1Channel, 0);
    ledcWrite(kLeftMotorIn2Channel, 0);
    ledcWrite(kRightMotorIn1Channel, 0);
    ledcWrite(kRightMotorIn2Channel, 0);
}

int MotorDriver::applySmoothFrictionCompensation(const int pwm) {
    const int magnitude = abs(pwm);
    if (magnitude <= kMotorCommandDeadband) {
        return 0;
    }

    const int blendMagnitude = min(magnitude, kFrictionCompensationBlendRange);
    const int compensation =
            (kMaximumFrictionCompensation * blendMagnitude +
             kFrictionCompensationBlendRange / 2) /
            kFrictionCompensationBlendRange;
    const int outputMagnitude = constrain(
        magnitude + compensation,
        0,
        kMaximumPwm
    );
    return pwm > 0 ? outputMagnitude : -outputMagnitude;
}

void MotorDriver::writeChannel(const int channelIn1, const int channelIn2, const int pwm) {
    if (pwm > 0) {
        const int output = pwm;
        ledcWrite(channelIn1, 0);
        ledcWrite(channelIn2, output);
    } else if (pwm < 0) {
        const int output = -pwm;
        ledcWrite(channelIn1, output);
        ledcWrite(channelIn2, 0);
    } else {
        ledcWrite(channelIn1, 0);
        ledcWrite(channelIn2, 0);
    }
}
