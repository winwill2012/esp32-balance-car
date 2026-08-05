#include "MotorDriver.h"

#include "Pins.h"

void MotorDriver::begin() {
    // 每个电机由两路 PWM 驱动 H 桥：一侧输出 PWM，另一侧保持 0 来选择方向。
    pinMode(kLeftMotorIn1, OUTPUT);
    pinMode(kLeftMotorIn2, OUTPUT);
    pinMode(kRightMotorIn1, OUTPUT);
    pinMode(kRightMotorIn2, OUTPUT);

    ledcSetup(
        kLeftMotorIn1Channel,
        kMotorPwmFrequencyHz,
        kPwmResolutionBits
    );
    ledcAttachPin(kLeftMotorIn1, kLeftMotorIn1Channel);
    ledcSetup(
        kLeftMotorIn2Channel,
        kMotorPwmFrequencyHz,
        kPwmResolutionBits
    );
    ledcAttachPin(kLeftMotorIn2, kLeftMotorIn2Channel);
    ledcSetup(
        kRightMotorIn1Channel,
        kMotorPwmFrequencyHz,
        kPwmResolutionBits
    );
    ledcAttachPin(kRightMotorIn1, kRightMotorIn1Channel);
    ledcSetup(
        kRightMotorIn2Channel,
        kMotorPwmFrequencyHz,
        kPwmResolutionBits
    );
    ledcAttachPin(kRightMotorIn2, kRightMotorIn2Channel);

    stop();
}

void MotorDriver::drive(
    const int balancePwm,
    const float turnPwm,
    const int leftDeadZone,
    const int rightDeadZone
) {
    // 差动混控约定：turnPwm > 0 时左轮增加、右轮减小，使车体向右转。
    // 最终输出先限幅，防止平衡 PWM 与转向 PWM 相加后超过 LEDC 范围。
    constexpr int maxPwm = kMaximumPwm;
    leftPwm_ = constrain(balancePwm + static_cast<int>(turnPwm), -maxPwm, maxPwm);
    rightPwm_ = constrain(balancePwm - static_cast<int>(turnPwm), -maxPwm, maxPwm);

    writeChannel(
        kLeftMotorIn1Channel,
        kLeftMotorIn2Channel,
        leftPwm_,
        leftDeadZone
    );
    writeChannel(
        kRightMotorIn1Channel,
        kRightMotorIn2Channel,
        rightPwm_,
        rightDeadZone
    );
}

void MotorDriver::driveRaw(const int leftPwm, const int rightPwm) {
    // 原始模式不叠加死区，只供死区标定使用；正常平衡控制应调用 drive()。
    constexpr int maxPwm = kMaximumPwm;
    leftPwm_ = constrain(leftPwm, -maxPwm, maxPwm);
    rightPwm_ = constrain(rightPwm, -maxPwm, maxPwm);

    writeChannel(
        kLeftMotorIn1Channel,
        kLeftMotorIn2Channel,
        leftPwm_,
        0
    );
    writeChannel(
        kRightMotorIn1Channel,
        kRightMotorIn2Channel,
        rightPwm_,
        0
    );
}

void MotorDriver::stop() {
    // 两个 H 桥输入均置 0。此时是滑行还是制动取决于具体电机驱动芯片的真值表。
    leftPwm_ = 0;
    rightPwm_ = 0;
    ledcWrite(kLeftMotorIn1Channel, 0);
    ledcWrite(kLeftMotorIn2Channel, 0);
    ledcWrite(kRightMotorIn1Channel, 0);
    ledcWrite(kRightMotorIn2Channel, 0);
}

void MotorDriver::writeChannel(
    const int channelIn1,
    const int channelIn2,
    const int pwm,
    const int deadZone
) {
    if (pwm > 0) {
        // 只有非零控制量才叠加死区；pwm == 0 时必须保持真正的零输出，避免静止漂移。
        const int output = constrain(
            pwm + deadZone,
            0,
            kMaximumPwm
        );
        ledcWrite(channelIn1, 0);
        ledcWrite(channelIn2, output);
    } else if (pwm < 0) {
        const int output = constrain(
            -pwm + deadZone,
            0,
            kMaximumPwm
        );
        ledcWrite(channelIn1, output);
        ledcWrite(channelIn2, 0);
    } else {
        ledcWrite(channelIn1, 0);
        ledcWrite(channelIn2, 0);
    }
}
