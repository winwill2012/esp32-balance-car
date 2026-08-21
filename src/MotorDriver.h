#ifndef ESP32_BALANCE_CAR_MOTOR_DRIVER_H
#define ESP32_BALANCE_CAR_MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
public:
    void begin();
    void drive(
        int balancePwm,
        float turnPwm,
        int leftDeadZone,
        int rightDeadZone
    );
    void driveRaw(int leftPwm, int rightPwm);
    void stop();

    int leftPwm() const { return leftPwm_; }
    int rightPwm() const { return rightPwm_; }

private:
    static int applySmoothDeadZone(int pwm, int deadZone);
    static void writeChannel(int channelIn1, int channelIn2, int pwm);

    int leftPwm_ = 0;
    int rightPwm_ = 0;
};

#endif // ESP32_BALANCE_CAR_MOTOR_DRIVER_H
