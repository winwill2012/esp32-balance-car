#include "StatusLed.h"
#include <Arduino.h>
#include "Pins.h"

void StatusLed::begin() {
    // 指示灯独占一个 LEDC 通道，避免与四路电机 PWM 互相覆盖配置。
    ledcSetup(kStatusLedChannel, kStatusLedPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kStatusLed, kStatusLedChannel);
    ledcWrite(kStatusLedChannel, 0);
}

void StatusLed::setDuty(const int duty) {
    ledcWrite(kStatusLedChannel, constrain(duty, 0, kMaximumPwm));
}
