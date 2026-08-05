#ifndef ESP32_BALANCE_CAR_PINS_H
#define ESP32_BALANCE_CAR_PINS_H

#include <cstdint>

// MPU6050 的 I2C 与中断引脚。当前程序使用轮询更新，预留中断脚未启用。
constexpr uint8_t kMpuScl = 32;
constexpr uint8_t kMpuSda = 33;
constexpr uint8_t kMpuInterrupt = 25;

// 左右电机 H 桥输入和 AB 相编码器引脚。
constexpr uint8_t kLeftMotorIn1 = 22;
constexpr uint8_t kLeftMotorIn2 = 23;
constexpr uint8_t kLeftEncoderA = 13;
constexpr uint8_t kLeftEncoderB = 14;

constexpr uint8_t kRightMotorIn1 = 19;
constexpr uint8_t kRightMotorIn2 = 18;
constexpr uint8_t kRightEncoderA = 27;
constexpr uint8_t kRightEncoderB = 26;

// ESP32 LEDC 硬件 PWM 通道。每路 H 桥输入必须使用独立通道。
constexpr uint8_t kLeftMotorIn1Channel = 0;
constexpr uint8_t kLeftMotorIn2Channel = 1;
constexpr uint8_t kRightMotorIn1Channel = 2;
constexpr uint8_t kRightMotorIn2Channel = 3;
constexpr uint8_t kStatusLedChannel = 4;

// 10 位 PWM 的有效范围为 0~1023。
constexpr uint8_t kPwmResolutionBits = 10;
constexpr int kMaximumPwm = (1 << kPwmResolutionBits) - 1;
constexpr uint32_t kMotorPwmFrequencyHz = 1000;
constexpr uint32_t kStatusLedPwmFrequencyHz = 5000;

// 电池分压采样和板载状态指示灯引脚。
constexpr uint8_t kBatterySense = 34;
constexpr uint8_t kStatusLed = 21;

#endif
