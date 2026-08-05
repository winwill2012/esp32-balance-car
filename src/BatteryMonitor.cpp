#include "BatteryMonitor.h"

#include <Arduino.h>

#include "Pins.h"

// 分压比 = (R1 + R2) / R2；当前硬件按 R1=100k、R2=27k 计算。
constexpr float kDividerRatio = 4.7037f;
// 2S 锂电池用 6.0~8.4 V 做线性百分比显示。该值只适合趋势显示，
// 并不是考虑放电曲线和负载压降后的精确剩余容量估算。
constexpr float kEmptyVoltage = 6.0f;
constexpr float kFullVoltage = 8.4f;
constexpr int kSampleCount = 16;

void BatteryMonitor::begin() const {
    pinMode(kBatterySense, INPUT);
    analogSetPinAttenuation(kBatterySense, ADC_11db);
}

float BatteryMonitor::readPercent() const {
    // 多次读取毫伏校准值后取平均，降低电机 PWM 和 ADC 本身造成的瞬时抖动。
    uint32_t sumMillivolts = 0;
    for (int i = 0; i < kSampleCount; ++i) {
        sumMillivolts += analogReadMilliVolts(kBatterySense);
    }

    // ADC 引脚读到的是分压后的电压，乘分压比还原电池端电压。
    const float senseVoltage =
        (sumMillivolts / static_cast<float>(kSampleCount)) / 1000.0f;
    const float batteryVoltage = senseVoltage * kDividerRatio;
    const float percent =
        (batteryVoltage - kEmptyVoltage) /
        (kFullVoltage - kEmptyVoltage) * 100.0f;
    return constrain(percent, 0.0f, 100.0f);
}
