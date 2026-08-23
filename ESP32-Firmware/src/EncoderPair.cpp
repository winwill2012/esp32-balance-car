#include "EncoderPair.h"

#include "Pins.h"

EncoderPair *EncoderPair::instance_ = nullptr;

void EncoderPair::begin() {
    // Arduino attachInterrupt 只接受普通函数指针，无法直接绑定成员函数；
    // 保存唯一实例后，静态 ISR 才能访问本对象的计数器和临界区锁。
    instance_ = this;

    pinMode(kLeftEncoderA, INPUT);
    pinMode(kLeftEncoderB, INPUT);
    pinMode(kRightEncoderA, INPUT);
    pinMode(kRightEncoderB, INPUT);

    // 仅在 A 相下降沿计数，读取同一时刻的 B 相电平判断旋转方向。
    // 这种方式每个完整 AB 周期计一次，分辨率低于四倍频，但 ISR 更轻量。
    attachInterrupt(
        digitalPinToInterrupt(kLeftEncoderA),
        handleLeftInterrupt,
        FALLING
    );
    attachInterrupt(
        digitalPinToInterrupt(kRightEncoderA),
        handleRightInterrupt,
        FALLING
    );
}

void EncoderPair::readAndReset(int32_t &leftCount, int32_t &rightCount) {
    // “读取 + 清零”必须在同一临界区完成，否则恰好到来的中断脉冲可能丢失。
    portENTER_CRITICAL(&mux_);
    leftCount = leftCount_;
    rightCount = rightCount_;
    leftCount_ = 0;
    rightCount_ = 0;
    portEXIT_CRITICAL(&mux_);
}

void IRAM_ATTR EncoderPair::handleLeftInterrupt() {
    if (!instance_) {
        return;
    }

    const bool channelB = digitalRead(kLeftEncoderB);
    // ISR 使用专用临界区 API；IRAM_ATTR 保证中断处理代码可在关 Cache 时执行。
    portENTER_CRITICAL_ISR(&instance_->mux_);
    instance_->leftCount_ += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&instance_->mux_);
}

void IRAM_ATTR EncoderPair::handleRightInterrupt() {
    if (!instance_) {
        return;
    }

    const bool channelB = digitalRead(kRightEncoderB);
    portENTER_CRITICAL_ISR(&instance_->mux_);
    instance_->rightCount_ += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&instance_->mux_);
}
