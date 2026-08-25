#include "PidController.h"
#include <Arduino.h>

PidController::PidController(const float kp, const float ki, const float kd,
                             const float integralLimit, const float outLimit) {
    this->error = 0;
    this->preError = 0;
    this->errIntegral = 0;
    this->actual = 0;
    this->target = 0;
    this->out = 0;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->errIntegralLimit = integralLimit;
    this->outLimit = outLimit;
}

void PidController::reset() {
    this->error = 0;
    this->preError = 0;
    this->errIntegral = 0;
    this->out = 0;
}

void PidController::setKi(const float i) {
    // 用户把 Ki 设为 0 表示关闭积分，同时清除历史累计，避免再次启用时突然跳变。
    if (i == 0.0f && this->ki != 0.0f) {
        this->errIntegral = 0.0f;
    }
    this->ki = i;
}

void PidController::update(const float dtSeconds) {
    updateErrorAndIntegral(dtSeconds);
    const float errorDerivative = dtSeconds > 0.0f
                                      ? (this->error - this->preError) / dtSeconds
                                      : 0.0f;
    calculateOutput(errorDerivative);
}

void PidController::updateWithDerivative(const float errorDerivative, const float dtSeconds) {
    updateErrorAndIntegral(dtSeconds);
    calculateOutput(errorDerivative);
}

void PidController::updateErrorAndIntegral(const float dtSeconds) {
    // 将当次误差赋值给上一次误差
    this->preError = this->error;
    // 当前误差重新计算
    this->error = this->target - this->actual;

    // 累计误差
    if (dtSeconds > 0.0f) {
        this->errIntegral += this->error * dtSeconds;
    }
    // 积分限幅
    this->errIntegral = constrain(this->errIntegral, -this->errIntegralLimit, this->errIntegralLimit);
}

void PidController::calculateOutput(const float errorDerivative) {
    // PID算出输出
    this->out = this->kp * this->error
                + this->ki * this->errIntegral
                + this->kd * errorDerivative;
    // 输出限幅
    this->out = constrain(this->out, -this->outLimit, this->outLimit);
}
