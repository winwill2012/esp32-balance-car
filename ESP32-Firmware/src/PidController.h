#ifndef ESP32_BALANCE_CAR_PIDCONTROLLER_H
#define ESP32_BALANCE_CAR_PIDCONTROLLER_H

class PidController {
public:
    PidController(float kp, float ki, float kd, float integralLimit, float outLimit);

    void setKp(const float p) { this->kp = p; }

    void setKi(float i);

    void setKd(const float d) { this->kd = d; }

    void setIntegralLimit(const float limit) { this->errIntegralLimit = limit; }

    void setOutLimit(const float limit) { this->outLimit = limit; }

    void setTarget(const float t) { this->target = t; }

    void setActual(const float a) { this->actual = a; }

    void reset();

    void update(const float dtSeconds);

    // 使用外部提供的误差变化率更新。平衡环可直接传入陀螺仪角速度，
    // 避免对带噪声的角度再次差分。
    void updateWithDerivative(const float errorDerivative, const float dtSeconds);

    float output() const { return out; }

private:
    void updateErrorAndIntegral(float dtSeconds);

    void calculateOutput(float errorDerivative);

    float kp, ki, kd; // PID三个参数
    float target, actual, out; // 目标值，实际值，pid控制输出值
    float error, preError, errIntegral; // 本次误差，前次误差，误差积分
    float errIntegralLimit; // 误差积分项限幅
    float outLimit; // 输出限幅
};


#endif //ESP32_BALANCE_CAR_PIDCONTROLLER_H
