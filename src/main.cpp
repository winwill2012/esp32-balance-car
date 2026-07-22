#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "ble_config.h"
#include "MPU6050_tockn.h"

MPU6050 mpu6050(Wire);

int leftMotorDeadZone = 42; // 左电机死区
int rightMotorDeadZone = 36; // 右电机死区

int leftPwm = 0, rightPwm = 0; // 左右电机实际需要输出的pwm
// 允许小车倾斜的最大角度（固件固定，不再由小程序下发）
static constexpr float MAX_LEAN = 60.0f;
// 电池 ADC：分压比（BAT+ → R1 → ADC → R2 → GND，ratio=(R1+R2)/R2）
// R1 = 100K,R2 = 27K
static constexpr float BAT_DIVIDER_RATIO = 4.7037f;

// 内环： PWM为目标输出
float kp = 25, ki = 0, kd = 0.5;
float pwmOut;

// 外环：速度 → 目标倾角
float kv = 0.7f; // 先从很小试起
float targetSpeedCmd = 0.0f; // BLE 下发的目标速度（可突变）
float targetSpeed = 0.0f; // 实际参与控制的目标速度（斜坡跟随后）
// 建议固定周期算速度，比如每 10ms
const uint32_t SPEED_DT_MS = 10;
static uint32_t lastSpeedMs = 0;
static float speed = 0; // 滤波后的车速（脉冲/周期 或 换算后的单位）
static float targetAngle = 0;

// 遥控突变保护：限制目标速度变化率，避免急加速/急反向把车扯倒
// 急刹强度（每秒最大变化量），值越大越剧烈；可由小程序 SLW= 配置
float speedSlew = 25.0f;
// 外环输出的目标倾角限幅（远小于倾倒阈值，急减速时也不要猛仰/猛俯）
static constexpr float MAX_TARGET_ANGLE = 45.0f;

// 转向差速（遥控写入，松手/断连归零；正值右转）
float turnPwm = 0.0f;

// 电机编码计数器访问互斥锁
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile long leftMotorCount = 0; // 左电机转速计数器
volatile long rightMotorCount = 0; // 右电机转速计数器

// 读取电池电量百分比
float readBatteryPercent() {
    constexpr int samples = 16;
    uint32_t sumMv = 0;
    for (int i = 0; i < samples; ++i) {
        sumMv += analogReadMilliVolts(BAT_DETECT);
    }
    const float vSense = (sumMv / static_cast<float>(samples)) / 1000.0f;
    const float vBat = vSense * BAT_DIVIDER_RATIO;
    constexpr float vMin = 6.0; // 2S电池最小电压
    constexpr float vMax = 8.4; // 2S电池最大电压

    const float pct = (vBat - vMin) / (vMax - vMin) * 100.0f;
    return constrain(pct, 0.0f, 100.0f);
}

// 左电机编码器中断处理函数
void IRAM_ATTR leftMotorCountISR() {
    const bool channelB = digitalRead(MOTOR_L_EN2);
    portENTER_CRITICAL_ISR(&encoderMux);
    leftMotorCount += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

// 右电机编码器中断处理函数
void IRAM_ATTR rightMotorCountISR() {
    const bool channelB = digitalRead(MOTOR_R_EN2);
    portENTER_CRITICAL_ISR(&encoderMux);
    rightMotorCount += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

// 读取编码器计数值并且重置为0，这个是定时读取
void readAndClearEncoderCounts(int32_t &left, int32_t &right) {
    portENTER_CRITICAL(&encoderMux);
    left = leftMotorCount;
    right = rightMotorCount;
    leftMotorCount = 0;
    rightMotorCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}

void setupPins() {
    pinMode(BAT_DETECT, INPUT);
    analogSetPinAttenuation(BAT_DETECT, ADC_11db);
    pinMode(LED, OUTPUT);
    pinMode(MOTOR_L_EN1, INPUT);
    pinMode(MOTOR_L_EN2, INPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);

    pinMode(MOTOR_R_EN1, INPUT);
    pinMode(MOTOR_R_EN2, INPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    ledcSetup(CH_L_IN1, LEDC_FREQ_HZ, LEDC_BITS_WIDTH);
    ledcAttachPin(MOTOR_L_IN1, CH_L_IN1);

    ledcSetup(CH_L_IN2, LEDC_FREQ_HZ, LEDC_BITS_WIDTH);
    ledcAttachPin(MOTOR_L_IN2, CH_L_IN2);

    ledcSetup(CH_R_IN1, LEDC_FREQ_HZ, LEDC_BITS_WIDTH);
    ledcAttachPin(MOTOR_R_IN1, CH_R_IN1);

    ledcSetup(CH_R_IN2, LEDC_FREQ_HZ, LEDC_BITS_WIDTH);
    ledcAttachPin(MOTOR_R_IN2, CH_R_IN2);

    attachInterrupt(digitalPinToInterrupt(MOTOR_L_EN1), leftMotorCountISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_R_EN1), rightMotorCountISR, FALLING);
}

void writeMotorChannel(const int chIn1, const int chIn2, const int pwm, const int deadZone) {
    if (pwm > 0) {
        const int out = constrain(pwm + deadZone, 0, static_cast<int>(MAX_PWM));
        ledcWrite(chIn1, 0);
        ledcWrite(chIn2, out);
    } else if (pwm < 0) {
        const int out = constrain(-pwm + deadZone, 0, static_cast<int>(MAX_PWM));
        ledcWrite(chIn1, out);
        ledcWrite(chIn2, 0);
    } else {
        ledcWrite(chIn1, 0);
        ledcWrite(chIn2, 0);
    }
}

void stopMotors() {
    leftPwm = 0;
    rightPwm = 0;
    ledcWrite(CH_L_IN1, 0);
    ledcWrite(CH_L_IN2, 0);
    ledcWrite(CH_R_IN1, 0);
    ledcWrite(CH_R_IN2, 0);
}

void driveMotor(const int pwm) {
    // turnPwm>0：左轮加快、右轮减慢 → 右转
    leftPwm = constrain(pwm + static_cast<int>(turnPwm), -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
    rightPwm = constrain(pwm - static_cast<int>(turnPwm), -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
    writeMotorChannel(CH_L_IN1, CH_L_IN2, leftPwm, leftMotorDeadZone);
    writeMotorChannel(CH_R_IN1, CH_R_IN2, rightPwm, rightMotorDeadZone);
}

// 检测电机死区
void detectDeadZone() {
    xTaskCreate([](void *) {
        int leftDeadZone = -1;
        int rightDeadZone = -1;
        Serial.println("开始电机死区检测");
        for (int pwm = 0; pwm <= MAX_PWM; pwm++) {
            int32_t leftCount;
            int32_t rightCount;
            // 清除上一个 PWM 档位留下的计数
            readAndClearEncoderCounts(leftCount, rightCount);
            driveMotor(pwm);
            vTaskDelay(pdMS_TO_TICKS(100));
            readAndClearEncoderCounts(leftCount, rightCount);
            Serial.printf("PWM=%d, 左计数=%ld, 右计数=%ld\n", pwm,
                          static_cast<long>(leftCount),
                          static_cast<long>(rightCount));

            if (leftDeadZone < 0 &&
                abs(leftCount) >= 10) {
                leftDeadZone = pwm;
                Serial.printf("左电机启动 PWM=%d\n", pwm);
            }

            if (rightDeadZone < 0 &&
                abs(rightCount) >= 10) {
                rightDeadZone = pwm;
                Serial.printf("右电机启动 PWM=%d\n", pwm);
            }

            if (leftDeadZone >= 0 && rightDeadZone >= 0) {
                break;
            }
        }
        driveMotor(0);
        Serial.printf("检测完成：左电机死区=%d，右电机死区=%d\n", leftDeadZone, rightDeadZone);
        vTaskDelete(nullptr);
    }, "detectDeadZone", 4096, nullptr, 1, nullptr);
}

void startPIDProcess() {
    xTaskCreate([](void *) {
        while (true) {
            mpu6050.update();
            const float angle = mpu6050.getAngleX();
            // 小车倾斜角度超过阈值，应该救不回来了，电机直接熄火
            if (abs(angle) > MAX_LEAN) {
                pwmOut = 0;
                targetSpeed = 0;
                stopMotors();
                static uint32_t lastFallenNotifyMs = 0;
                const uint32_t nowFallen = millis();
                if (nowFallen - lastFallenNotifyMs >= 100) {
                    lastFallenNotifyMs = nowFallen;
                    bleConfigNotifyStatus(angle, readBatteryPercent(), 0);
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                return;
            }
            uint32_t now = millis();
            // 外环速度控制
            if (now - lastSpeedMs >= SPEED_DT_MS) {
                const float dt = (now - lastSpeedMs) * 0.001f;
                lastSpeedMs = now;
                int32_t leftCount = 0, rightCount = 0;
                readAndClearEncoderCounts(leftCount, rightCount);
                // 两轮平均；注意左右编码器方向是否相反，必要时一边取负
                const float rawSpeed = 0.5f * (leftCount + rightCount);
                // 简单低通，减弱编码器噪声，保持速度变化趋势即可
                speed = 0.7f * speed + 0.3f * rawSpeed;

                // 目标速度斜坡：遥控可瞬间反向，控制量平滑过渡
                const float maxDelta = speedSlew * dt;
                const float speedGap = targetSpeedCmd - targetSpeed;
                targetSpeed += constrain(speedGap, -maxDelta, maxDelta);

                // 外环：速度误差 → 目标倾角
                // speed > 0 时，让车往负方向倾一点，把车“刹住”
                const float speedErr = targetSpeed - speed;
                targetAngle = -kv * speedErr;
                targetAngle = constrain(targetAngle, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);
            }
            const float err = targetAngle - angle;
            /** 这里的kd的符号的判断方法：
             * 先定kp的值，勉强能直立，稍微给kd一个很小的值，如果有改善，符号对，如果恶化了，表示符号反了
            **/
            pwmOut = kp * err - kd * mpu6050.getGyroX();
            driveMotor(static_cast<int>(pwmOut));

            static uint32_t lastNotifyMs = 0;
            now = millis();
            if (now - lastNotifyMs >= 100) {
                lastNotifyMs = now;
                const float batPct = readBatteryPercent();
                bleConfigNotifyStatus(angle, batPct, pwmOut);
                Serial.printf("倾角: %.2f 目标角: %.2f spd: %.2f/%.2f pwm: %.1f turn: %.1f\n",
                              angle, targetAngle, targetSpeed, targetSpeedCmd, pwmOut, turnPwm);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }, "PIDProcess", 4096, nullptr, 1, nullptr);
}

void setup() {
    Serial.begin(115200);
    setupPins();
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    mpu6050.begin();
    // mpu6050.calcGyroOffsets(true);
    mpu6050.setGyroOffsets(-3.63, -0.75, -1.08);
    // delay(3000);
    // detectDeadZone();
    bleConfigBegin(&kp, &kd, &kv, &targetSpeedCmd, &turnPwm, &speedSlew);

    // 启动PID控制进程
    startPIDProcess();
}

void loop() {
}
