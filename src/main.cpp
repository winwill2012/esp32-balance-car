#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "ble_config.h"
#include "MPU6050_tockn.h"

MPU6050 mpu6050(Wire);

int leftMotorDeadZone = 36; // 左电机死区
int rightMotorDeadZone = 47; // 右电机死区
int leftPwm = 0, rightPwm = 0; // 左右电机实际需要输出的pwm

// 内环： PWM为目标输出
float kp = 25, ki = 0, kd = 0.5;
float pwmOut;

// 外环：速度 → 目标倾角
float kv = 0.7f; // 先从很小试起
float targetSpeed = 0.0f; // 静止时为 0
float maxLean = 60.0f; // 限制目标倾角，防外环给太大角
// 建议固定周期算速度，比如每 10ms
const uint32_t SPEED_DT_MS = 10;
static uint32_t lastSpeedMs = 0;
static float speed = 0; // 滤波后的车速（脉冲/周期 或 换算后的单位）
static float targetAngle = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile long leftMotorCount = 0; // 左电机转速计数器
volatile long rightMotorCount = 0; // 右电机转速计数器

void IRAM_ATTR leftMotorCountISR() {
    const bool channelB = digitalRead(MOTOR_L_EN2);
    portENTER_CRITICAL_ISR(&encoderMux);
    leftMotorCount += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightMotorCountISR() {
    const bool channelB = digitalRead(MOTOR_R_EN2);
    portENTER_CRITICAL_ISR(&encoderMux);
    rightMotorCount += channelB ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

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

void driveMotor(const int pwm) {
    if (pwm > 0) {
        leftPwm = constrain(pwm + leftMotorDeadZone, -255, 255);
        rightPwm = constrain(pwm + rightMotorDeadZone, -255, 255);
        ledcWrite(CH_L_IN1, 0);
        ledcWrite(CH_L_IN2, leftPwm);
        ledcWrite(CH_R_IN1, 0);
        ledcWrite(CH_R_IN2, rightPwm);
    } else {
        leftPwm = constrain(pwm - leftMotorDeadZone, -255, 255);
        rightPwm = constrain(pwm - rightMotorDeadZone, -255, 255);
        ledcWrite(CH_L_IN1, -leftPwm);
        ledcWrite(CH_L_IN2, 0);
        ledcWrite(CH_R_IN1, -rightPwm);
        ledcWrite(CH_R_IN2, 0);
    }
}

// 检测电机死区
void detectDeadZone() {
    xTaskCreate([](void *) {
        int leftDeadZone = -1;
        int rightDeadZone = -1;
        Serial.println("开始电机死区检测");
        for (int pwm = 0; pwm <= 255; pwm++) {
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

void setup() {
    Serial.begin(115200);
    setupPins();
    Wire.begin(MPU6050_SDA, MPU6050_SCL);
    mpu6050.begin();
    // mpu6050.calcGyroOffsets(true);
    mpu6050.setGyroOffsets(-3.63, -0.75, -1.08);
    // detectDeadZone();
    bleConfigBegin(&kp, &kd, &kv, &maxLean);
}

void loop() {
    mpu6050.update();
    const float angle = mpu6050.getAngleX();
    // 小车倾斜角度超过60度，应该就不回来了，电机直接熄火
    if (abs(angle) > 60) {
        pwmOut = 0;
        driveMotor(0);
        return;
    }
    uint32_t now = millis();
    // 外环速度控制
    if (now - lastSpeedMs >= SPEED_DT_MS) {
        lastSpeedMs = now;
        int32_t leftCount = 0, rightCount = 0;
        readAndClearEncoderCounts(leftCount, rightCount);
        // 这里需要保证左右轮得到的计数值符号一致，如果不一致，下一行就要改成相减
        Serial.printf("左右轮count: %d, %d\n", leftCount, rightCount);
        // 两轮平均；注意左右编码器方向是否相反，必要时一边取负
        const float rawSpeed = 0.5f * (leftCount - rightCount);
        // 简单低通，减弱编码器噪声
        speed = 0.7f * speed + 0.3f * rawSpeed;
        // 外环：速度误差 → 目标倾角
        // speed > 0 时，让车往负方向倾一点，把车“刹住”
        float speedErr = targetSpeed - speed;
        targetAngle = -kv * speedErr;
        targetAngle = constrain(targetAngle, -maxLean, maxLean);
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
        bleConfigNotifyStatus(angle, targetAngle, pwmOut);
        Serial.printf("倾角: %.2f 目标角: %.2f pwm: %.1f kp: %.2f kd: %.2f kv: %.4f\n",
                      angle, targetAngle, pwmOut, kp, kd, kv);
    }
    delay(1);
}
