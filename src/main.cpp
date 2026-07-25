#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "pins.h"
#include "ble_config.h"
#include "MPU6050_tockn.h"

MPU6050 mpu6050(Wire);

int leftMotorDeadZone = 41; // 左电机死区
int rightMotorDeadZone = 41; // 右电机死区

int leftPwm = 0, rightPwm = 0; // 左右电机实际需要输出的pwm
// 允许小车倾斜的最大角度（固件固定，不再由小程序下发）
static constexpr float MAX_LEAN = 60.0f;
// 电池 ADC：分压比（BAT+ → R1 → ADC → R2 → GND，ratio=(R1+R2)/R2）
// R1 = 100K,R2 = 27K
static constexpr float BAT_DIVIDER_RATIO = 4.7037f;

// 内环： PWM为目标输出
float kp = 25, ki = 0, kd = 0.5;
float pwmOut;
// 机械零点：车真正站稳时传感器倾角（°），控制用 raw - angleOffset
float angleOffset = 0.0f;

// 外环：速度 → 目标倾角
float kv = 0.7f; // 先从很小试起
float targetSpeedCmd = 0.0f; // BLE 下发的目标速度（可突变）
float targetSpeed = 0.0f; // 实际参与控制的目标速度（斜坡跟随后）
// 建议固定周期算速度，比如每 10ms
const uint32_t SPEED_DT_MS = 10;
static uint32_t lastSpeedMs = 0;
static float speed = 0; // 滤波后的车速（脉冲/周期 或 换算后的单位）
static float leftSpeed = 0;
static float rightSpeed = 0;
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

    // 信号指示灯：用独立 LEDC 通道做闪烁 / 呼吸
    ledcSetup(CH_LED, LEDC_LED_FREQ_HZ, LEDC_BITS_WIDTH);
    ledcAttachPin(LED, CH_LED);
    ledcWrite(CH_LED, 0);

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

static void loadGyroOffsets() {
    Preferences prefs;
    prefs.begin("gyro", true);
    if (prefs.isKey("ox")) {
        const float ox = prefs.getFloat("ox", -0.47f);
        const float oy = prefs.getFloat("oy", 0.55f);
        const float oz = prefs.getFloat("oz", -1.42f);
        mpu6050.setGyroOffsets(ox, oy, oz);
        Serial.printf("[GYRO] NVS 加载偏移 ox=%.3f oy=%.3f oz=%.3f\n", ox, oy, oz);
    } else {
        mpu6050.setGyroOffsets(-0.47f, 0.55f, -1.42f);
        Serial.println("[GYRO] 使用默认偏移");
    }
    prefs.end();
}

static void saveGyroOffsets() {
    Preferences prefs;
    prefs.begin("gyro", false);
    prefs.putFloat("ox", mpu6050.getGyroXoffset());
    prefs.putFloat("oy", mpu6050.getGyroYoffset());
    prefs.putFloat("oz", mpu6050.getGyroZoffset());
    prefs.end();
    Serial.printf("[GYRO] 已保存偏移 ox=%.3f oy=%.3f oz=%.3f\n",
                  mpu6050.getGyroXoffset(),
                  mpu6050.getGyroYoffset(),
                  mpu6050.getGyroZoffset());
}

static void runGyroCalibration() {
    targetSpeedCmd = 0.0f;
    targetSpeed = 0.0f;
    turnPwm = 0.0f;
    pwmOut = 0.0f;
    targetAngle = 0.0f;
    stopMotors();

    // 小程序侧已做 3 秒准备倒计时；此处不再额外 delay
    mpu6050.calcGyroOffsets(false, 0, 0);
    saveGyroOffsets();
    bleConfigNotifyCalibDone(
        mpu6050.getGyroXoffset(),
        mpu6050.getGyroYoffset(),
        mpu6050.getGyroZoffset()
    );
}

static void loadDeadZones() {
    Preferences prefs;
    prefs.begin("motor", true);
    if (prefs.isKey("ldz")) {
        leftMotorDeadZone = prefs.getInt("ldz", leftMotorDeadZone);
    }
    if (prefs.isKey("rdz")) {
        rightMotorDeadZone = prefs.getInt("rdz", rightMotorDeadZone);
    }
    prefs.end();
    Serial.printf("[DZ] NVS 加载 左=%d 右=%d\n", leftMotorDeadZone, rightMotorDeadZone);
}

static void saveDeadZones() {
    Preferences prefs;
    prefs.begin("motor", false);
    prefs.putInt("ldz", leftMotorDeadZone);
    prefs.putInt("rdz", rightMotorDeadZone);
    prefs.end();
    Serial.printf("[DZ] 已保存到 NVS 左=%d 右=%d\n", leftMotorDeadZone, rightMotorDeadZone);
}

static void runDeadZoneDetection() {
    const int prevLeft = leftMotorDeadZone;
    const int prevRight = rightMotorDeadZone;

    targetSpeedCmd = 0.0f;
    targetSpeed = 0.0f;
    turnPwm = 0.0f;
    pwmOut = 0.0f;
    targetAngle = 0.0f;
    stopMotors();

    int foundLeft = -1;
    int foundRight = -1;
    Serial.println("[DZ] 开始电机死区检测，请保持轮子悬空");

    for (int pwm = 0; pwm <= static_cast<int>(MAX_PWM); pwm++) {
        int32_t leftCount = 0;
        int32_t rightCount = 0;
        readAndClearEncoderCounts(leftCount, rightCount);
        // 检测时不加死区补偿，直接输出原始 PWM
        writeMotorChannel(CH_L_IN1, CH_L_IN2, pwm, 0);
        writeMotorChannel(CH_R_IN1, CH_R_IN2, pwm, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        readAndClearEncoderCounts(leftCount, rightCount);
        Serial.printf("[DZ] PWM=%d 左计数=%ld 右计数=%ld\n",
                      pwm,
                      static_cast<long>(leftCount),
                      static_cast<long>(rightCount));

        if (foundLeft < 0 && abs(leftCount) >= 10) {
            foundLeft = pwm;
            Serial.printf("[DZ] 左电机启动 PWM=%d\n", pwm);
        }
        if (foundRight < 0 && abs(rightCount) >= 10) {
            foundRight = pwm;
            Serial.printf("[DZ] 右电机启动 PWM=%d\n", pwm);
        }
        if (foundLeft >= 0 && foundRight >= 0) {
            break;
        }
    }

    stopMotors();

    if (foundLeft >= 0 && foundRight >= 0) {
        leftMotorDeadZone = foundLeft;
        rightMotorDeadZone = foundRight;
        saveDeadZones();
        bleConfigNotifyDeadZoneDone(leftMotorDeadZone, rightMotorDeadZone);
        Serial.printf("[DZ] 检测完成 左=%d 右=%d\n", leftMotorDeadZone, rightMotorDeadZone);
    } else {
        leftMotorDeadZone = prevLeft > 0 ? prevLeft : 41;
        rightMotorDeadZone = prevRight > 0 ? prevRight : 41;
        // 仍通知当前值，便于小程序结束等待；DZ=1 表示流程结束
        bleConfigNotifyDeadZoneDone(leftMotorDeadZone, rightMotorDeadZone);
        Serial.printf("[DZ] 检测未完成，保留原值 左=%d 右=%d\n",
                      leftMotorDeadZone, rightMotorDeadZone);
    }
}

void driveMotor(const int pwm) {
    // turnPwm>0：左轮加快、右轮减慢 → 右转
    leftPwm = constrain(pwm + static_cast<int>(turnPwm), -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
    rightPwm = constrain(pwm - static_cast<int>(turnPwm), -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
    writeMotorChannel(CH_L_IN1, CH_L_IN2, leftPwm, leftMotorDeadZone);
    writeMotorChannel(CH_R_IN1, CH_R_IN2, rightPwm, rightMotorDeadZone);
}

// 兼容旧接口：改为同步检测，由 PID 任务调用
void detectDeadZone() {
    runDeadZoneDetection();
}

void startStatusLedProcess() {
    xTaskCreate([](void *) {
        bool blinkOn = false;
        uint32_t lastBlinkMs = 0;
        float breathPhase = 0.0f;
        const uint32_t blinkIntervalMs = 350;
        const float breathStep = 0.06f; // 约 2~3 秒一个呼吸周期

        while (true) {
            if (bleConfigIsConnected()) {
                // 已连接：正弦呼吸灯
                breathPhase += breathStep;
                if (breathPhase > 6.2831853f) {
                    breathPhase -= 6.2831853f;
                }
                const float wave = 0.5f * (1.0f + sinf(breathPhase));
                // 略抬高最低亮度，避免“熄灭感”
                const int duty = static_cast<int>(20.0f + wave * 235.0f);
                ledcWrite(CH_LED, constrain(duty, 0, static_cast<int>(MAX_PWM)));
                vTaskDelay(pdMS_TO_TICKS(20));
            } else {
                // 未连接：持续闪烁
                const uint32_t now = millis();
                if (now - lastBlinkMs >= blinkIntervalMs) {
                    lastBlinkMs = now;
                    blinkOn = !blinkOn;
                    ledcWrite(CH_LED, blinkOn ? static_cast<int>(MAX_PWM) : 0);
                }
                breathPhase = 0.0f;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }, "StatusLed", 2048, nullptr, 1, nullptr);
}

void startPIDProcess() {
    xTaskCreate([](void *) {
        while (true) {
            if (bleConfigConsumeGyroCalibRequest()) {
                runGyroCalibration();
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            if (bleConfigConsumeDeadZoneRequest()) {
                runDeadZoneDetection();
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            mpu6050.update();
            const float rawAngle = mpu6050.getAngleX();
            // 扣除机械零点后参与内环；倾倒保护仍看原始倾角
            const float angle = rawAngle - angleOffset;
            // 小车倾斜角度超过阈值，应该救不回来了，电机直接熄火
            if (abs(rawAngle) > MAX_LEAN) {
                pwmOut = 0;
                targetSpeed = 0;
                stopMotors();
                static uint32_t lastFallenNotifyMs = 0;
                const uint32_t nowFallen = millis();
                if (nowFallen - lastFallenNotifyMs >= 100) {
                    lastFallenNotifyMs = nowFallen;
                    bleConfigNotifyStatus(rawAngle, readBatteryPercent(), 0, 0, 0);
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            uint32_t now = millis();
            // 外环速度控制
            if (now - lastSpeedMs >= SPEED_DT_MS) {
                const float dt = (now - lastSpeedMs) * 0.001f;
                lastSpeedMs = now;
                int32_t leftCount = 0, rightCount = 0;
                readAndClearEncoderCounts(leftCount, rightCount);
                // 两轮平均；注意左右编码器方向是否相反，必要时一边取负
                // 简单低通，减弱编码器噪声，保持速度变化趋势即可
                leftSpeed = 0.7f * leftSpeed + 0.3f * leftCount;
                rightSpeed = 0.7f * rightSpeed + 0.3f * rightCount;
                speed = 0.5f * (leftSpeed + rightSpeed);

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
                bleConfigNotifyStatus(rawAngle, batPct, pwmOut, leftSpeed, rightSpeed);
                Serial.printf("倾角: %.2f(raw %.2f) 目标角: %.2f spd: %.2f/%.2f pwm: %.1f turn: %.1f\n",
                              angle, rawAngle, targetAngle, targetSpeed, targetSpeedCmd, pwmOut, turnPwm);
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
    loadGyroOffsets();
    loadDeadZones();
    // delay(3000);
    bleConfigBegin(
        &kp,
        &kd,
        &kv,
        &angleOffset,
        &leftMotorDeadZone,
        &rightMotorDeadZone,
        &targetSpeedCmd,
        &turnPwm,
        &speedSlew
    );

    // 启动信号灯与 PID 控制进程
    startStatusLedProcess();
    startPIDProcess();
}

void loop() {
}
