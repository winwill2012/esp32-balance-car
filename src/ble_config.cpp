#include "ble_config.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

namespace {

Preferences prefs;
float *gKp = nullptr;
float *gKd = nullptr;
float *gKv = nullptr;
float *gAngleOffset = nullptr;
int *gLeftDz = nullptr;
int *gRightDz = nullptr;
float *gTargetSpeed = nullptr;
float *gTurnPwm = nullptr;
float *gSpeedSlew = nullptr;
float gFactoryKp = 80.0f;
float gFactoryKd = 1.2f;
float gFactoryKv = 0.68f;
float gFactoryA0 = 0.0f;
bool gConnected = false;
volatile bool gGyroCalibRequested = false;
volatile bool gDeadZoneRequested = false;

static constexpr float ANGLE_OFFSET_MIN = -5.0f;
static constexpr float ANGLE_OFFSET_MAX = 5.0f;

void clearMotionCmd() {
    if (gTargetSpeed) {
        *gTargetSpeed = 0.0f;
    }
    if (gTurnPwm) {
        *gTurnPwm = 0.0f;
    }
}

BLEServer *gServer = nullptr;
BLECharacteristic *gCmdChar = nullptr;
BLECharacteristic *gStatusChar = nullptr;

void applyParams(float kp, float kd, float kv) {
    if (!gKp || !gKd || !gKv) {
        return;
    }
    *gKp = kp;
    *gKd = kd;
    *gKv = kv;
    Serial.printf("[BLE] 参数已更新 kp=%.3f kd=%.3f kv=%.4f\n", kp, kd, kv);
}

void applyAngleOffset(float a0) {
    if (!gAngleOffset) {
        return;
    }
    if (a0 < ANGLE_OFFSET_MIN) {
        a0 = ANGLE_OFFSET_MIN;
    } else if (a0 > ANGLE_OFFSET_MAX) {
        a0 = ANGLE_OFFSET_MAX;
    }
    *gAngleOffset = a0;
    Serial.printf("[BLE] 机械零点 a0=%.2f\n", *gAngleOffset);
}

void persistParams() {
    if (!gKp || !gKd || !gKv || !gAngleOffset) {
        return;
    }
    prefs.begin("pid", false);
    prefs.putFloat("kp", *gKp);
    prefs.putFloat("kd", *gKd);
    prefs.putFloat("kv", *gKv);
    prefs.putFloat("a0", *gAngleOffset);
    prefs.end();
    Serial.printf("[BLE] 已持久化到 NVS kp=%.3f kd=%.3f kv=%.4f a0=%.2f\n",
                  *gKp, *gKd, *gKv, *gAngleOffset);
}

/**
 * 从 NVS 恢复已保存参数；若从未 SAVE，则回退到出厂默认值。
 * 用于开机、断连丢弃临时参数、重连后保证 GET 返回落盘值。
 */
void restorePersistedParams() {
    if (!gKp || !gKd || !gKv || !gAngleOffset) {
        return;
    }
    prefs.begin("pid", true);
    const bool hasSaved =
        prefs.isKey("kp") || prefs.isKey("kd") || prefs.isKey("kv") || prefs.isKey("a0");
    if (hasSaved) {
        *gKp = prefs.getFloat("kp", gFactoryKp);
        *gKd = prefs.getFloat("kd", gFactoryKd);
        *gKv = prefs.getFloat("kv", gFactoryKv);
        *gAngleOffset = prefs.getFloat("a0", gFactoryA0);
        if (*gAngleOffset < ANGLE_OFFSET_MIN) {
            *gAngleOffset = ANGLE_OFFSET_MIN;
        } else if (*gAngleOffset > ANGLE_OFFSET_MAX) {
            *gAngleOffset = ANGLE_OFFSET_MAX;
        }
        Serial.printf("[BLE] 从 NVS 恢复 kp=%.3f kd=%.3f kv=%.4f a0=%.2f\n",
                      *gKp, *gKd, *gKv, *gAngleOffset);
    } else {
        *gKp = gFactoryKp;
        *gKd = gFactoryKd;
        *gKv = gFactoryKv;
        *gAngleOffset = gFactoryA0;
        Serial.printf("[BLE] 无 NVS 记录，使用出厂默认 kp=%.3f kd=%.3f kv=%.4f a0=%.2f\n",
                      *gKp, *gKd, *gKv, *gAngleOffset);
    }
    prefs.end();
}

void replyStatus(const char *prefix) {
    if (!gStatusChar || !gKp || !gKd || !gKv || !gAngleOffset) {
        return;
    }
    char buf[128];
    const int ldz = gLeftDz ? *gLeftDz : 0;
    const int rdz = gRightDz ? *gRightDz : 0;
    snprintf(
        buf,
        sizeof(buf),
        "%sKP=%.3f,KD=%.3f,KV=%.4f,A0=%.2f,LDZ=%d,RDZ=%d",
        prefix ? prefix : "",
        *gKp,
        *gKd,
        *gKv,
        *gAngleOffset,
        ldz,
        rdz
    );
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

/**
 * 支持指令（大小写不敏感）:
 *   KP= / KD= / KV=
 *   SET=kp,kd,kv[,a0]
 *   A0=xx                （机械零点 / 平衡倾角偏置，°，范围 ±5）
 *   SPD=xx               （目标速度，前正后负）
 *   TRN=xx               （转向差速 turnPwm，右正左负）
 *   SLW=xx               （急刹强度 / 目标速度斜坡，值越大越剧烈）
 *   GCAL                 （陀螺仪零偏校准，需保持静止）
 *   DZCAL                （电机死区检测，轮子需悬空）
 *   GET / SAVE
 */
void handleCommand(const std::string &raw) {
    char line[128];
    size_t n = raw.size() < sizeof(line) - 1 ? raw.size() : sizeof(line) - 1;
    memcpy(line, raw.data(), n);
    line[n] = '\0';

    for (size_t i = 0; i < n; ++i) {
        if (line[i] >= 'a' && line[i] <= 'z') {
            line[i] = static_cast<char>(line[i] - 'a' + 'A');
        }
        if (line[i] == '\r' || line[i] == '\n') {
            line[i] = '\0';
        }
    }

    if (strncmp(line, "GET", 3) == 0) {
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "SAVE", 4) == 0) {
        persistParams();
        replyStatus("SAVED ");
        return;
    }

    if (strncmp(line, "GCAL", 4) == 0) {
        clearMotionCmd();
        gGyroCalibRequested = true;
        Serial.println("[BLE] 收到陀螺仪校准请求");
        if (gStatusChar) {
            gStatusChar->setValue("CAL=0");
            gStatusChar->notify();
        }
        return;
    }

    if (strncmp(line, "DZCAL", 5) == 0) {
        clearMotionCmd();
        gDeadZoneRequested = true;
        Serial.println("[BLE] 收到电机死区检测请求");
        if (gStatusChar) {
            gStatusChar->setValue("DZ=0");
            gStatusChar->notify();
        }
        return;
    }

    if (strncmp(line, "SET=", 4) == 0) {
        float kp = 0, kd = 0, kv = 0, a0 = 0;
        const int matched = sscanf(line + 4, "%f,%f,%f,%f", &kp, &kd, &kv, &a0);
        if (matched >= 4) {
            applyParams(kp, kd, kv);
            applyAngleOffset(a0);
            replyStatus("OK ");
        } else if (matched >= 3) {
            applyParams(kp, kd, kv);
            replyStatus("OK ");
        } else if (matched == 2 && gKv) {
            applyParams(kp, kd, *gKv);
            replyStatus("OK ");
        }
        return;
    }

    if (strncmp(line, "KP=", 3) == 0 && gKp && gKd && gKv) {
        applyParams(static_cast<float>(atof(line + 3)), *gKd, *gKv);
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "KD=", 3) == 0 && gKp && gKd && gKv) {
        applyParams(*gKp, static_cast<float>(atof(line + 3)), *gKv);
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "KV=", 3) == 0 && gKp && gKd && gKv) {
        applyParams(*gKp, *gKd, static_cast<float>(atof(line + 3)));
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "A0=", 3) == 0) {
        applyAngleOffset(static_cast<float>(atof(line + 3)));
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "SPD=", 4) == 0 && gTargetSpeed) {
        *gTargetSpeed = static_cast<float>(atof(line + 4));
        Serial.printf("[BLE] 目标速度 spd=%.3f\n", *gTargetSpeed);
        return;
    }

    if (strncmp(line, "TRN=", 4) == 0 && gTurnPwm) {
        *gTurnPwm = static_cast<float>(atof(line + 4));
        Serial.printf("[BLE] 转向差速 trn=%.3f\n", *gTurnPwm);
        return;
    }

    if (strncmp(line, "SLW=", 4) == 0 && gSpeedSlew) {
        float v = static_cast<float>(atof(line + 4));
        if (v < 1.0f) {
            v = 1.0f;
        }
        *gSpeedSlew = v;
        Serial.printf("[BLE] 急刹强度 slw=%.3f\n", *gSpeedSlew);
        return;
    }

    Serial.printf("[BLE] 未知指令: %s\n", line);
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *server) override {
        (void)server;
        gConnected = true;
        // 重连时确保 RAM 是 NVS/出厂值，避免上次临时调参残留
        restorePersistedParams();
        Serial.println("[BLE] 已连接");
    }

    void onDisconnect(BLEServer *server) override {
        (void)server;
        gConnected = false;
        clearMotionCmd();
        // 未点「持久化」的临时参数在断连后失效
        restorePersistedParams();
        Serial.println("[BLE] 已断开，临时参数已丢弃，重新广播");
        delay(100);
        BLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string value = characteristic->getValue();
        if (value.empty()) {
            return;
        }
        handleCommand(value);
    }
};

} // namespace

void bleConfigBegin(
    float *kp,
    float *kd,
    float *kv,
    float *angleOffset,
    int *leftDz,
    int *rightDz,
    float *targetSpeed,
    float *turnPwm,
    float *speedSlew
) {
    gKp = kp;
    gKd = kd;
    gKv = kv;
    gAngleOffset = angleOffset;
    gLeftDz = leftDz;
    gRightDz = rightDz;
    gTargetSpeed = targetSpeed;
    gTurnPwm = turnPwm;
    gSpeedSlew = speedSlew;
    // 记录代码里的出厂默认，供无 NVS 或断连回退使用
    gFactoryKp = *gKp;
    gFactoryKd = *gKd;
    gFactoryKv = *gKv;
    gFactoryA0 = *gAngleOffset;
    clearMotionCmd();
    restorePersistedParams();

    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(128);
    gServer = BLEDevice::createServer();
    gServer->setCallbacks(new ServerCallbacks());

    BLEService *service = gServer->createService(BLE_SERVICE_UUID);

    gCmdChar = service->createCharacteristic(
        BLE_CHAR_COMMAND_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    gCmdChar->setCallbacks(new CommandCallbacks());

    gStatusChar = service->createCharacteristic(
        BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    gStatusChar->addDescriptor(new BLE2902());

    char initBuf[128];
    snprintf(
        initBuf,
        sizeof(initBuf),
        "KP=%.3f,KD=%.3f,KV=%.4f,A0=%.2f,LDZ=%d,RDZ=%d",
        *gKp,
        *gKd,
        *gKv,
        *gAngleOffset,
        gLeftDz ? *gLeftDz : 0,
        gRightDz ? *gRightDz : 0
    );
    gStatusChar->setValue(initBuf);

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf(
        "[BLE] 广播中，设备名=%s kp=%.3f kd=%.3f kv=%.4f a0=%.2f ldz=%d rdz=%d\n",
        BLE_DEVICE_NAME,
        *gKp,
        *gKd,
        *gKv,
        *gAngleOffset,
        gLeftDz ? *gLeftDz : 0,
        gRightDz ? *gRightDz : 0
    );
}

bool bleConfigIsConnected() {
    return gConnected;
}

void bleConfigNotifyStatus(
    float angle,
    float batteryPercent,
    float pwm,
    float leftSpeed,
    float rightSpeed
) {
    if (!gConnected || !gStatusChar) {
        return;
    }
    char buf[96];
    snprintf(
        buf,
        sizeof(buf),
        "ANG=%.2f,BAT=%.1f,PWM=%.1f,LSP=%.1f,RSP=%.1f",
        angle,
        batteryPercent,
        pwm,
        leftSpeed,
        rightSpeed
    );
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

void bleConfigSaveParams() {
    persistParams();
}

bool bleConfigConsumeGyroCalibRequest() {
    if (!gGyroCalibRequested) {
        return false;
    }
    gGyroCalibRequested = false;
    return true;
}

void bleConfigNotifyCalibDone(float gyroXoffset, float gyroYoffset, float gyroZoffset) {
    if (!gConnected || !gStatusChar) {
        return;
    }
    char buf[80];
    snprintf(
        buf,
        sizeof(buf),
        "CAL=1,GX=%.3f,GY=%.3f,GZ=%.3f",
        gyroXoffset,
        gyroYoffset,
        gyroZoffset
    );
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

bool bleConfigConsumeDeadZoneRequest() {
    if (!gDeadZoneRequested) {
        return false;
    }
    gDeadZoneRequested = false;
    return true;
}

void bleConfigNotifyDeadZoneDone(int leftDz, int rightDz) {
    if (!gConnected || !gStatusChar) {
        return;
    }
    if (gLeftDz) {
        *gLeftDz = leftDz;
    }
    if (gRightDz) {
        *gRightDz = rightDz;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "DZ=1,LDZ=%d,RDZ=%d", leftDz, rightDz);
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}
