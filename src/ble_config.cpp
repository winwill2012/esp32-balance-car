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
float *gTargetSpeed = nullptr;
float *gTurnPwm = nullptr;
float *gSpeedSlew = nullptr;
bool gConnected = false;

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

void persistParams() {
    if (!gKp || !gKd || !gKv) {
        return;
    }
    prefs.begin("pid", false);
    prefs.putFloat("kp", *gKp);
    prefs.putFloat("kd", *gKd);
    prefs.putFloat("kv", *gKv);
    prefs.end();
}

void loadParamsFromNvs() {
    if (!gKp || !gKd || !gKv) {
        return;
    }
    prefs.begin("pid", true);
    if (prefs.isKey("kp")) {
        *gKp = prefs.getFloat("kp", *gKp);
    }
    if (prefs.isKey("kd")) {
        *gKd = prefs.getFloat("kd", *gKd);
    }
    if (prefs.isKey("kv")) {
        *gKv = prefs.getFloat("kv", *gKv);
    }
    prefs.end();
    Serial.printf("[BLE] NVS 加载 kp=%.3f kd=%.3f kv=%.4f\n", *gKp, *gKd, *gKv);
}

void replyStatus(const char *prefix) {
    if (!gStatusChar || !gKp || !gKd || !gKv) {
        return;
    }
    char buf[96];
    snprintf(buf, sizeof(buf), "%sKP=%.3f,KD=%.3f,KV=%.4f",
             prefix ? prefix : "", *gKp, *gKd, *gKv);
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

/**
 * 支持指令（大小写不敏感）:
 *   KP= / KD= / KV=
 *   SET=kp,kd            （兼容旧协议）
 *   SET=kp,kd,kv
 *   SPD=xx               （目标速度，前正后负）
 *   TRN=xx               （转向差速 turnPwm，右正左负）
 *   SLW=xx               （急刹强度 / 目标速度斜坡，值越大越剧烈）
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

    if (strncmp(line, "SET=", 4) == 0) {
        float kp = 0, kd = 0, kv = 0, ignored = 0;
        const int matched = sscanf(line + 4, "%f,%f,%f,%f", &kp, &kd, &kv, &ignored);
        if (matched >= 3) {
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
        Serial.println("[BLE] 已连接");
    }

    void onDisconnect(BLEServer *server) override {
        (void)server;
        gConnected = false;
        clearMotionCmd();
        Serial.println("[BLE] 已断开，重新广播");
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

void bleConfigBegin(float *kp, float *kd, float *kv, float *targetSpeed, float *turnPwm, float *speedSlew) {
    gKp = kp;
    gKd = kd;
    gKv = kv;
    gTargetSpeed = targetSpeed;
    gTurnPwm = turnPwm;
    gSpeedSlew = speedSlew;
    clearMotionCmd();
    loadParamsFromNvs();

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

    char initBuf[64];
    snprintf(initBuf, sizeof(initBuf), "KP=%.3f,KD=%.3f,KV=%.4f", *gKp, *gKd, *gKv);
    gStatusChar->setValue(initBuf);

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] 广播中，设备名=%s kp=%.3f kd=%.3f kv=%.4f\n",
                  BLE_DEVICE_NAME, *gKp, *gKd, *gKv);
}

bool bleConfigIsConnected() {
    return gConnected;
}

void bleConfigNotifyStatus(float angle, float batteryPercent, float pwm) {
    if (!gConnected || !gStatusChar) {
        return;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "ANG=%.2f,BAT=%.1f,PWM=%.1f", angle, batteryPercent, pwm);
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

void bleConfigSaveParams() {
    persistParams();
}
