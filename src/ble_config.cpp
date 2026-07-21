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
float *gMaxLean = nullptr;
bool gConnected = false;

BLEServer *gServer = nullptr;
BLECharacteristic *gCmdChar = nullptr;
BLECharacteristic *gStatusChar = nullptr;

void applyParams(float kp, float kd, float kv, float maxLean) {
    if (!gKp || !gKd || !gKv || !gMaxLean) {
        return;
    }
    *gKp = kp;
    *gKd = kd;
    *gKv = kv;
    *gMaxLean = maxLean;
    Serial.printf("[BLE] 参数已更新 kp=%.3f kd=%.3f kv=%.4f ml=%.2f\n", kp, kd, kv, maxLean);
}

void persistParams() {
    if (!gKp || !gKd || !gKv || !gMaxLean) {
        return;
    }
    prefs.begin("pid", false);
    prefs.putFloat("kp", *gKp);
    prefs.putFloat("kd", *gKd);
    prefs.putFloat("kv", *gKv);
    prefs.putFloat("ml", *gMaxLean);
    prefs.end();
}

void loadParamsFromNvs() {
    if (!gKp || !gKd || !gKv || !gMaxLean) {
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
    if (prefs.isKey("ml")) {
        *gMaxLean = prefs.getFloat("ml", *gMaxLean);
    }
    prefs.end();
    Serial.printf("[BLE] NVS 加载 kp=%.3f kd=%.3f kv=%.4f ml=%.2f\n",
                  *gKp, *gKd, *gKv, *gMaxLean);
}

void replyStatus(const char *prefix) {
    if (!gStatusChar || !gKp || !gKd || !gKv || !gMaxLean) {
        return;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "%sKP=%.3f,KD=%.3f,KV=%.4f,ML=%.2f",
             prefix ? prefix : "", *gKp, *gKd, *gKv, *gMaxLean);
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

/**
 * 支持指令（大小写不敏感）:
 *   KP= / KD= / KV= / ML=
 *   SET=kp,kd            （兼容旧协议）
 *   SET=kp,kd,kv,ml
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
        float kp = 0, kd = 0, kv = 0, ml = 0;
        const int matched = sscanf(line + 4, "%f,%f,%f,%f", &kp, &kd, &kv, &ml);
        if (matched == 4) {
            applyParams(kp, kd, kv, ml);
            replyStatus("OK ");
        } else if (matched == 2 && gKv && gMaxLean) {
            applyParams(kp, kd, *gKv, *gMaxLean);
            replyStatus("OK ");
        }
        return;
    }

    if (strncmp(line, "KP=", 3) == 0 && gKp && gKd && gKv && gMaxLean) {
        applyParams(static_cast<float>(atof(line + 3)), *gKd, *gKv, *gMaxLean);
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "KD=", 3) == 0 && gKp && gKd && gKv && gMaxLean) {
        applyParams(*gKp, static_cast<float>(atof(line + 3)), *gKv, *gMaxLean);
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "KV=", 3) == 0 && gKp && gKd && gKv && gMaxLean) {
        applyParams(*gKp, *gKd, static_cast<float>(atof(line + 3)), *gMaxLean);
        replyStatus("OK ");
        return;
    }

    if (strncmp(line, "ML=", 3) == 0 && gKp && gKd && gKv && gMaxLean) {
        applyParams(*gKp, *gKd, *gKv, static_cast<float>(atof(line + 3)));
        replyStatus("OK ");
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

void bleConfigBegin(float *kp, float *kd, float *kv, float *maxLean) {
    gKp = kp;
    gKd = kd;
    gKv = kv;
    gMaxLean = maxLean;
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

    char initBuf[96];
    snprintf(initBuf, sizeof(initBuf), "KP=%.3f,KD=%.3f,KV=%.4f,ML=%.2f",
             *gKp, *gKd, *gKv, *gMaxLean);
    gStatusChar->setValue(initBuf);

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf("[BLE] 广播中，设备名=%s kp=%.3f kd=%.3f kv=%.4f ml=%.2f\n",
                  BLE_DEVICE_NAME, *gKp, *gKd, *gKv, *gMaxLean);
}

bool bleConfigIsConnected() {
    return gConnected;
}

void bleConfigNotifyStatus(float angle, float targetAngle, float pwm) {
    if (!gConnected || !gStatusChar) {
        return;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "ANG=%.2f,TA=%.2f,PWM=%.1f", angle, targetAngle, pwm);
    gStatusChar->setValue(buf);
    gStatusChar->notify();
}

void bleConfigSaveParams() {
    persistParams();
}
