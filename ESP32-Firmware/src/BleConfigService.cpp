#include "BleConfigService.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <Preferences.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

constexpr char kDeviceName[] = "ESP32-Car";
constexpr char kServiceUuid[] = "0000fff0-0000-1000-8000-00805f9b34fb";
constexpr char kCommandUuid[] = "0000fff1-0000-1000-8000-00805f9b34fb";
constexpr char kStatusUuid[] = "0000fff2-0000-1000-8000-00805f9b34fb";

constexpr float kMinAngleOffset = -5.0f;
constexpr float kMaxAngleOffset = 5.0f;
BleConfigService::ServerCallbacks::ServerCallbacks(BleConfigService &owner)
    : owner_(owner) {}

void BleConfigService::ServerCallbacks::onConnect(BLEServer *server) {
    (void)server;
    owner_.connected_ = true;
    // 每次连接都从 NVS 恢复，确保小程序读到的是已保存参数，而不是上次遗留的临时值。
    owner_.restorePersistedParameters();
    Serial.println("[BLE] 已连接");
}

void BleConfigService::ServerCallbacks::onDisconnect(BLEServer *server) {
    (void)server;
    owner_.connected_ = false;
    // 断连立即停车；未执行 SAVE 的临时调参也回滚到 NVS/出厂值。
    owner_.clearMotionCommand();
    owner_.restorePersistedParameters();
    Serial.println("[BLE] 已断开，临时参数已丢弃，重新广播");
    delay(100);
    BLEDevice::startAdvertising();
}

BleConfigService::CommandCallbacks::CommandCallbacks(BleConfigService &owner)
    : owner_(owner) {}

void BleConfigService::CommandCallbacks::onWrite(
    BLECharacteristic *characteristic
) {
    const std::string value = characteristic->getValue();
    if (!value.empty()) {
        owner_.handleCommand(value);
    }
}

BleConfigService::BleConfigService()
    : serverCallbacks_(*this), commandCallbacks_(*this) {}

void BleConfigService::begin(
    ControlParameters &parameters,
    MotionCommand &motionCommand
) {
    // 保存引用关系和代码默认参数。factoryParameters_ 用于没有 NVS 记录时回退。
    parameters_ = &parameters;
    motionCommand_ = &motionCommand;
    factoryParameters_ = parameters;

    clearMotionCommand();
    restorePersistedParameters();

    BLEDevice::init(kDeviceName);
    BLEDevice::setMTU(128);
    server_ = BLEDevice::createServer();
    server_->setCallbacks(&serverCallbacks_);

    BLEService *service = server_->createService(kServiceUuid);
    // Command：小程序写入；Status：小程序读取或订阅通知。两者职责保持单一。
    commandCharacteristic_ = service->createCharacteristic(
        kCommandUuid,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR
    );
    commandCharacteristic_->setCallbacks(&commandCallbacks_);

    statusCharacteristic_ = service->createCharacteristic(
        kStatusUuid,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
    );
    statusCharacteristic_->addDescriptor(new BLE2902());

    char initialValue[128];
    snprintf(
        initialValue,
        sizeof(initialValue),
        "KP=%.3f,KD=%.3f,KV=%.4f,A0=%.2f,LDZ=%d,RDZ=%d",
        parameters_->angleKp,
        parameters_->angleKd,
        parameters_->speedKp,
        parameters_->angleOffset,
        parameters_->leftMotorDeadZone,
        parameters_->rightMotorDeadZone
    );
    statusCharacteristic_->setValue(initialValue);

    service->start();
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf(
        "[BLE] 广播中：%s kp=%.3f kd=%.3f kv=%.4f a0=%.2f "
        "ldz=%d rdz=%d\n",
        kDeviceName,
        parameters_->angleKp,
        parameters_->angleKd,
        parameters_->speedKp,
        parameters_->angleOffset,
        parameters_->leftMotorDeadZone,
        parameters_->rightMotorDeadZone
    );
}

bool BleConfigService::isConnected() const {
    return connected_;
}

void BleConfigService::notifyStatus(
    const float angle,
    const float batteryPercent,
    const int pwm,
    const float leftSpeed,
    const float rightSpeed
) {
    if (!connected_ || statusCharacteristic_ == nullptr) {
        return;
    }

    // 使用稳定的键值文本协议，便于小程序按逗号拆分，同时兼容串口人工调试。
    char status[96];
    snprintf(
        status,
        sizeof(status),
        "ANG=%.2f,BAT=%.1f,PWM=%d,LSP=%.1f,RSP=%.1f",
        angle,
        batteryPercent,
        pwm,
        leftSpeed,
        rightSpeed
    );
    statusCharacteristic_->setValue(status);
    statusCharacteristic_->notify();
}

void BleConfigService::notifyCalibrationDone(
    const float gyroXOffset,
    const float gyroYOffset,
    const float gyroZOffset
) {
    if (!connected_ || statusCharacteristic_ == nullptr) {
        return;
    }

    char status[80];
    snprintf(
        status,
        sizeof(status),
        "CAL=1,GX=%.3f,GY=%.3f,GZ=%.3f",
        gyroXOffset,
        gyroYOffset,
        gyroZOffset
    );
    statusCharacteristic_->setValue(status);
    statusCharacteristic_->notify();
}

void BleConfigService::notifyDeadZoneDone(
    const int leftDeadZone,
    const int rightDeadZone
) {
    if (parameters_ != nullptr) {
        parameters_->leftMotorDeadZone = leftDeadZone;
        parameters_->rightMotorDeadZone = rightDeadZone;
    }
    if (!connected_ || statusCharacteristic_ == nullptr) {
        return;
    }

    char status[64];
    snprintf(
        status,
        sizeof(status),
        "DZ=1,LDZ=%d,RDZ=%d",
        leftDeadZone,
        rightDeadZone
    );
    statusCharacteristic_->setValue(status);
    statusCharacteristic_->notify();
}

bool BleConfigService::consumeGyroCalibrationRequest() {
    if (!gyroCalibrationRequested_) {
        return false;
    }
    // 消费即清零，保证一条 GCAL 命令只执行一次校准。
    gyroCalibrationRequested_ = false;
    return true;
}

bool BleConfigService::consumeDeadZoneRequest() {
    if (!deadZoneRequested_) {
        return false;
    }
    deadZoneRequested_ = false;
    return true;
}

void BleConfigService::saveParameters() {
    persistParameters();
}

void BleConfigService::clearMotionCommand() {
    if (motionCommand_ != nullptr) {
        motionCommand_->clear();
    }
}

void BleConfigService::applyBalanceParameters(
    const float angleKp,
    const float angleKd,
    const float speedKp
) {
    if (parameters_ == nullptr) {
        return;
    }
    parameters_->angleKp = angleKp;
    parameters_->angleKd = angleKd;
    parameters_->speedKp = speedKp;
    Serial.printf(
        "[BLE] 参数更新 kp=%.3f kd=%.3f kv=%.4f\n",
        angleKp,
        angleKd,
        speedKp
    );
}

void BleConfigService::applyAngleOffset(float angleOffset) {
    if (parameters_ == nullptr) {
        return;
    }
    angleOffset = constrain(angleOffset, kMinAngleOffset, kMaxAngleOffset);
    parameters_->angleOffset = angleOffset;
    Serial.printf("[BLE] 机械零点 a0=%.2f\n", angleOffset);
}

void BleConfigService::persistParameters() {
    if (parameters_ == nullptr) {
        return;
    }

    // 仅保存控制参数。电机死区由独立的 motor NVS 空间负责，避免职责交叉。
    Preferences preferences;
    preferences.begin("pid", false);
    preferences.putFloat("kp", parameters_->angleKp);
    preferences.putFloat("kd", parameters_->angleKd);
    preferences.putFloat("kv", parameters_->speedKp);
    preferences.putFloat("a0", parameters_->angleOffset);
    preferences.end();
    Serial.printf(
        "[BLE] 参数已保存 kp=%.3f kd=%.3f kv=%.4f a0=%.2f\n",
        parameters_->angleKp,
        parameters_->angleKd,
        parameters_->speedKp,
        parameters_->angleOffset
    );
}

void BleConfigService::restorePersistedParameters() {
    if (parameters_ == nullptr) {
        return;
    }

    Preferences preferences;
    preferences.begin("pid", true);
    // 兼容旧固件：只要任意历史键存在就逐项读取，缺失的新键使用代码默认值。
    const bool hasSavedParameters =
        preferences.isKey("kp") ||
        preferences.isKey("kd") ||
        preferences.isKey("kv") ||
        preferences.isKey("a0");

    if (hasSavedParameters) {
        parameters_->angleKp =
            preferences.getFloat("kp", factoryParameters_.angleKp);
        parameters_->angleKd =
            preferences.getFloat("kd", factoryParameters_.angleKd);
        parameters_->speedKp =
            preferences.getFloat("kv", factoryParameters_.speedKp);
        parameters_->angleOffset = constrain(
            preferences.getFloat("a0", factoryParameters_.angleOffset),
            kMinAngleOffset,
            kMaxAngleOffset
        );
        Serial.printf(
            "[BLE] 从 NVS 恢复 kp=%.3f kd=%.3f kv=%.4f a0=%.2f\n",
            parameters_->angleKp,
            parameters_->angleKd,
            parameters_->speedKp,
            parameters_->angleOffset
        );
    } else {
        // 恢复 PID 出厂值时保留已由电机模块加载的左右死区。
        const int leftDeadZone = parameters_->leftMotorDeadZone;
        const int rightDeadZone = parameters_->rightMotorDeadZone;
        *parameters_ = factoryParameters_;
        parameters_->leftMotorDeadZone = leftDeadZone;
        parameters_->rightMotorDeadZone = rightDeadZone;
        Serial.println("[BLE] 无 PID NVS 记录，使用代码默认参数");
    }
    preferences.end();
}

void BleConfigService::replyParameters(const char *prefix) {
    if (parameters_ == nullptr || statusCharacteristic_ == nullptr) {
        return;
    }

    char status[128];
    snprintf(
        status,
        sizeof(status),
        "%sKP=%.3f,KD=%.3f,KV=%.4f,A0=%.2f,LDZ=%d,RDZ=%d",
        prefix == nullptr ? "" : prefix,
        parameters_->angleKp,
        parameters_->angleKd,
        parameters_->speedKp,
        parameters_->angleOffset,
        parameters_->leftMotorDeadZone,
        parameters_->rightMotorDeadZone
    );
    statusCharacteristic_->setValue(status);
    statusCharacteristic_->notify();
}

void BleConfigService::handleCommand(const std::string &rawCommand) {
    // 协议命令（不区分大小写）：
    // GET / SAVE / GCAL / DZCAL
    // SET=KP,KD,KV[,A0]，并兼容旧版仅发送 KP,KD 的格式
    // KP= / KD= / KV= / A0= / SPD= / TRN= / SLW=
    char command[128];
    const size_t length =
        rawCommand.size() < sizeof(command) - 1
            ? rawCommand.size()
            : sizeof(command) - 1;
    memcpy(command, rawCommand.data(), length);
    command[length] = '\0';

    // 在固定长度缓冲区内统一转为大写，并截断换行，避免动态字符串解析产生碎片。
    for (size_t i = 0; i < length; ++i) {
        if (command[i] >= 'a' && command[i] <= 'z') {
            command[i] = static_cast<char>(command[i] - 'a' + 'A');
        }
        if (command[i] == '\r' || command[i] == '\n') {
            command[i] = '\0';
        }
    }

    if (strncmp(command, "GET", 3) == 0) {
        replyParameters("OK ");
        return;
    }
    if (strncmp(command, "SAVE", 4) == 0) {
        persistParameters();
        replyParameters("SAVED ");
        return;
    }
    if (strncmp(command, "GCAL", 4) == 0) {
        // 这里只应答并置位，真正校准由控制任务在电机停止后执行。
        clearMotionCommand();
        gyroCalibrationRequested_ = true;
        if (statusCharacteristic_ != nullptr) {
            statusCharacteristic_->setValue("CAL=0");
            statusCharacteristic_->notify();
        }
        return;
    }
    if (strncmp(command, "DZCAL", 5) == 0) {
        clearMotionCommand();
        deadZoneRequested_ = true;
        if (statusCharacteristic_ != nullptr) {
            statusCharacteristic_->setValue("DZ=0");
            statusCharacteristic_->notify();
        }
        return;
    }

    if (strncmp(command, "SET=", 4) == 0) {
        float angleKp = 0.0f;
        float angleKd = 0.0f;
        float speedKp = 0.0f;
        float angleOffset = 0.0f;
        const int matched = sscanf(
            command + 4,
            "%f,%f,%f,%f",
            &angleKp,
            &angleKd,
            &speedKp,
            &angleOffset
        );
        // matched 表示成功解析的字段数，从而兼容不同版本小程序的 SET 长度。
        if (matched >= 3) {
            applyBalanceParameters(angleKp, angleKd, speedKp);
        } else if (matched == 2 && parameters_ != nullptr) {
            applyBalanceParameters(angleKp, angleKd, parameters_->speedKp);
        } else {
            return;
        }
        if (matched >= 4) {
            applyAngleOffset(angleOffset);
        }
        replyParameters("OK ");
        return;
    }

    if (parameters_ == nullptr) {
        return;
    }
    if (strncmp(command, "KP=", 3) == 0) {
        applyBalanceParameters(
            static_cast<float>(atof(command + 3)),
            parameters_->angleKd,
            parameters_->speedKp
        );
        replyParameters("OK ");
        return;
    }
    if (strncmp(command, "KD=", 3) == 0) {
        applyBalanceParameters(
            parameters_->angleKp,
            static_cast<float>(atof(command + 3)),
            parameters_->speedKp
        );
        replyParameters("OK ");
        return;
    }
    if (strncmp(command, "KV=", 3) == 0) {
        applyBalanceParameters(
            parameters_->angleKp,
            parameters_->angleKd,
            static_cast<float>(atof(command + 3))
        );
        replyParameters("OK ");
        return;
    }
    if (strncmp(command, "A0=", 3) == 0) {
        applyAngleOffset(static_cast<float>(atof(command + 3)));
        replyParameters("OK ");
        return;
    }
    if (strncmp(command, "SPD=", 4) == 0 && motionCommand_ != nullptr) {
        motionCommand_->targetSpeed = static_cast<float>(atof(command + 4));
        return;
    }
    if (strncmp(command, "TRN=", 4) == 0 && motionCommand_ != nullptr) {
        motionCommand_->turnPwm = static_cast<float>(atof(command + 4));
        return;
    }
    if (strncmp(command, "SLW=", 4) == 0) {
        parameters_->speedSlew =
            max(1.0f, static_cast<float>(atof(command + 4)));
        return;
    }

    Serial.printf("[BLE] 未知指令: %s\n", command);
}
