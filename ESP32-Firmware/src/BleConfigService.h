#ifndef ESP32_BALANCE_CAR_BLE_CONFIG_H
#define ESP32_BALANCE_CAR_BLE_CONFIG_H

#include <BLEServer.h>
#include <string>

#include "ControlTypes.h"

/**
 * BLE 配置与遥测服务。
 *
 * 写特征接收调参、遥控和校准命令；读/通知特征返回参数与运行状态。
 * 类只保存 ControlParameters 和 MotionCommand 的非拥有指针，因此传入对象的
 * 生命周期必须长于本服务（当前二者均由 BalanceController 持有）。
 */
class BleConfigService {
public:
    BleConfigService();

    void begin(ControlParameters &parameters, MotionCommand &motionCommand);
    bool isConnected() const;

    void notifyStatus(
        float angle,
        float batteryPercent,
        int pwm,
        float leftSpeed,
        float rightSpeed
    ) const;
    void notifyCalibrationDone(
        float gyroXOffset,
        float gyroYOffset,
        float gyroZOffset
    );
    bool consumeGyroCalibrationRequest();
    void saveParameters();

private:
    class ServerCallbacks final : public BLEServerCallbacks {
    public:
        explicit ServerCallbacks(BleConfigService &owner);
        void onConnect(BLEServer *server) override;
        void onDisconnect(BLEServer *server) override;

    private:
        BleConfigService &owner_;
    };

    class CommandCallbacks final : public BLECharacteristicCallbacks {
    public:
        explicit CommandCallbacks(BleConfigService &owner);
        void onWrite(BLECharacteristic *characteristic) override;

    private:
        BleConfigService &owner_;
    };

    void clearMotionCommand();
    void applyBalanceParameters(float angleKp, float angleKd, float speedKp);
    void applyPidParameters(
        float angleKp,
        float angleKi,
        float angleKd,
        float speedKp,
        float speedKi,
        float speedKd,
        float turnKp,
        float turnKi,
        float turnKd
    );
    void applyAngleOffset(float angleOffset);
    void persistParameters();
    void restorePersistedParameters();
    void replyParameters(const char *prefix);
    void handleCommand(const std::string &rawCommand);

    ControlParameters *parameters_ = nullptr;
    MotionCommand *motionCommand_ = nullptr;
    ControlParameters factoryParameters_;

    BLEServer *server_ = nullptr;
    BLECharacteristic *commandCharacteristic_ = nullptr;
    BLECharacteristic *statusCharacteristic_ = nullptr;

    bool connected_ = false;
    // BLE 回调与控制任务之间只传递一次性请求；耗时操作不在 BLE 回调中执行。
    volatile bool gyroCalibrationRequested_ = false;

    ServerCallbacks serverCallbacks_;
    CommandCallbacks commandCallbacks_;
};

#endif
