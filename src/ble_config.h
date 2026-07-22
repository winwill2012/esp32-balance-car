#ifndef ESP32_CAR_BLE_CONFIG_H
#define ESP32_CAR_BLE_CONFIG_H

#include <Arduino.h>

// 设备广播名，小程序按此扫描过滤
#define BLE_DEVICE_NAME "ESP32-Car"

// 自定义 GATT：写指令 / 读状态
#define BLE_SERVICE_UUID        "0000fff0-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_COMMAND_UUID   "0000fff1-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_STATUS_UUID    "0000fff2-0000-1000-8000-00805f9b34fb"

/**
 * 初始化 BLE，绑定内环 kp/kd、外环 kv，以及遥控 targetSpeed / turnPwm / speedSlew。
 * 会从 NVS 恢复上次保存的 PID 参数（若存在）。
 */
void bleConfigBegin(float *kp, float *kd, float *kv, float *targetSpeed, float *turnPwm, float *speedSlew);

/** 是否有小程序已连接 */
bool bleConfigIsConnected();

/**
 * 向已连接客户端推送运行遥测（建议 50~200ms 调用一次）。
 * 格式: ANG=xx.xx,BAT=xx.x,PWM=xx.xx
 * BAT 为电池电量百分比 0~100。
 * 参数仅在 GET/SET/SAVE 等指令应答中返回。
 */
void bleConfigNotifyStatus(float angle, float batteryPercent, float pwm);

/** 将当前参数持久化到 NVS */
void bleConfigSaveParams();

#endif // ESP32_CAR_BLE_CONFIG_H
