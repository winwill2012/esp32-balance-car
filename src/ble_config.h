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
 * 初始化 BLE，绑定内环 kp/kd、外环 kv、机械零点 angleOffset、
 * 电机死区 leftDz/rightDz，以及遥控 targetSpeed / turnPwm / speedSlew。
 * 会从 NVS 恢复上次「持久化到芯片」的参数；若从未保存则使用出厂默认。
 * 未持久化的临时调参在 BLE 断连后失效，重连后再次从 NVS/出厂值恢复。
 * 电机死区由检测流程自动写入 NVS，断连不回滚。
 */
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
);

/** 是否有小程序已连接 */
bool bleConfigIsConnected();

/**
 * 向已连接客户端推送运行遥测（建议 50~200ms 调用一次）。
 * 格式: ANG=xx.xx,BAT=xx.x,PWM=xx.xx,LSP=xx.x,RSP=xx.x
 * BAT 为电池电量百分比 0~100。
 * LSP/RSP 为左右轮编码器速度（脉冲/采样周期）。
 * 参数仅在 GET/SET/SAVE 等指令应答中返回。
 */
void bleConfigNotifyStatus(
    float angle,
    float batteryPercent,
    float pwm,
    float leftSpeed,
    float rightSpeed
);

/** 将当前 PID/零点参数持久化到 NVS */
void bleConfigSaveParams();

/** 小程序下发 GCAL 后置位；PID 任务消费并执行校准 */
bool bleConfigConsumeGyroCalibRequest();

/** 校准完成通知：CAL=1,GX=..,GY=..,GZ=.. */
void bleConfigNotifyCalibDone(float gyroXoffset, float gyroYoffset, float gyroZoffset);

/** 小程序下发 DZCAL 后置位；PID 任务消费并执行死区检测 */
bool bleConfigConsumeDeadZoneRequest();

/** 死区检测完成通知：DZ=1,LDZ=..,RDZ=.. */
void bleConfigNotifyDeadZoneDone(int leftDz, int rightDz);

#endif // ESP32_CAR_BLE_CONFIG_H
