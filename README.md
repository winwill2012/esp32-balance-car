# ESP32 平衡小车

基于 ESP32 的两轮平衡小车，采用“速度环 → 直立环”的串级控制，
并通过微信小程序和 BLE 完成参数调整、遥控、校准与状态监控。

## 固件模块

- `BalanceController`：组织速度/直立环、校准流程和 FreeRTOS 任务。
- `BleConfigService`：处理 BLE GATT、控制指令和 PID 参数持久化。
- `EncoderPair`：封装左右编码器和中断计数。
- `MotorDriver`：封装 PWM 输出、左右轮混控和电机死区补偿。
- `BatteryMonitor`：读取并换算电池电量。
- `StatusLed`：封装连接状态指示灯。
- `ControlParameters` / `MotionCommand`：集中保存调参数据和遥控命令。
- `pins`：集中定义硬件引脚、PWM 通道和硬件常量。

程序入口位于 `src/Main.cpp`，只负责创建并启动 `BalanceController`。

## 构建

```shell
pio run
```

固件产物为 `.pio/build/esp32dev/firmware.bin`。
