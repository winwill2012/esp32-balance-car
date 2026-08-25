# ESP32 平衡小车

基于 ESP32 的两轮平衡小车，采用“速度环 → 直立环”的串级控制，
并通过微信小程序和 BLE 完成参数调整、遥控、校准与状态监控。

## 固件模块

- `BalanceController`：组织速度、直立和转向闭环、校准流程及 FreeRTOS 任务。
- `Mpu6050`：使用 ESP32 Arduino Core 的 `Wire` 接口直接读写 MPU6050，
  并完成零偏校准与姿态角互补滤波。
- `BleConfigService`：处理 BLE GATT、控制指令和 PID 参数持久化。
- `PidController`：封装 P/PI/PD/PID 运算、积分与输出限幅，并支持外部微分输入。
- `EncoderPair`：封装左右编码器和中断计数。
- `MotorDriver`：封装 PWM 输出、左右轮混控和固定小幅渐进摩擦补偿。
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

## 转向闭环

蓝牙 `TRN` 指令直接表示目标偏航角速度（°/s），不再进行比例换算，
由 `PidController` 根据 MPU6050 Z 轴角速度计算左右轮差动 PWM。
小程序每 150 ms 发送一次遥控心跳；固件超过 450 ms 未收到转向指令时，
自动将目标偏航角速度置零，以抑制原地自转并保持直线。

## PID 参数

倾角、速度和转向三个控制环均支持独立的 `Kp`、`Ki`、`Kd`，可在微信小程序
实时调整并持久化到 NVS。任一增益设置为 0 即关闭对应的 P、I 或 D 项。
PID 积分按 `error * dt` 累计，普通微分按误差变化率计算；倾角环的 D 项继续使用
MPU6050 X 轴角速度，避免对姿态角做差分放大噪声。
