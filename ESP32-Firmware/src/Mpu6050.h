#ifndef ESP32_BALANCE_CAR_MPU6050_I2C_H
#define ESP32_BALANCE_CAR_MPU6050_I2C_H

#include <Arduino.h>
#include <Wire.h>

/**
 * 基于 ESP32 Arduino Core Wire 的 MPU6050 驱动。
 *
 * 仅实现平衡控制需要的功能：传感器配置、批量采样、陀螺仪校准和
 * X 轴互补滤波。角速度单位为 deg/s，角度单位为度。
 */
class Mpu6050 {
public:
    explicit Mpu6050(TwoWire &wire, uint8_t address = 0x68);

    bool begin();
    bool update();
    bool calibrateGyro(uint16_t sampleCount = 1000);

    void setGyroOffsets(float x, float y, float z);
    float gyroXOffset() const { return gyroXOffset_; }
    float gyroYOffset() const { return gyroYOffset_; }
    float gyroZOffset() const { return gyroZOffset_; }

    float angleX() const { return angleX_; }
    float gyroX() const { return gyroX_; }
    float gyroZ() const { return gyroZ_; }

private:
    bool writeRegister(uint8_t reg, uint8_t value) const;
    bool readRegisters(uint8_t firstReg, uint8_t *data, size_t length) const;
    bool readRawSample(
        int16_t &accX,
        int16_t &accY,
        int16_t &accZ,
        int16_t &gyroX,
        int16_t &gyroY,
        int16_t &gyroZ
    ) const;

    TwoWire &wire_;
    uint8_t address_;

    float gyroXOffset_ = 0.0f;
    float gyroYOffset_ = 0.0f;
    float gyroZOffset_ = 0.0f;
    float gyroX_ = 0.0f;
    float gyroZ_ = 0.0f;
    float angleX_ = 0.0f;
    uint32_t previousUpdateUs_ = 0;
    bool filterInitialized_ = false;
};

#endif
