#include "Mpu6050.h"
#include <cmath>

constexpr uint8_t kSampleRateDividerRegister = 0x19;
constexpr uint8_t kConfigRegister = 0x1A;
constexpr uint8_t kGyroConfigRegister = 0x1B;
constexpr uint8_t kAccelConfigRegister = 0x1C;
constexpr uint8_t kAccelDataRegister = 0x3B;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr uint8_t kWhoAmIRegister = 0x75;

// DLPF_CFG=3：加速度计带宽约 44 Hz、陀螺仪带宽约 42 Hz。
// 启用 DLPF 后内部采样基准为 1 kHz，分频值 4 得到 200 Hz 输出率。
constexpr uint8_t kDigitalLowPassFilterConfig = 0x03;
constexpr uint8_t kSampleRateDivider = 0x04;
constexpr uint32_t kSamplePeriodMs = 5;

constexpr float kAccelScale = 16384.0f; // AFS_SEL=0: 16384 LSB/g
constexpr float kGyroScale = 65.5f; // FS_SEL=1: 65.5 LSB/(deg/s)
constexpr float kAccelFilterWeight = 0.02f;
constexpr float kGyroFilterWeight = 1.0f - kAccelFilterWeight;

int16_t decodeInt16(const uint8_t *bytes) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]
    );
}

Mpu6050::Mpu6050(TwoWire &wire, const uint8_t address)
    : wire_(wire), address_(address) {
}

bool Mpu6050::begin() {
    uint8_t identity = 0;
    if (!readRegisters(kWhoAmIRegister, &identity, 1) ||
        (identity & 0x7E) != 0x68) {
        return false;
    }

    // 选择 X 轴陀螺仪作为时钟源并退出睡眠，然后使用与原控制参数一致的量程。
    if (!writeRegister(kPowerManagementRegister, 0x01)) {
        return false;
    }
    delay(10);
    if (!writeRegister(kConfigRegister, kDigitalLowPassFilterConfig) ||
        !writeRegister(kSampleRateDividerRegister, kSampleRateDivider) ||
        !writeRegister(kGyroConfigRegister, 0x08) ||
        !writeRegister(kAccelConfigRegister, 0x00)) {
        return false;
    }

    filterInitialized_ = false;
    previousUpdateUs_ = micros();
    return update();
}

bool Mpu6050::update() {
    int16_t rawAccX = 0;
    int16_t rawAccY = 0;
    int16_t rawAccZ = 0;
    int16_t rawGyroX = 0;
    int16_t rawGyroY = 0;
    int16_t rawGyroZ = 0;
    if (!readRawSample(
        rawAccX,
        rawAccY,
        rawAccZ,
        rawGyroX,
        rawGyroY,
        rawGyroZ)) {
        return false;
    }

    const float accX = rawAccX / kAccelScale;
    const float accY = rawAccY / kAccelScale;
    const float accZ = rawAccZ / kAccelScale;
    const float accelAngleX = atan2f(accY, accZ + fabsf(accX)) *
                              180.0f / PI;

    gyroX_ = rawGyroX / kGyroScale - gyroXOffset_;
    gyroZ_ = rawGyroZ / kGyroScale - gyroZOffset_;
    const uint32_t nowUs = micros();
    const float dt = (nowUs - previousUpdateUs_) * 0.000001f;
    previousUpdateUs_ = nowUs;

    if (!filterInitialized_) {
        angleX_ = accelAngleX;
        filterInitialized_ = true;
    } else {
        angleX_ = kGyroFilterWeight * (angleX_ + gyroX_ * dt) +
                  kAccelFilterWeight * accelAngleX;
    }
    return true;
}

bool Mpu6050::calibrateGyro(const uint16_t sampleCount) {
    if (sampleCount == 0) {
        return false;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    for (uint16_t sample = 0; sample < sampleCount; ++sample) {
        int16_t accX = 0;
        int16_t accY = 0;
        int16_t accZ = 0;
        int16_t gyroX = 0;
        int16_t gyroY = 0;
        int16_t gyroZ = 0;
        if (!readRawSample(accX, accY, accZ, gyroX, gyroY, gyroZ)) {
            return false;
        }
        sumX += gyroX / kGyroScale;
        sumY += gyroY / kGyroScale;
        sumZ += gyroZ / kGyroScale;

        // 与 200 Hz 输出周期对齐，确保每次累计的是一份新数据。
        delay(kSamplePeriodMs);
    }

    gyroXOffset_ = static_cast<float>(sumX / sampleCount);
    gyroYOffset_ = static_cast<float>(sumY / sampleCount);
    gyroZOffset_ = static_cast<float>(sumZ / sampleCount);
    filterInitialized_ = false;
    previousUpdateUs_ = micros();
    return true;
}

void Mpu6050::setGyroOffsets(
    const float x,
    const float y,
    const float z
) {
    gyroXOffset_ = x;
    gyroYOffset_ = y;
    gyroZOffset_ = z;
}

bool Mpu6050::writeRegister(const uint8_t reg, const uint8_t value) const {
    wire_.beginTransmission(address_);
    wire_.write(reg);
    wire_.write(value);
    return wire_.endTransmission(true) == 0;
}

bool Mpu6050::readRegisters(const uint8_t firstReg, uint8_t *data, const size_t length) const {
    wire_.beginTransmission(address_);
    wire_.write(firstReg);
    if (wire_.endTransmission(false) != 0) {
        return false;
    }

    const size_t received = wire_.requestFrom(
        address_,
        length,
        true
    );
    if (received != length || wire_.available() < static_cast<int>(length)) {
        while (wire_.available()) {
            wire_.read();
        }
        return false;
    }
    for (size_t index = 0; index < length; ++index) {
        data[index] = static_cast<uint8_t>(wire_.read());
    }
    return true;
}

bool Mpu6050::readRawSample(
    int16_t &accX,
    int16_t &accY,
    int16_t &accZ,
    int16_t &gyroX,
    int16_t &gyroY,
    int16_t &gyroZ
) const {
    uint8_t data[14] = {};
    if (!readRegisters(kAccelDataRegister, data, sizeof(data))) {
        return false;
    }
    accX = decodeInt16(&data[0]);
    accY = decodeInt16(&data[2]);
    accZ = decodeInt16(&data[4]);
    // data[6..7] 是温度，本项目不使用。
    gyroX = decodeInt16(&data[8]);
    gyroY = decodeInt16(&data[10]);
    gyroZ = decodeInt16(&data[12]);
    return true;
}
