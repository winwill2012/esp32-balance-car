#include <Arduino.h>
#include <Wire.h>

// MPU6050相关引脚
constexpr uint8_t mpu6050Scl = 32;
constexpr uint8_t mpu6050Sda = 33;
constexpr uint8_t mpu6050Int = 25;

//MPU6050 I2C从机地址
constexpr uint8_t mpu6050Address = 0x68;

// WHO_AM_I寄存器地址
constexpr uint8_t registerWhoAmI = 0x75;

// 往特定的寄存器地址写入特定的数值
bool writeRegister(const uint8_t reg, const uint8_t value) {
    Wire.beginTransmission(mpu6050Address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

// 从某一个起始寄存器连续读取len个数值
bool readRegisters(const uint8_t reg, uint8_t *value, const size_t len) {
    Wire.beginTransmission(mpu6050Address);
    Wire.write(reg);
    // 如果数据写入失败，返回
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    const size_t size = Wire.requestFrom(mpu6050Address, len, true);
    // 如果获取的数据数量与要求的数量不一致，说明读取数据有问题，则将所有脏数据全部读取并丢弃
    if (size != len || Wire.available() < len) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }
    for (int i = 0; i < len; i++) {
        value[i] = Wire.read();
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(mpu6050Sda, mpu6050Scl, 400000);
    if (!writeRegister(0x6B, 0x01) ||
        !writeRegister(0x1B, 0x08) ||
        !writeRegister(0x1C, 0x00) ||
        !writeRegister(0x1A, 0x03) ||
        !writeRegister(0x19, 0x04)) {
        Serial.println("初始化MPU6050失败");
        return;
    }
    uint8_t identity = 0;
    if (!readRegisters(registerWhoAmI, &identity, 1)) {
        Serial.println("从寄存器读取芯片I2C地址失败");
        return;
    }
    Serial.printf("从寄存器获取芯片I2C地址: %0x\n", (identity & 0x7E));
}

void loop() {
}
