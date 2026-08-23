#include <Arduino.h>

#include "BalanceController.h"

BalanceController controller;

void setup() {
    Serial.begin(115200);
    controller.begin();
}

void loop() {
    // 实时控制和状态灯均运行在 BalanceController 创建的 FreeRTOS 任务中。
    // Arduino loop 不参与控制，保留低频休眠即可。
    delay(1000);
}
