#include <Arduino.h>
#include "BalanceController.h"

BalanceController controller;

void setup() {
    Serial.begin(115200);
    controller.begin();
}

void loop() {
    delay(1000);
}
