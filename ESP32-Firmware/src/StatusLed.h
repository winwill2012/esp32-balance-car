#ifndef ESP32_BALANCE_CAR_STATUS_LED_H
#define ESP32_BALANCE_CAR_STATUS_LED_H

class StatusLed {
public:
    static void begin();

    static void setDuty(int duty);
};

#endif
