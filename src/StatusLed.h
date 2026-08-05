#ifndef ESP32_BALANCE_CAR_STATUS_LED_H
#define ESP32_BALANCE_CAR_STATUS_LED_H

class StatusLed {
public:
    void begin() const;
    void setDuty(int duty) const;
};

#endif
