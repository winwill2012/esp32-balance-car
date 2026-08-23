#ifndef ESP32_BALANCE_CAR_BATTERY_MONITOR_H
#define ESP32_BALANCE_CAR_BATTERY_MONITOR_H

class BatteryMonitor {
public:
    void begin() const;
    float readPercent() const;
};

#endif
