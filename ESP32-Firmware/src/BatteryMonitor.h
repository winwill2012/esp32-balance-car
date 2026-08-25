#ifndef ESP32_BALANCE_CAR_BATTERY_MONITOR_H
#define ESP32_BALANCE_CAR_BATTERY_MONITOR_H

class BatteryMonitor {
public:
    static void begin();

    static float readPercent();
};

#endif
