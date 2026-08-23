#ifndef ESP32_BALANCE_CAR_ENCODER_PAIR_H
#define ESP32_BALANCE_CAR_ENCODER_PAIR_H

#include <Arduino.h>

class EncoderPair {
public:
    void begin();
    void readAndReset(int32_t &leftCount, int32_t &rightCount);

private:
    static EncoderPair *instance_;

    static void IRAM_ATTR handleLeftInterrupt();
    static void IRAM_ATTR handleRightInterrupt();

    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile int32_t leftCount_ = 0;
    volatile int32_t rightCount_ = 0;
};

#endif // ESP32_BALANCE_CAR_ENCODER_PAIR_H
