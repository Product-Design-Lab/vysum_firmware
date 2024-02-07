#pragma once

#include <Arduino.h>

namespace PDL_Async_Button
{
    typedef enum {
      IDLE,
      SHORT_PRESSED,
      LONG_PRESSED,
    } ButtonSignal_e;

    void setPin(uint8_t pin);
    void setDebounceTimerValue(uint32_t ms);
    void setLongPressTimerValue(uint32_t ms);
    uint8_t getState();
    void init();

}