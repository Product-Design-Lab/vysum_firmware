#pragma once

#include <Arduino.h>

namespace PDL_Async_Button
{

enum ButtonState
{
    IDLE,
    SHORT_PRESS,
    LONG_PRESS
};

void setPin(uint8_t pin);
void setDebounceTime(uint32_t ms);
void setLongPressTime(uint32_t ms);
uint8_t getState();
void init();
void setDebug(bool debug);

} // namespace PDL_Async_Button