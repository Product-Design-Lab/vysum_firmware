#include "PDL_Async_Button.h"

void setup()
{
    PDL_Async_Button::setPin(2);
    PDL_Async_Button::setDebounceTimerValue(5);
    PDL_Async_Button::setLongPressTimerValue(1000);
    PDL_Async_Button::init();
}

void loop()
{
    uint8_t state = PDL_Async_Button::getState();
    if (state == PDL_Async_Button::SHORT_PRESS)
    {
        Serial.println("Short Press");
    }
    else if (state == PDL_Async_Button::LONG_PRESS)
    {
        Serial.println("Long Press");
    }
    delay(10);
}