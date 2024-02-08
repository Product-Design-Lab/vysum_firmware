#include "PDL_Async_Button.h"
#include "Adafruit_TinyUSB.h"

void setup()
{
    Serial.begin(115200);
    PDL_Async_Button::setPin(D0);
    PDL_Async_Button::setDebounceTime(5);
    PDL_Async_Button::setLongPressTime(1000);
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