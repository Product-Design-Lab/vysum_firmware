#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial


#include "diagnostics.h"
#include "pose_checker.hpp"

void setup()
{
    DIAG_init();
    IMU::init(0.5f);
}

// main task
void loop()
{
    if (IMU::isVertical())
    {
        Serial.println("Vertical");
    }
    else
    {
        Serial.println("Not vertical");
    }
    delay(1000);
}
