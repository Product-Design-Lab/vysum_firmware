#include <Arduino.h>
#include "dropDetection.h"


void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    APDS_DropSensor::init();
}

void loop()
{
    delay(10);
}
