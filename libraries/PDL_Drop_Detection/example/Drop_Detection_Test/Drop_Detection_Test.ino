#include <Arduino.h>
#include "DropDetection.h"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    APDS_DropSensor::init();
}

int previousCount = 0;
void loop()
{
    if (Serial.available())
    {
        char c = Serial.peek();
        if (c == 'p')
        {
            APDS_DropSensor::pause();
        }
        else if (c == 'r')
        {
            APDS_DropSensor::resume();
        }
        else
        {
            int num = Serial.parseInt();
            APDS_DropSensor::setDebug((uint8_t)num);
        }
    }

    // int count = APDS_DropSensor::get_drop_count();
    // if (count != previousCount)
    // {
    //     Serial.printf("dropCount=%d\n", count);
    //     previousCount = count;
    // }

    delay(1000);
}
