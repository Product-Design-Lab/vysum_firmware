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

    int count = APDS_DropSensor::get_drop_count();
    if(count != previousCount)
    {
        Serial.printf("dropCount=%d\n", count);
        previousCount = count;
    }
    
    delay(200);
}
