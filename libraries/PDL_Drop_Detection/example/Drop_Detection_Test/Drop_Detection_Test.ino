#include <Arduino.h>
#include "DropDetection.h"

WaterDropSensor dropSensor;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect
    }

    dropSensor.init();
}

int previousCount = 0;

void loop()
{
    if (Serial.available())
    {
        char c = Serial.peek();
        if (c == 'p')
        {
            dropSensor.pause();
        }
        else if (c == 'r')
        {
            dropSensor.resume();
        }
        else
        {
            int num = Serial.parseInt();
            dropSensor.setDebug(static_cast<uint8_t>(num));
        }
    }

    int count = dropSensor.getDropCount();
    if (count != previousCount)
    {
        Serial.printf("dropCount=%d\n", count);
        previousCount = count;
    }

    delay(1000);
}
