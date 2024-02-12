#include <Arduino.h>
#include "dropDetection.h"

APDS_DropSensor drop_sensor;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("Drop Detection Test");
    drop_sensor.init();
}

void loop()
{
    drop_sensor.update();
    delay(10);
}
