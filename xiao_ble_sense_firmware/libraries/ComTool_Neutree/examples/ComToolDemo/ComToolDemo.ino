#include "ComTool_Neutree.h"
#include <Adafruit_TinyUSB.h> // for Serial

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial)
        ;
    Serial.println("Hello World");
}

void loop()
{
    for (int i = 0; i < 100; i++)
    {
        ComToolPlot("i", i);
        ComToolPlot("-i", -i);
        delay(100);
    }
}