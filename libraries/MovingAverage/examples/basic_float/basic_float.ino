#include "MovingAverage.h"
#include "Adafruit_TinyUSB.h"//seeed xiao board requires this line


// Buffer (and added samples) will be initialised as uint8_t, total 16 samples
MovingAverage<float, 16> filter;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        delay(10);
    }; // wait for serial monitor
    Serial.print("result: ");
    Serial.println(filter.add(255.2)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.add(6.3)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.add(9.9)); // insert new number and get result
    Serial.print("result: ");
    Serial.println(filter.get()); // get last result, without adding a newone
}

void loop()
{
}
