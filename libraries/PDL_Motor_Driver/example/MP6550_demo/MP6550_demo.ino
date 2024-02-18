#include "motor_driver.h"

MotorDriver mp6550;

void setup()
{
    mp6550.setPwmPin(8, 9);
    mp6550.setDirNoPin();
    mp6550.setEnableNoPin();
    mp6550.setVisenPin(10);
    
    mp6550.setMaxPwm(255);
}

void loop()
{
    mp6550.setPwmPercent(0.5);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent());
        delay(100);
    }

    mp6550.setPwmPercent(0);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent());
        delay(100);
    }

    mp6550.setPwmPercent(-0.5);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent());
        delay(100);
    }

    mp6550.setPwmPercent(0);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent());
        delay(100);
    }
}
