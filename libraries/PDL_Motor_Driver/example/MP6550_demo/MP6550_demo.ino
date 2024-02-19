#include "motor_driver.h"

MotorDriver mp6550;

void setup()
{
    mp6550.setPwmPin(D8, D9);
    mp6550.setDirNoPin();
    mp6550.setEnableNoPin();
    mp6550.setVisenPin(A1);
    mp6550.setVisenSensitivity(5.0); // breakout board gives 200 mV/A, so 5 A/V
    
    mp6550.setMaxPwm(255);

    mp6550.setDebug(true);
}

void loop()
{
    mp6550.runMotor(0.5);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent_mA());
        delay(100);
    }

    mp6550.runMotor(0);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent_mA());
        delay(100);
    }

    mp6550.runMotor(-0.5);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent_mA());
        delay(100);
    }

    mp6550.runMotor(0);
    for (int i = 0; i < 10; i++)
    {
        Serial.println(mp6550.getCurrent_mA());
        delay(100);
    }
}
