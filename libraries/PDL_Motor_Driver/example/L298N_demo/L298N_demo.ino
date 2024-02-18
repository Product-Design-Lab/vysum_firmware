#include <Arduino.h>
#include <motor_driver.h>

MotorDriver l298n;

void setup()
{
    l298n.setPwmPin(9);
    l298n.setDirPin(8, 10);
    l298n.setEnableNoPin();
    l298n.setVisenNoPin();

    l298n.setMaxPwm(255);
    l298n.setDebug(true);
}

void loop()
{
    l298n.runMotor(0.5);
    delay(1000);
    l298n.runMotor(0);
    delay(1000);
    l298n.runMotor(-0.5);
    delay(1000);
    l298n.runMotor(0);
    delay(1000);
}
