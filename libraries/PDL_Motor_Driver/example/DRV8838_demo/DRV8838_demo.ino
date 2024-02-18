#include "motor_driver.h"



// create an instance of the class
MotorDriver drv8838;

// DRV8838 has 1 PWM pin, 1 direction pin, no enable pin, and no current sense pin

void setup()
{
    drv8838.setPwmPin(9);
    drv8838.setDirPin(10);
    drv8838.setEnableNoPin();
    drv8838.setVisenNoPin();
    drv8838.setMaxPwm(255);
    drv8838.setDebug(true);
}

void loop()
{
    drv8838.runMotor(0.5);
    delay(1000);
    drv8838.runMotor(0);
    delay(1000);
    drv8838.runMotor(-0.5);
    delay(1000);
    drv8838.runMotor(0);
    delay(1000);

}