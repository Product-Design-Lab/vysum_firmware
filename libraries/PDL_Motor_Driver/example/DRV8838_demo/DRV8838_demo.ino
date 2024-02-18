#include "motor_driver.h"



// create an instance of the class
MotorDriver drv8838;

// DRV8838 has 1 PWM pin, 1 direction pin, no enable pin, and no current sense pin

void setup()
{
    // set the PWM pin
    drv8838.setPwmPin(9);

    // set the direction pin
    drv8838.setDirPin(8);

    // set the enable pin
    drv8838.setEnableNoPin();

    // set the visen pin
    drv8838.senVisenNoPin();

    // set the maximum PWM
    drv8838.setMaxPwm(255);

}

void loop()
{
    // set the PWM to 50%
    drv8838.setPwmPercent(0.5);
    delay(1000);

    // set the PWM to 0%
    drv8838.setPwmPercent(0);
    delay(1000);

    // set the PWM to -50%
    drv8838.setPwmPercent(-0.5);
    delay(1000);

    // set the PWM to 0%
    drv8838.setPwmPercent(0);
    delay(3000);

}