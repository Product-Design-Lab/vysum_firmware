#include <Arduino.h>
#include "motor_controller.h"

MotorDriver mp6550;
// HwRotaryEncoder RotaryEncoder; // Assuming you have a class HwRotaryEncoder
MotorController motor_controller(mp6550, RotaryEncoder); // Pass references here

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    mp6550.setPwmPin(D8, D9);
    mp6550.setDirNoPin();
    mp6550.setEnableNoPin();
    mp6550.setVisenPin(A1);
    mp6550.setMaxPwm(255);
    mp6550.setDebug(false);

    RotaryEncoder.begin(D2, D3);
    RotaryEncoder.start();

    motor_controller.setPositionLimits(20000, -20000);
    motor_controller.setGain(-0.01);
    motor_controller.setDebug(true);
    motor_controller.start();
}

void loop()
{
    if (Serial.available())
    {
        int targetPosition = Serial.parseInt();
        motor_controller.setTargetPosition(targetPosition); // Corrected call
    }
    delay(100);
}
