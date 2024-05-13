#include <Arduino.h>
#include "motor_controller.h"

MotorDriver mp6550;
HwRotaryEncoder encoder; 
MotorController motor_controller(mp6550, encoder); // Pass references here

void pos_reached_cbs()
{
    Serial.println("MAIN: Position reached");
}

void motor_stalled_cbs()
{
    Serial.println("MAIN: Motor stalled");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    mp6550.setPwmPin(D8, D9);
    mp6550.setDirNoPin();
    mp6550.setEnableNoPin();
    mp6550.setVisenPin(A1);
    mp6550.setVisenSensitivity(5.0);// breakout board gives 200 mV/A, so 5 A/V
    mp6550.setMaxPwm(255);
    mp6550.setDebug(false);

    encoder.begin(D2, D3);
    encoder.start();

    motor_controller.setPositionLimits(20000, -20000);
    motor_controller.setGain(0.005);
    motor_controller.setDebug(MotorController::DEBUG_OFF);
    motor_controller.setLoopDelay(50);
    motor_controller.setOnTargetReach(pos_reached_cbs);
    motor_controller.setOnMotorStall(motor_stalled_cbs);
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
