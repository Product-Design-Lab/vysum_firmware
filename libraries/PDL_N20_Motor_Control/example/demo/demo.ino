#include "PDL_N20_Motor_Control.h"

void setup()
{
    PDL_N20_Motor_Control::setPin(2, 3, 9, 8, 10);
    PDL_N20_Motor_Control::setMaxPwm(255);
    PDL_N20_Motor_Control::setPositionLimits(20000, -20000);
    PDL_N20_Motor_Control::setGain(-0.01);
    PDL_N20_Motor_Control::enable();
    PDL_N20_Motor_Control::enableDebug();
    PDL_N20_Motor_Control::init();
}

void loop()
{
    PDL_N20_Motor_Control::setPwmPercent(0.5);
    delay(1000);
    PDL_N20_Motor_Control::setPwmPercent(0);
    delay(1000);
    PDL_N20_Motor_Control::setPwmPercent(-0.5);
    delay(1000);
    PDL_N20_Motor_Control::setPwmPercent(0);
    delay(3000);

    PDL_N20_Motor_Control::setTargetPosition(2000);
    delay(1000);
    PDL_N20_Motor_Control::setTargetPosition(0);
    delay(1000);
    PDL_N20_Motor_Control::setTargetPosition(-2000);
    delay(1000);
    PDL_N20_Motor_Control::setTargetPosition(0);
    delay(3000);
}
