#include "PDL_N20_Motor_Control.h"
#include "Adafruit_TinyUSB.h"

void setup()
{

    Serial.begin(115200);
    while(!Serial);
    PDL_N20_Motor_Control::setPin(2, 3, 9, 8, 10);
    PDL_N20_Motor_Control::setMaxPwm(255);
    PDL_N20_Motor_Control::setCurrentPosition(0);
    PDL_N20_Motor_Control ::setTargetPosition(0);
    PDL_N20_Motor_Control::setPositionLimits(30000, -20000);
    PDL_N20_Motor_Control::setGain(-0.01);
    PDL_N20_Motor_Control::enable();
    PDL_N20_Motor_Control::disableDebug();
    // PDL_N20_Motor_Control::enableDebug();
    PDL_N20_Motor_Control::init();
    Serial.println("calibrating...");
    PDL_N20_Motor_Control::setPwmPercent(-0.5);
    delay(5000);
    Serial.println("calibratiion done");
    PDL_N20_Motor_Control::setPwmPercent(0);
    PDL_N20_Motor_Control::setCurrentPosition(0);
    Serial.println(PDL_N20_Motor_Control::getCurrentPosition());
    delay(5000);
    PDL_N20_Motor_Control ::setTargetPosition(0);
    
}

void loop()
{
    // PDL_N20_Motor_Control::setPwmPercent(0.5);
    // delay(1000);
    // PDL_N20_Motor_Control::setPwmPercent(0);
    // delay(1000);
    // PDL_N20_Motor_Control::setPwmPercent(-0.5);
    // delay(1000);
    // PDL_N20_Motor_Control::setPwmPercent(0);
    // delay(3000);

    // PDL_N20_Motor_Control::setTargetPosition(2000);
    // delay(1000);
    // PDL_N20_Motor_Control::setTargetPosition(0);
    // delay(1000);
    // PDL_N20_Motor_Control::setTargetPosition(-2000);
    // delay(1000);
    // PDL_N20_Motor_Control::setTargetPosition(0);
    // delay(3000);

    if (Serial.available())
    {
        int targetPosition = Serial.parseInt();
        PDL_N20_Motor_Control::setTargetPosition(targetPosition);
    }
}
