#include "PDL_Async_Button.h"
#include "DropDetection.h"
#include "PDL_N20_Motor_Control.h"

#define GRIP_PWM -0.5
#define RELEASE_PWM 0.5
#define DISPENSE_MOTOR_ADVANCE 20000
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  PDL_N20_Motor_Control::setPin(2, 3, 9, 8, 10);
  PDL_N20_Motor_Control::setMaxPwm(255);
  PDL_N20_Motor_Control::setPositionLimits(20000, -20000);
  PDL_N20_Motor_Control::setGain(-0.01);
  PDL_N20_Motor_Control::enable();
  PDL_N20_Motor_Control::init();

  PDL_Async_Button::setPin(D0);
  PDL_Async_Button::setDebounceTime(5);
  PDL_Async_Button::setLongPressTime(1000);
  PDL_Async_Button::init();
  PDL_Async_Button::getState();

  APDS_DropSensor::init();
  APDS_DropSensor::pause();

  // wait for long press to open grip
  Serial.println("Long press to open grip");
  while (PDL_Async_Button::getState() != PDL_Async_Button::LONG_PRESS) {
    delay(100);
  }

  // fully open grip
  Serial.println("Opening grip...");
  PDL_N20_Motor_Control::setPwmPercent(RELEASE_PWM);
  delay(1000);
  while (fabs(PDL_N20_Motor_Control::getCurrentSpeed()) > 10) {
    delay(100);
  }

  PDL_N20_Motor_Control::setPwmPercent(0);

  // wait for short press to close grip
  Serial.println("Short press to close grip");
  while (PDL_Async_Button::getState() != PDL_Async_Button::SHORT_PRESS) {
    delay(100);
  }

  // lightly grip the bottle
  Serial.println("Closing grip...");
  PDL_N20_Motor_Control::setPwmPercent(GRIP_PWM);
  delay(1000);
  while (fabs(PDL_N20_Motor_Control::getCurrentSpeed()) > 10) {
    delay(100);
  }

  Serial.println("Grip closed, posiiton recorded.");
  PDL_N20_Motor_Control::setPwmPercent(0);
  PDL_N20_Motor_Control::setCurrentPosition(0);
  PDL_N20_Motor_Control::setTargetPosition(0);

  Serial.println("short press to dispense, long press to release");
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop() {
  if (Serial.available()) {
    int pos = Serial.parseInt();
    PDL_N20_Motor_Control::setTargetPosition(pos);
    Serial.printf("Target position: %d\n", pos);
  }

  if (PDL_Async_Button::getState() == PDL_Async_Button::SHORT_PRESS) {
    Serial.println("Short press detected, Dispensing...");
    APDS_DropSensor::resume();
    PDL_N20_Motor_Control::setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    delay(1000);
    // wait for motor to reach target position or motor stall or drop detected
    while (1) {
      float spd = PDL_N20_Motor_Control::getCurrentSpeed();
      float pos = PDL_N20_Motor_Control::getCurrentPosition();
      Serial.printf("spd:%f, pos:%f\n", spd, pos);

      if (fabs(spd) < 10) {
        Serial.println("Motor stalled");
        break;
      }
      if (APDS_DropSensor::get_drop_count() > 0) {
        APDS_DropSensor::set_drop_count(0);
        Serial.println("Drop detected");
        break;
      }
      if (fabs(DISPENSE_MOTOR_ADVANCE-pos)<50) {
        Serial.println("Motor reached target position");
        break;
      }
      delay(100);
    }

    PDL_N20_Motor_Control::setTargetPosition(0);
    while (fabs(PDL_N20_Motor_Control::getCurrentSpeed()) > 10) {
      delay(100);
    }
  }

  // if (PDL_Async_Button::getState() == PDL_Async_Button::LONG_PRESS) {
  //   Serial.println("Long pressed detected, Opening grip...");
  //   APDS_DropSensor::pause();
  //   PDL_N20_Motor_Control::setPwmPercent(RELEASE_PWM);
  //   delay(1000);
  //   while (fabs(PDL_N20_Motor_Control::getCurrentSpeed()) > 10) {
  //     delay(100);
  //   }
  //   PDL_N20_Motor_Control::setPwmPercent(0);
  // }

  delay(200);
}