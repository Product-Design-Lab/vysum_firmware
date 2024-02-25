#include "PDL_Async_Button_Group.h"
#include "DropDetection.h"
#include "motor_controller.h"

#define GRIP_PWM -0.5
#define RELEASE_PWM 0.5
#define DISPENSE_MOTOR_ADVANCE 10000
#define STALL_THRESHOLD_SPEED 10

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder); // Pass references here
AsyncButtonGroup button;
uint8_t buttonState;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  mp6550.setPwmPin(D8, D9);
  mp6550.setDirNoPin();
  mp6550.setEnableNoPin();
  mp6550.setMaxPwm(255);
  mp6550.setDebug(false);

  encoder.begin(D2, D3);
  encoder.start();

  motor_controller.setPositionLimits(20000, -20000);
  motor_controller.setGain(-0.002);
  motor_controller.setDebug(false);
  motor_controller.setLoopDelay(50);
  motor_controller.start();

  button.setPin(D0);
  button.setDebounceTime(5);
  button.setLongPressTime(1000);
  button.init();

  APDS_DropSensor::init();
  APDS_DropSensor::pause();

  // wait for short press
  Serial.println("short press to grip");
  while (button.getState() != AsyncButtonGroup::BUTTON_SHORT_PRESS)
  {
    delay(100);
  }

  // grip the bottle
  Serial.println("Gripping...");
  motor_controller.setPwm(GRIP_PWM);
  delay(1000);
  while (fabs(motor_controller.getCurrentSpeed()) > STALL_THRESHOLD_SPEED)
  {
    delay(100);
  }

  Serial.println("Grip closed, Position reset.");
  motor_controller.setPwm(0); // stop motor
  motor_controller.setCurrentPosition(0);
  motor_controller.setTargetPosition(0);

  Serial.println("short press to dispense, long press to release");
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop()
{
  if (Serial.available())
  {
    int pos = Serial.parseInt();
    motor_controller.setTargetPosition(pos);
    Serial.printf("Target position: %d\n", pos);
  }

  buttonState = button.getState();

  if (buttonState == AsyncButtonGroup::BUTTON_SHORT_PRESS)
  {
    Serial.println("Short press detected, Dispensing...");
    APDS_DropSensor::resume();
    APDS_DropSensor::set_drop_count(0);
    motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    delay(1000);
    // wait for motor to reach target position or motor stall or drop detected
    while (1)
    {
      float spd = motor_controller.getCurrentSpeed();
      float pos = motor_controller.getCurrentPosition();
      Serial.printf("spd:%.2f, pos:%.2f\n", spd, pos);

      if (fabs(spd) < 5)
      {
        Serial.println("Motor stalled");
        break;
      }
      if (APDS_DropSensor::get_drop_count() > 0)
      {
        APDS_DropSensor::set_drop_count(0);
        Serial.println("Drop detected");
        break;
      }
      if (fabs(DISPENSE_MOTOR_ADVANCE - pos) < 100)
      {
        Serial.println("Motor reached target position");
        break;
      }
      delay(100);
    }

    motor_controller.setTargetPosition(0);
    while (1)
    {
      float spd = motor_controller.getCurrentSpeed();
      float pos = motor_controller.getCurrentPosition();
      Serial.printf("spd:%.2f, pos:%.2f\n", spd, pos);

      if (fabs(spd) < 5)
      {
        Serial.println("Motor stopped");
        break;
      }
      delay(100);
    }
    Serial.println("Dispense complete.");
  }
  else if (buttonState == AsyncButtonGroup::BUTTON_LONG_PRESS)
  {
    Serial.println("Long pressed detected, Opening grip...");
    APDS_DropSensor::pause();
    float start_pos = motor_controller.getCurrentPosition();
    motor_controller.setPwm(RELEASE_PWM);
    delay(1000);
    while (1)
    {
      if (fabs(motor_controller.getCurrentSpeed()) < 5)
      {
        Serial.println("Motor stalled, Grip opened");
        break;
      }
      else if (fabs(motor_controller.getCurrentPosition() - start_pos) > 1000)
      {
        Serial.println("distance limit reached, Grip opened");
        break;
      }
      else if (digitalRead(D0) == LOW)
      {
        Serial.println("Button pressed, stop Grip openening");
        break;
      }

      delay(100);
    }
    motor_controller.setPwm(0);
  }

  delay(200);
}