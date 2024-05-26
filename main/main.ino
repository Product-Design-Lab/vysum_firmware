#include "PDL_Async_Button.h"
#include "WaterdropSensor.h"
#include "PDL_Motor_Controller.h"
#include "PDL_Shutdown_Timer.h"
#include "pins.h"

#define GRIP_PWM -0.5
#define RELEASE_PWM 0.5
#define DISPENSE_MOTOR_ADVANCE 10000
#define STALL_THRESHOLD_SPEED 10

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder); // Pass references here
PDL_Async_Button button;
WaterdropSensor dropSensor;
PDL_Shutdown_Timer shutdownTimer(PIN_POWER_EN, 30, HIGH);

uint8_t buttonState;

typedef enum
{
  STATE_INIT,
  STATE_GRIP,
  STATE_RELEASE,
  STATE_IDLE,
  STATE_DISPENSE,
  STATE_PAUSE,
  STATE_RETRACT,
  STATE_MAX
} main_state_t;

main_state_t main_state = STATE_INIT;

void gripping()
{
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
}

void releasing()
{
  Serial.println("Opening grip...");
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

void dispense()
{
  Serial.println("Dispensing...");
  motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
  delay(1000);
  // wait for motor to reach target position or motor stall or drop detected
  while (1)
  {
    float spd = motor_controller.getCurrentSpeed();
    float pos = motor_controller.getCurrentPosition();
    // Serial.printf("spd:%.2f, pos:%.2f\n", spd, pos);

    if (fabs(spd) < 5)
    {
      Serial.println("Motor stalled");
      break;
    }
    if (dropSensor.getDropCount() > 0)
    {
      dropSensor.setDropCount(0);
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
}

void retract()
{
  Serial.println("Retracting...");
  motor_controller.setTargetPosition(0);
  while (1)
  {
    float spd = motor_controller.getCurrentSpeed();
    float pos = motor_controller.getCurrentPosition();
    // Serial.printf("spd:%.2f, pos:%.2f\n", spd, pos);

    if (fabs(spd) < 5)
    {
      Serial.println("Motor stopped");
      break;
    }
    delay(100);
  }
  Serial.println("Retract complete.");
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  
  digitalWrite(LED_GREEN, LOW);

  shutdownTimer.setDebug(PDL_Shutdown_Timer::DEBUG_ON);
  // shutdownTimer.start();

  mp6550.setPwmPin(PIN_MOTOR_PWM_1, PIN_MOTOR_PWM_2);
  mp6550.setDirNoPin();
  mp6550.setEnableNoPin();
  mp6550.setMaxPwm(255);
  mp6550.setDebug(false);

  encoder.begin(PIN_MOTOR_ENCODER_A, PIN_MOTOR_ENCODER_B);
  encoder.start();

  motor_controller.setPositionLimits(20000, -20000);
  motor_controller.setGain(-0.002);
  motor_controller.setDebug(true);
  motor_controller.setLoopDelay(50);
  motor_controller.start();

  button.setPin(PIN_BUTTON);
  button.setDebounceTime(5);
  button.setLongPressTime(1000);
  button.init();

  dropSensor.init();
  // dropSensor.pause();
  dropSensor.setDebug(dropSensor.DEBUG_LOWPASS);
  dropSensor.setCrossCountTrigThreshold(4);
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop()
{
  if (Serial.available())
  {
    int pos = Serial.parseInt();
    if (pos >= 0 && pos <= 10)
    {
      dropSensor.setDebug((uint8_t)pos);
      // Serial.printf("Set debug flag: %d\n", pos);
    }
    else
    {
      motor_controller.setTargetPosition(pos);
      Serial.printf("Target position: %d\n", pos);
    }
  }

  buttonState = button.getState();
  // state transition
  switch (main_state)
  {
  case STATE_INIT:
    if (buttonState == PDL_Async_Button::BUTTON_SHORT_PRESS)
    {
      main_state = STATE_GRIP;
    }
    else if (buttonState == PDL_Async_Button::BUTTON_LONG_PRESS)
    {
      main_state = STATE_RELEASE;
    }
    break;
  case STATE_GRIP:
    main_state = STATE_IDLE;
    break;
  case STATE_RELEASE:
    main_state = STATE_INIT;
    break;
  case STATE_IDLE:
    if (buttonState == PDL_Async_Button::BUTTON_SHORT_PRESS)
    {
      main_state = STATE_DISPENSE;
    }
    else if (buttonState == PDL_Async_Button::BUTTON_LONG_PRESS)
    {
      main_state = STATE_RELEASE;
    }
    break;
  case STATE_DISPENSE:
    main_state = STATE_RETRACT;
    break;
  case STATE_RETRACT:
    main_state = STATE_IDLE;
    break;
  }

  switch (main_state)
  {
  // state action
  case STATE_GRIP:
    gripping();
    // shutdownTimer.reset();
    break;
  case STATE_RELEASE:
    releasing();
    // shutdownTimer.reset();
    break;
  case STATE_DISPENSE:
    dispense();
    // shutdownTimer.reset();
    break;
  case STATE_RETRACT:
    retract();
    // shutdownTimer.reset();
    break;
  default:
    break;
  }
  delay(200);
}