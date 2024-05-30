#include "PDL_Async_Button.h"
#include "WaterdropSensor.h"
#include "PDL_Motor_Controller.h"
#include "PDL_Shutdown_Timer.h"
#include "PDL_RGB_Indicator.h"
#include "PDL_Tilt_Sensor.h"
#include "state_machine.h"

#include "pins.h"
#include "global_config.h"

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder); // Pass references here
PDL_Async_Button button;
WaterdropSensor dropSensor;
PDL_Shutdown_Timer shutdownTimer(PIN_POWER_EN, 30, HIGH);
RGB_Indicator led(PIN_LED_RED, PIN_LED_GREEN, PIN_LED_BLUE, false);
PDL_Tilt_Sensor tiltSensor;

void action_grip(void)
{
  motor_controller.setPwm(GRIP_PWM);
  led.setPattern(GREEN_BLINK);
  shutdownTimer.reset();
  Serial.println("Gripping");
}

void action_release(void)
{
  motor_controller.setPwm(RELEASE_PWM);
  led.setPattern(GREEN_CONST);
  shutdownTimer.reset();
}

void action_dispense(void)
{
  motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
  led.setPattern(BLUE_BLINK);
  shutdownTimer.reset();
}

void action_retract(void)
{
  motor_controller.setTargetPosition(0);
  led.setPattern(BLUE_CONST);
  shutdownTimer.reset();
}

void action_pause(void)
{
  motor_controller.setPwm(0);
  led.setPattern(RED_BLINK);
  shutdownTimer.reset();
}

void action_init(void)
{
  motor_controller.setPwm(0);
  motor_controller.setCurrentPosition(0);
  motor_controller.setTargetPosition(0);
  led.setPattern(GREEN_BREATHING);
  shutdownTimer.reset();
}

void action_idle(void)
{
  motor_controller.setPwm(0);
  led.setPattern(GREEN_BREATHING);
  shutdownTimer.reset();
}

void cbs_MotorStall()
{
  // Serial.printf("Motor stalled at position %d\n", motor_controller.getCurrentPosition());
  HandleEvent(EVENT_MOTOR_STALL);
}

void cbs_MotorReach()
{
  HandleEvent(EVENT_DISTANCE_REACHED);
}

void cbs_DropDetected(void *context)
{
  HandleEvent(EVENT_DROP_DETECTED);
}

void cbs_ButtonShortPress()
{
  HandleEvent(EVENT_SHORT_PRESS);
}

void cbs_ButtonLongPress()
{
  HandleEvent(EVENT_LONG_PRESS);
}

void cbs_DeviceTilted()
{
  HandleEvent(EVENT_DEVICE_TILTED);
  Serial.println("Device tilted");
}

void cbs_DeviceVertical()
{
  HandleEvent(EVENT_DEVICE_VERTICAL);
  Serial.println("Device vertical");
}

void cbs_Timeout()
{
  HandleEvent(EVENT_TIMEOUT);
}

void Init_StateMachine(void)
{
  SetInitAction(action_init);
  SetGrippingAction(action_grip);
  SetReleasingAction(action_release);
  SetIdleAction(action_idle);
  SetDispensingAction(action_dispense);
  SetRetractingAction(action_retract);
  SetPauseAction(action_pause);
  StateMachine_Init();
}

void setup()
{
  Serial.begin(115200);

  Init_StateMachine();

  shutdownTimer.setDebug(PDL_Shutdown_Timer::DEBUG_OFF);
  shutdownTimer.start();

  mp6550.setPwmPin(PIN_MOTOR_PWM_1, PIN_MOTOR_PWM_2);
  mp6550.setDirNoPin();
  mp6550.setEnableNoPin();
  mp6550.setMaxPwm(255);
  mp6550.setDebug(false);

  encoder.begin(PIN_MOTOR_ENCODER_A, PIN_MOTOR_ENCODER_B);
  encoder.start();

  motor_controller.setPositionLimits(20000, -20000);
  motor_controller.setGain(0.002, 0, 0);
  motor_controller.setDebug(MotorController::DEBUG_OFF);
  motor_controller.setLoopDelay(50);
  motor_controller.setOnMotorStall(cbs_MotorStall);
  motor_controller.setOnTargetReach(cbs_MotorReach);
  motor_controller.start();

  button.setPin(PIN_BUTTON);
  button.setDebounceTime(5);
  button.setLongPressTime(1000);
  button.setShortPressCallback(cbs_ButtonShortPress);
  button.setLongPressCallback(cbs_ButtonLongPress);
  button.init();

  dropSensor.setDebug(WaterdropSensor::DEBUG_INFO);
  dropSensor.setCrossCountTrigThreshold(4);
  dropSensor.setDropDetectedCallback(cbs_DropDetected, nullptr);
  dropSensor.init();
  // dropSensor.pause();

  tiltSensor.setDebugStatus(IMU_DEBUG_STATUS_ANGLE);
  tiltSensor.setVerticalThresholds(-10, 10, -10, 10);
  tiltSensor.setLoopDelay(100);
  tiltSensor.setTiltedCallback(cbs_DeviceTilted);
  tiltSensor.setLevelCallback(cbs_DeviceVertical);
  tiltSensor.init();
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop()
{
  delay(200);
}