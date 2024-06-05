#include "PDL_Async_Button.h"
#include "WaterdropSensor.h"
#include "PDL_Motor_Controller.h"
#include "PDL_Shutdown_Timer.h"
#include "PDL_RGB_Indicator.h"
#include "PDL_Tilt_Sensor.h"
#include "state_machine.h"

// #include "watchdog.h"
#include "pins.h"
#include "global_config.h"

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder);

PDL_Async_Button button(PIN_BUTTON, HIGH);

APDS9960 apds(Wire, -1);
WaterdropSensor dropSensor(apds);

PDL_Shutdown_Timer shutdownTimer(PIN_POWER_EN, 30, HIGH);

RGB_Indicator led(PIN_LED_RED, PIN_LED_GREEN, PIN_LED_BLUE, false);

PDL_Tilt_Sensor tiltSensor;

void action_init(void)
{
  led.setPattern(WHITE_CONST);
  motor_controller.setPwm(0);
  // motor_controller.pause();
  // dropSensor.pause();
  // tiltSensor.pause();
  shutdownTimer.reset();
  button.enable();
  // Serial.println("Init");
}

void action_grip(void)
{
  led.setPattern(YELLOW_BLINK);
  motor_controller.setPwm(GRIP_PWM);
  // motor_controller.start();
  // dropSensor.pause();
  // tiltSensor.pause();
  shutdownTimer.reset();
  button.disable();
  // Serial.println("Gripping");
}

void action_idle(void)
{
  led.setPattern(GREEN_BREATHING);
  // motor_controller.pause();
  motor_controller.setCurrentPosition(0);
  motor_controller.setPwm(0);
  // dropSensor.pause();
  // tiltSensor.pause();
  shutdownTimer.reset();
  button.enable();
  // Serial.println("Idle");
}

void action_dispense(void)
{
  led.setPattern(YELLOW_CONST);
  // dropSensor.resume();
  // tiltSensor.resume();
  motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
  // motor_controller.start();
  shutdownTimer.reset();
  button.disable();
  // Serial.println("Dispensing");
}

void action_pause(void)
{
  led.setPattern(RED_BLINK);
  // dropSensor.pause();
  // tiltSensor.resume();
  motor_controller.setPwm(0);
  // motor_controller.pause();
  shutdownTimer.reset();
  button.disable();
  // Serial.println("Paused");
}

void action_retract(void)
{
  led.setPattern(BLUE_CONST);
  // dropSensor.pause();
  // tiltSensor.pause();
  motor_controller.setTargetPosition(0);
  // motor_controller.start();
  shutdownTimer.reset();
  button.disable();
  // Serial.println("Retracting");
}

void action_release(void)
{
  led.setPattern(BLUE_BLINK);
  motor_controller.setPwm(RELEASE_PWM);
  // motor_controller.start();
  // dropSensor.pause();
  // tiltSensor.pause();
  shutdownTimer.reset();
  button.disable();
  // Serial.println("Releasing");
}

void cbs_MotorStall()
{
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


void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Init RGB LED");
  led.setPattern(RAINBOW);
  Serial.println("LED init done");

  // Serial.println("Init watchdog");
  // watchdog_init();
  // Serial.println("watchdog init done");

  Serial.println("Init Shutdown Timer");
  shutdownTimer.setDebug(PDL_Shutdown_Timer::DEBUG_OFF);
  shutdownTimer.start();
  Serial.println("Shutdown Timer init done");

  Serial.println("Init Motor Driver");
  mp6550.setPwmPin(PIN_MOTOR_PWM_1, PIN_MOTOR_PWM_2);
  mp6550.setDirNoPin();
  mp6550.setEnableNoPin();
  mp6550.setMaxPwm(255);
  mp6550.setDebug(false);
  mp6550.setVisenPin(PIN_VISEN);
  mp6550.setVisenSensitivity(5.0);
  Serial.println("Motor Driver init done");

  Serial.println("Init Encoder");
  encoder.begin(PIN_MOTOR_ENCODER_A, PIN_MOTOR_ENCODER_B);
  encoder.start();
  Serial.println("Encoder init done");

  Serial.println("Init Motor Controller");
  motor_controller.setPositionLimits(20000, -20000);
  motor_controller.setGain(0.002, 0, 0);
  motor_controller.setStallThreshold(100, 0.5);
  motor_controller.setDebug(MotorController::DEBUG_OFF);
  motor_controller.setLoopDelay(50);
  motor_controller.setOnMotorStall(cbs_MotorStall);
  motor_controller.setOnTargetReach(cbs_MotorReach);
  motor_controller.start();
  Serial.println("Motor Controller init done");

  Serial.println("Init Button");
  button.setDebounceTime(5);
  button.setLongPressTime(1000);
  button.setShortPressCallback(cbs_ButtonShortPress);
  button.setLongPressCallback(cbs_ButtonLongPress);
  button.init();
  Serial.println("Button init done");

  Serial.println("Init Drop sensor");
  dropSensor.setDebug(WaterdropSensor::DEBUG_INFO);
  dropSensor.setCrossCountTrigThreshold(4);
  dropSensor.setDropDetectedCallback(cbs_DropDetected, nullptr);
  dropSensor.init();
  Serial.println("Drop sensor init done");

  Serial.println("Init Tilt sensor");
  tiltSensor.setDebugStatus(IMU_DEBUG_STATUS_NONE);
  tiltSensor.setVerticalThresholds(-10, 10, -10, 10);
  tiltSensor.setLoopDelay(100);
  // tiltSensor.setTiltedCallback(cbs_DeviceTilted);
  // tiltSensor.setLevelCallback(cbs_DeviceVertical);
  tiltSensor.init();
  Serial.println("Tilt sensor init done");

  Serial.println("Init State Machine");
  SetInitAction(action_init);
  SetGrippingAction(action_grip);
  SetReleasingAction(action_release);
  SetIdleAction(action_idle);
  SetDispensingAction(action_dispense);
  SetRetractingAction(action_retract);
  SetPauseAction(action_pause);
  StateMachine_Init();
  Serial.println("State Machine init done");

  action_init();

  Serial.println("Setup Completed, RTOS Scheduler Started");
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop()
{
  delay(1000);
  // watchdog_feed();
}