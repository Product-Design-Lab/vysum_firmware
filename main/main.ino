#include "PDL_Async_Button.h"
#include "WaterdropSensor.h"
#include "PDL_Motor_Controller.h"
#include "PDL_Shutdown_Timer.h"
#include "PDL_RGB_Indicator.h"
#include "PDL_Tilt_Sensor.h"
#include "state_machine.h"
#include "event_timer.h"
// #include "Adafruit_NeoPixel.h"
#include <math.h>

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
// Adafruit_NeoPixel ring(LED_COUNT, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);


long tic = 0;


bool flag_position_reset = false;

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
  flag_position_reset = false;    // NB
  // Serial.println("Gripping");
}

void action_idle(void)
{
  led.setPattern(GREEN_BREATHING);
  // motor_controller.pause();
  if (!flag_position_reset) {   // NB
    motor_controller.setCurrentPosition(0);
    flag_position_reset = true;
  }


  // double x_nom = 45;
  // double y_nom = 0;
  // double x_tol = 15;
  // double y_tol = 5;


  // double x_measured = -tiltSensor.getAngleX();
  // double y_measured = tiltSensor.getAngleY();

  // int angle_step = 30;

  // double x_delta = x_nom - x_measured;
  // double y_delta = y_nom - y_measured;

  // double theta = atan2(x_delta,y_delta) * 180/PI;

  // int theta_LED = theta/angle_step;

  // double mag = sqrt(x_delta*x_delta + y_delta*y_delta);

  // Serial.println("Angle: " + String(theta) + ", " + String(x_delta) + ", " + String(y_delta) + ", " + String(x_measured) + ", " + String(y_measured));

  // Serial.println(theta_LED);
  
  // Serial.println(mag);


  // if (mag < x_tol) {
  //   for (int i = 0; i < LED_COUNT; i++)
  //     {
  //         ring.setPixelColor(i, ring.Color(0, 255, 0));
  //     }
  // }
  // else {
  //     for (int i = 0; i < LED_COUNT; i++)
  //     {
  //       if (i == theta_LED) {
  //         ring.setPixelColor(i, ring.Color(255, 255, 0));
  //       }
  //       else {
  //         ring.setPixelColor(i, ring.Color(0, 0, 0));
  //       }
        
  //     }
  // }

  // ring.show();
  
  
  motor_controller.setPwm(0);
  // dropSensor.pause();
  // tiltSensor.pause();
  shutdownTimer.reset();
  button.enable();
  event_timer_stop();
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
  event_timer_set_timeout(5000); // wait for 2 seconds
  event_timer_reset();
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
  event_timer_set_timeout(3000); // wait for 3 seconds
  event_timer_reset();
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
  event_timer_set_timeout(2000);
  event_timer_reset();
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

  tic = millis();
  while (!Serial) {
    if (millis() - tic > SERIAL_TIMEOUT) {
      break;
    }
  }
  // while (!Serial)
  //   ;

  // Serial.println("Init LED ring");
  // pinMode(PIN_LED_DATA, OUTPUT);
  // ring.begin();
  // ring.setBrightness(LED_BRIGHTNESS);
  // ring.show();
  // Serial.println("LED ring init done");

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
  dropSensor.setCrossCountTrigThreshold(2);
  dropSensor.setDropDetectedCallback(cbs_DropDetected, nullptr);
  dropSensor.init();
  Serial.println("Drop sensor init done");

  Serial.println("Init Tilt sensor");
  tiltSensor.setDebugStatus(IMU_DEBUG_STATUS_NONE);
  tiltSensor.setVerticalThresholds(-10, 10, -10, 10);
  tiltSensor.setLoopDelay(50);
  // tiltSensor.setTiltedCallback(cbs_DeviceTilted);
  // tiltSensor.setLevelCallback(cbs_DeviceVertical);
  tiltSensor.init();
  Serial.println("Tilt sensor init done");

  Serial.println("Init event timer");
  event_timer_init(5000, cbs_Timeout);
  Serial.println("Event timer init done");

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