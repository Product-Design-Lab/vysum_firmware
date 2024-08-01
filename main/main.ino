#include "Adafruit_NeoPixel.h"
#include "Adafruit_TinyUSB.h"
#include "PDL_Addressable_LED.h"
#include "PDL_Async_Button.h"
#include "PDL_Motor_Controller.h"
#include "PDL_RGB_Indicator.h"
#include "PDL_Shutdown_Timer.h"
#include "PDL_Tilt_Sensor.h"
#include "WaterdropSensor.h"
#include "event_timer.h"
#include "state_machine.h"

#include "global_config.h"
#include "pins.h"

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder);

PDL_Async_Button button(PIN_BUTTON, HIGH);

APDS9960 apds(Wire, -1);
WaterdropSensor dropSensor(apds);

PDL_Shutdown_Timer shutdownTimer(PIN_POWER_EN, SHUTDOWN__TIMEOUT, HIGH);

RGB_Indicator led(PIN_LED_RED, PIN_LED_GREEN, PIN_LED_BLUE, false);

PDL_Tilt_Sensor tiltSensor;

Adafruit_NeoPixel np(LED_COUNT, PIN_LED_DATA,
                     NEO_GRB + NEO_KHZ800); // do not use this object directly, use led_ring instead
PDL_Addressable_LED led_ring(np);

bool flag_position_reset = false;

// *************************************************
// *** Actions *************************************
// *************************************************

void action_init(void)
{
    led.setPattern(WHITE_CONST);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_GREEN_CONST_ALL);

    motor_controller.setPwm(0);
    shutdownTimer.reset();
    button.enable();
}

void action_grip(void)
{
    led.setPattern(YELLOW_BLINK);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_FADE_ALL);

    motor_controller.setPwm(GRIP_PWM);
    delay(200);
    motor_controller.setStallThreshold(200, 0.4);
    shutdownTimer.reset();
    button.disable();
    flag_position_reset = false;
}

void action_idle(void)
{
    led.setPattern(GREEN_BREATHING);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_GREEN_MARQUEE_CIRCULAR);

    if (!flag_position_reset)
    {
        motor_controller.setCurrentPosition(0);
        flag_position_reset = true;
        motor_controller.setStallThreshold(50, 0.5);
    }

    if (tilt_limit_flag) 
    {
      if (tiltSensor.isVertical())
      {
        // case when device is vertical already. Note: may need to remain vertiel for some time
        HandleEvent(EVENT_DEVICE_VERTICAL);
      }
    }
    else 
    {
      HandleEvent(EVENT_DEVICE_VERTICAL);
    }
    

    motor_controller.setPwm(0);
    shutdownTimer.reset();
    button.enable();
    event_timer_stop();
}

void action_ready(void)
{
    led.setPattern(GREEN_CONST);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_GREEN_CONST_ALL);

    motor_controller.setPwm(0);
    shutdownTimer.reset();
    button.enable();
}

void action_dispense(void)
{
    motor_controller.setMaxSpeed(DISPENSE_FAST_SPEED); 
    event_timer_set_timeout(DISPENSE_FAST_TIMEOUT); // allow for 2 seconds
    event_timer_reset();
    led.setPattern(YELLOW_CONST);
    // led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_MARQUEE_CIRCULAR);
    
    // motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    motor_controller.setPwm(-0.8);
    
    shutdownTimer.reset();
    button.disable();
}

void action_slow_dispense(void)
{
    motor_controller.setMaxSpeed(DISPENSE_SLOW_SPEED);
    event_timer_set_timeout(DISPENSE_SLOW_TIMEOUT); // allow for additional 2 seconds
    event_timer_reset();
    led.setPattern(YELLOW_BLINK);
    // led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_MARQUEE_CIRCULAR);
    // motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    motor_controller.setPwm(-0.7);
    shutdownTimer.reset();
    button.disable();
}

void action_hold(void)
{
    // motor_controller.setMaxSpeed(DISPENSE_SLOW_SPEED);
    event_timer_set_timeout(HOLD_TIMEOUT); // allow for additional 2 seconds
    event_timer_reset();
    led.setPattern(YELLOW_BLINK);
    // delay(200);
    // led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_MARQUEE_CIRCULAR);
    // motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    // motor_controller.setPwm(-0.7);
    shutdownTimer.reset();
    button.disable();
}

void action_high_dispense(void)
{
    // motor_controller.setMaxSpeed(DISPENSE_HIGH_SPEED);
    event_timer_set_timeout(DISPENSE_HIGH_TIMEOUT); // allow for additional 2 seconds
    event_timer_reset();
    led.setPattern(YELLOW_BLINK);
    // led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_MARQUEE_CIRCULAR);
    // motor_controller.setTargetPosition(DISPENSE_MOTOR_ADVANCE);
    motor_controller.setPwm(-0.9);
    shutdownTimer.reset();
    button.disable();
}

void action_retract(void)
{
    motor_controller.setMaxSpeed(RETRACT_SPEED); 
    event_timer_set_timeout(RETRACT_TIMEOUT);
    event_timer_reset();
    led.setPattern(BLUE_CONST);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_BLUE_MARQUEE_CIRCULAR);
    motor_controller.setTargetPosition(-500);
    shutdownTimer.reset();
    button.disable();
}

void action_release(void)
{
    event_timer_set_timeout(2000);
    event_timer_reset();
    led.setPattern(BLUE_BLINK);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_BLUE_FADE_ALL);
    motor_controller.setPwm(RELEASE_PWM * 1.5);
    delay(200);
    motor_controller.setPwm(RELEASE_PWM);
    shutdownTimer.reset();
    button.disable();
}

// *************************************************
// *** Callbacks ***********************************
// *************************************************

void cbs_ButtonShortPress()
{
    HandleEvent(EVENT_SHORT_PRESS);
}

void cbs_ButtonLongPress()
{
    HandleEvent(EVENT_LONG_PRESS);
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

void ApdsEventCallback(APDS_Data::data_crossing_state_t state)
{
    static APDS_Data::data_crossing_state_t state_prev = state;
    static int pair_crossing_count_prev = 0, dot_crossing_count_prev = 0, lp_crossing_count_prev = 0;
    int pair_crossing_count = APDS_Data::sum_pair_cross_count(state);
    int dot_crossing_count = APDS_Data::sum_dot_cross_count(state);
    int lp_crossing_count = APDS_Data::sum_lp_cross_count(state);

    // tune the threshold to get the desired sensitivity
    if (lp_crossing_count >= 1 && lp_crossing_count_prev == 0)
    {
        HandleEvent(EVENT_DROP_SUSPENDING);
    }
    if (pair_crossing_count >= 1 && pair_crossing_count_prev == 0)
    {
        HandleEvent(EVENT_DROP_SUSPENDING);
    }
    if (dot_crossing_count >= 1 && dot_crossing_count_prev == 0)
    {
        HandleEvent(EVENT_DROP_SUSPENDING);
    }

    pair_crossing_count_prev = pair_crossing_count;
    dot_crossing_count_prev = dot_crossing_count;
    lp_crossing_count_prev = lp_crossing_count;
}

void cbs_DeviceTilted()
{
    HandleEvent(EVENT_DEVICE_TILTED);
}

void cbs_DeviceVertical()
{
    HandleEvent(EVENT_DEVICE_VERTICAL);
}

void cbs_Timeout()
{
    HandleEvent(EVENT_TIMEOUT);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && (millis() < SERIAL_TIMEOUT))
        ;

    Serial.println("Init RGB LED");
    led.setPattern(RAINBOW);
    Serial.println("LED init done");

    Serial.println("Init LED Ring");
    led_ring.setDebug(0);
    led_ring.init();
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_GREEN_MARQUEE_CIRCULAR);
    Serial.println("LED Ring init done");

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
    // motor_controller.setGain(0.005);
    motor_controller.setStallThreshold(50, 0.5);
    motor_controller.setDebug(MotorController::DEBUG_OFF);
    motor_controller.setLoopDelay(MOTOR_LOOP_DELAY_MS);
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
    dropSensor.setCrossCountTrigThreshold(3);
    dropSensor.setDropDetectedCallback(cbs_DropDetected, nullptr);
    dropSensor.setSensorDataCrossingCallback(ApdsEventCallback);
    dropSensor.init();
    Serial.println("Drop sensor init done");

    Serial.println("Init Tilt sensor");
    tiltSensor.setDebugStatus(IMU_DEBUG_STATUS_NONE);
    tiltSensor.setVerticalThresholds(-10, 10, -10, 10);
    tiltSensor.setLoopDelay(100);
    if (tilt_limit_flag) 
    {
      tiltSensor.setTiltedCallback(cbs_DeviceTilted);
      tiltSensor.setLevelCallback(cbs_DeviceVertical);
    }
    
    tiltSensor.init();
    Serial.println("Tilt sensor init done");

    Serial.println("Init event timer");
    event_timer_init(2000, cbs_Timeout);
    Serial.println("Event timer init done");

    Serial.println("Init State Machine");
    SetInitAction(action_init);
    SetGrippingAction(action_grip);
    SetIdleAction(action_idle);
    SetReadyAction(action_ready);
    SetDispensingAction(action_dispense);
    SetDispensingSlowAction(action_slow_dispense);
    SetDispensingHighAction(action_high_dispense);
    SetHoldAction(action_hold);
    SetReleasingAction(action_release);
    SetRetractingAction(action_retract);
    StateMachine_Init();
    Serial.println("State Machine init done");

    action_init();

    Serial.println("Setup Completed, RTOS Scheduler Started");
}

// push button to start motor, drop sensor to reverse motor, long press to open grip
void loop()
{
    if (Serial.available())
    {
        // "d_ir 1" to set ir sensor debug to 1
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("d_ir"))
        {
            int debug = input.substring(5).toInt();
            dropSensor.setDebug(debug);
        }
        else if (input.startsWith("release"))
        {
            motor_controller.setPwm(RELEASE_PWM * 1.5);
            delay(1000);
            motor_controller.setPwm(0);
            flag_position_reset = true;
            action_init();
        }
    }

    // Serial.println(motor_controller.getCurrentPosition());

    delay(1000);
}