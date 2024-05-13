#include "motor_controller.h"

void MotorController::setPositionLimits(const int32_t max_pos, const int32_t min_pos)
{
    if (max_pos < min_pos)
    {
        this->max_pos = min_pos;
        this->min_pos = max_pos;
    }
    else
    {
        this->max_pos = max_pos;
        this->min_pos = min_pos;
    }
}

void MotorController::setTargetPosition(const int32_t target_position)
{
    int32_t _target_position = target_position;
    if (target_position > max_pos)
    {
        _target_position = max_pos;
    }
    else if (target_position < min_pos)
    {
        _target_position = min_pos;
    }
    this->target_position = _target_position;
    control_mode = CONTROL_POSITION;

    onTargetReachCalled = false;
    onMotorStallCalled = false;
}

void MotorController::setCurrentPosition(const int32_t current_position)
{
    encoder.writeAbs(current_position);
    this->current_position = current_position;
}

void MotorController::setGain(const float Kp, const float Ki, const float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void MotorController::setPwm(float u)
{
    this->u = u;
    control_mode = CONTROL_PWM;
    onTargetReachCalled = false;
    onMotorStallCalled = false;
}

void MotorController::printDebug()
{
    // print mode
    switch (debug_option)
    {
    case DEBUG_OFF:
        return;
    case DEBUG_CONTROL_LOOP:

        if (control_mode == CONTROL_PWM)
        {
            Serial.printf("pwm mode, current_pos:%ld, current_spd:%6.3f, pwm:%d", current_position, current_speed, u);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            Serial.printf("pos mode, target_pos:%ld, current_pos:%ld, err_kp:%3.3f, err_ki:%3.3f, err_kd:%3.3f, u:%3.3f",
                          target_position, current_position, error, error_integral, error_derivative, u);
            Serial.printf(", speed:%6.3f", current_speed);
        }
        break;
    case DEBUG_EVENT:
        if (motor_stalled)
        {
            Serial.print("Motor Stalled");
        }
        if (target_reached)
        {
            Serial.print("Target Reached");
        }
        break;
    case DEBUG_CURRENT:

        if (motor.hasCurrentPin())
        {
            Serial.printf(", I:%d, (%.3fmA)", motor.getCurrent(), motor.getCurrent_mA());
        }
        break;
    default:
        break;
    }
    Serial.println();
}

void MotorController::motorTaskWrapper(void *parameter)
{
    static_cast<MotorController *>(parameter)->motorTask();
}

void MotorController::checkMotorStall()
{
    // stall condition: position not reached, pwm !=0, speed at 0 for stall_threshold_ms milli sec
    static uint32_t stall_start_tick = 0;

    if (target_reached || current_speed != 0 || u == 0)
    {
        motor_stalled = false;
        stall_start_tick = 0;
        return;
    }

    if (stall_start_tick == 0)
    {
        stall_start_tick = millis();
    }

    if (millis() - stall_start_tick > stall_threshold_ms)
    {
        motor_stalled = true;
    }
}

void MotorController::checkTargetReach()
{
    target_reached = (bool)(abs(target_position - current_position) < position_torlerance);
}

void MotorController::pid_position_control()
{
    // use PID control
    error = target_position - current_position;

    if (Ki != 0)
    {
        error_integral += error;
        error_integral = fmin(fmax(error_integral, -abs(1 / Ki)), abs(1 / Ki)); // anti-windup
    }

    static float prev_error = error;
    error_derivative = error - prev_error;
    prev_error = error;

    u = Kp * error + Ki * error_integral + Kd * error_derivative;
    // Serial.printf("Kp:%3.3f, Ki:%3.3f, Kd:%3.3f, error:%3.3f, error_integral:%3.3f, error_derivative:%3.3f, u:%3.3f\n", Kp, Ki, Kd, error, error_integral, error_derivative, u);

    u = fmin(fmax(u, -1), 1); // normalize u to [-1,1]
    motor.runMotor(u);
}

void MotorController::motorTask()
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        current_speed = 1.0 * (encoder.read()); // need to run at fixed interval
        current_position = encoder.readAbs();

        if (control_mode == CONTROL_PWM)
        {
            motor.runMotor(u);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            pid_position_control();
        }

        checkTargetReach();
        checkMotorStall();

        if (target_reached && (onTargetReach != NULL) && !onTargetReachCalled)
        {
            onTargetReachCalled = true;
            onTargetReach();
        }

        if (motor_stalled && (onMotorStall != NULL) && !onMotorStallCalled)
        {
            onMotorStallCalled = true;
            onMotorStall();
        }

        printDebug();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorController::start(uint8_t priority)
{
    xTaskCreate(motorTaskWrapper, "motorTask", 2048, this, priority, &motorTaskHandle);
}

// callback functions
void MotorController::setOnMotorStall(MotorEventCallback callback)
{
    onMotorStall = callback;
}

void MotorController::setOnTargetReach(MotorEventCallback callback)
{
    onTargetReach = callback;
}
