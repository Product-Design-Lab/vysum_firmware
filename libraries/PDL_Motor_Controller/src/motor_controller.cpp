#include "motor_controller.h"

MotorController::MotorController(MotorDriver &motor, HwRotaryEncoder &encoder) : motor(motor), encoder(encoder)
{
    // do nothing
}

MotorController::~MotorController()
{
    // do nothing
}

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
}

int32_t MotorController::getCurrentPosition()
{
    return current_position;
}

void MotorController::setCurrentPosition(const int32_t current_position)
{
    encoder.writeAbs(current_position);
    this->current_position = current_position;
}

void MotorController::setGain(const float Kp, const float Ki)
{
    this->Kp = Kp;
    this->Ki = Ki;
}

void MotorController::setPwm(float u)
{
    this->u = u;
    control_mode = CONTROL_PWM;
}

void MotorController::setLoopDelay(const uint32_t delay_ms)
{
    xFrequency = pdMS_TO_TICKS(delay_ms);
}

void MotorController::setDebug(const bool debug)
{
    this->debug_enabled = debug;
}

void MotorController::printDebug()
{
    // print mode
    if (control_mode == CONTROL_PWM)
    {
        Serial.printf("pwm mode, current_pos:%ld, current_spd:%6.3f, pwm:%d", current_position, current_speed, u);
    }
    else if (control_mode == CONTROL_POSITION)
    {
        Serial.printf("pos mode, target_pos:%ld, current_pos:%ld, err_kp:%3.3f, err_ki:%3.3f, u:%3.3f", target_position, current_position, error, error_integral, u);
        Serial.printf(", speed:%6.3f", current_speed);
    }

    if (motor.hasCurrentPin())
    {
        Serial.printf(", I:%d, (%.3fmA)", motor.getCurrent(), motor.getCurrent_mA());
    }
    Serial.println();
}

void MotorController::motorTaskWrapper(void *parameter)
{
    static_cast<MotorController *>(parameter)->motorTask();
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
            // use PI control
            error = target_position - current_position;

            if (Ki != 0.0)
            {
                error_integral += error;
                error_integral = fmin(fmax(error_integral, -abs(1 / Ki)), abs(1 / Ki)); // anti-windup
                u = Kp * error + Ki * error_integral;
            }
            else
            {
                error_integral = 0;
                u = Kp * error;
            }

            u = fmin(fmax(u, -1), 1); // normalize u to [-1,1]
            motor.runMotor(u);
        }

        if (debug_enabled)
        {
            printDebug();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorController::start(uint8_t priority)
{
    xTaskCreate(motorTaskWrapper, "motorTask", 2048, this, priority, &motorTaskHandle);
}