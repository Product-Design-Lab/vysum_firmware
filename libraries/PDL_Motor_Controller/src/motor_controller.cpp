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

void MotorController::setGain(const float K)
{
    this->Kp = K;
}

void MotorController::setPwm(float u)
{
    this->u = u;
    control_mode = CONTROL_PWM;
}

void MotorController::setDebug(const bool debug)
{
    this->debug_enabled = debug;
}

void MotorController::printDebug()
{
    if (control_mode == CONTROL_PWM)
    {
        // position, speed, target pwm, current pwm
        Serial.printf("pwm mode, current_pos:%ld, current_spd:%6.3f, pwm:%df\n", current_position, current_speed, u);
    }
    else if (control_mode == CONTROL_POSITION)
    {
        // target position, current position, error, Kp, u, current pwm, current speed
        Serial.printf("pos mode, target_pos:%ld, current_pos:%ld, spd:%6.3f, err:%6.3f, u:%6.3f\n", target_position, current_position, current_speed, error, u);
    }
}

void MotorController::motorTaskWrapper(void *parameter)
{
    static_cast<MotorController *>(parameter)->motorTask();
}

void MotorController::motorTask()
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10; // 10ms
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
            error = target_position - current_position;
            u = Kp * error;
            u = fmin(fmax(u, -1), 1); // normalize u to [-1,1]
            motor.runMotor(u);
        }

        if(debug_enabled)
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