#include "motor_driver.h"

#if defined(NRF52840_XXAA) || defined(NRF52833_XXAA)
#include <Adafruit_TinyUSB.h>
#endif

MotorDriver::MotorDriver()
{
    // do nothing
}

MotorDriver::~MotorDriver()
{
    // do nothing
}

void MotorDriver::setPwmPin(const uint8_t pin)
{
    PWM_pin = pin;
    pinMode(PWM_pin, OUTPUT);
    PWM_pin_count = 1;
}

void MotorDriver::setPwmPin(const uint8_t pin1, const uint8_t pin2)
{
    PWM_pin = pin1;
    PWM_pin2 = pin2;
    pinMode(PWM_pin, OUTPUT);
    pinMode(PWM_pin2, OUTPUT);
    PWM_pin_count = 2;
}

void MotorDriver::setDirPin(const uint8_t pin)
{
    DIR_pin = pin;
    pinMode(DIR_pin, OUTPUT);
    dir_pin_count = 1;
}

void MotorDriver::setDirPin(const uint8_t pin1, const uint8_t pin2)
{
    DIR_pin = pin1;
    DIR_pin2 = pin2;
    pinMode(DIR_pin, OUTPUT);
    pinMode(DIR_pin2, OUTPUT);
    dir_pin_count = 2;
}

void MotorDriver::setDirNoPin()
{
    DIR_pin = 0;
    DIR_pin2 = 0;
    dir_pin_count = 0;
}

void MotorDriver::setVisenPin(const uint8_t pin)
{
    VISEN_pin = pin;
    // pinMode(VISEN_pin, INPUT);
    has_visen_pin = 1;
}

void MotorDriver::setVisenNoPin()
{
    VISEN_pin = 0;
    has_visen_pin = 0;
}

void MotorDriver::setEnablePin(const uint8_t pin)
{
    enable_pin = pin;
    pinMode(enable_pin, OUTPUT);
    has_enable_pin = 1;
}

void MotorDriver::setEnableNoPin()
{
    enable_pin = 0;
    has_enable_pin = 0;
}

void MotorDriver::setEnable(bool enable)
{
    if (!has_enable_pin)
    {
        return;
    }
    if (enable)
    {
        digitalWrite(enable_pin, HIGH);
    }
    else
    {
        digitalWrite(enable_pin, LOW);
    }
}

void MotorDriver::setMaxPwm(const uint32_t max_pwm)
{
    this->max_pwm = max_pwm;
}

void MotorDriver::runMotor(const float pwm)
{
    if (pwm > 1)
        target_pwm_f32 = 1;
    else if (pwm < -1)
        target_pwm_f32 = -1;
    else
        target_pwm_f32 = pwm;

    target_pwm_u32 = (uint32_t)(abs(target_pwm_f32) * max_pwm); // abs defined in arudino.h

    if (target_pwm_f32 > 0)
        direction = FORWARD;
    else
        direction = BACKWARD;

    if (dir_pin_count == 1)
    {
        digitalWrite(DIR_pin, direction);

        if (debug_enabled)
            Serial.printf("setting pin %d to %d\n", DIR_pin, direction);
    }
    else if (dir_pin_count == 2)
    {
        digitalWrite(DIR_pin, direction);
        digitalWrite(DIR_pin2, !direction);

        if (debug_enabled)
            Serial.printf("setting pin %d to %d, pin %d to %d\n", DIR_pin, direction, DIR_pin2, !direction);
    }

    if (PWM_pin_count == 1)
    {
        analogWrite(PWM_pin, target_pwm_u32);
        if (debug_enabled)
            Serial.printf("setting pin %d to %d\n", PWM_pin, target_pwm_u32);
    }

    else if (PWM_pin_count == 2)
    {
        analogWrite(PWM_pin, direction * target_pwm_u32);
        analogWrite(PWM_pin2, !direction * target_pwm_u32);
        Serial.printf("setting pin %d to %d, pin %d to %d\n", PWM_pin, direction * target_pwm_u32, PWM_pin2, !direction * target_pwm_u32);
    }
}

int MotorDriver::getCurrent()
{
    if (has_visen_pin == 0)
    {
        return 0;
    }
    return analogRead(VISEN_pin);
}

void MotorDriver::setDebug(bool enable)
{
    debug_enabled = enable;
    if(!Serial)
    {
        Serial.begin(115200);
    }
}