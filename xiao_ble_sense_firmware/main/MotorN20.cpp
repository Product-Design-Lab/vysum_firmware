#include "MotorN20.hpp"
#include "diagnostics.h"
#include "pins.h"
#include "global_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "RotaryEncoder.h"

#include <Arduino.h>

namespace MotorN20
{

#define PWM_RESOLUTION_BITS 10
#define PWM_MAX (bit(PWM_RESOLUTION_BITS) - 1)
    // private variables

    // control settings
    static control_mode_e control_mode;
    static bool enabled;
    static float target_position = 0;
    static float target_pwm = 0;
    static float Kp = 0;
    static float squeeze_pwm = 0.5;
    static float clamp_pwm = 0.1;
    static float max_position = 200000;

    // feedbacks
    static float current_position = 0;
    static float current_speed = 0;
    static uint16_t current_pwm;

    static TaskHandle_t motorTaskHandle = NULL;

    static void set_pwm(const float percent)
    {
        bool current_dir = percent < 0 ? LOW : HIGH;
        current_pwm = static_cast<uint16_t>(round(abs(percent) * PWM_MAX));
        digitalWrite(PIN_MOTOR_DIR, current_dir);
        analogWrite(PIN_MOTOR_PWM, current_pwm);
    }

    static void run_diagnostics(float error, float u)
    {
        if (control_mode == CONTROL_PWM)
        {
            // position, speed, target pwm, current pwm
            Serial.printf("MotorN20 pwm mode, current_pos:%6.3f, current_spd:%6.3f, target_pwm:%6.3f, current_pwm:%d\n", current_position, current_speed, target_pwm, current_pwm);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            // target position, current position, error, Kp, u, current pwm, current speed
            Serial.printf("MotorN20 pos mode, target_pos:%6.3f, current_pos:%6.3f, error:%6.3f, Kp:%6.3f, u:%6.3f, current_pwm:%d, current_spd:%6.3f\n", target_position, current_position, error, Kp, u, current_pwm, current_speed);
        }
    }

    static void motorTask(void *pvParameters)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_LOOP_DELAY_MS); // 50ms

        float u = 0;
        float error = 0;
        while (1)
        {
            current_speed = 1.0 * (RotaryEncoder.read()) / MOTOR_LOOP_DELAY_MS; // any read action clears current reading. must read before readAbs()
            current_position = RotaryEncoder.readAbs();

            if (enabled)
            {
                if (control_mode == CONTROL_PWM)
                {
                    set_pwm(target_pwm);
                }
                else if (control_mode == CONTROL_POSITION)
                {
                    error = target_position - current_position;
                    u = Kp * error; // typical errpr:50, Kp:-0.02, u:-1
                    u = fmin(fmax(u, -1), 1);
                    set_pwm(u);
                }
                else if (control_mode == CONTROL_CLAMP)
                {
                    set_pwm(clamp_pwm);
                }
                else if (control_mode == CONTROL_SQUEEZE)
                {
                    set_pwm(squeeze_pwm);
                }
                else if (control_mode == CONTROL_RELEASE)
                {
                    set_pwm(-clamp_pwm);
                }
            }
            else // disabled
            {
                set_pwm(0);
            }

            if ((bool)(DIAG::get_opt() & DIAG::D_MOTOR) == true)
            {
                run_diagnostics(error, u);
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void set_default()
    {
        set_mode(CONTROL_PWM);
        set_gain(-0.01);
        set_target_pwm(0);
        set_target_position(0);
        set_max_position(200000);
        set_current_position(0);
        enable();
    }

    int8_t init()
    {
        pinMode(PIN_MOTOR_DIR, OUTPUT);
        digitalWrite(PIN_MOTOR_DIR, HIGH);

        pinMode(PIN_MOTOR_PWM, OUTPUT);
        analogWriteResolution(PWM_RESOLUTION_BITS);
        analogWrite(PIN_MOTOR_PWM, 0);

        RotaryEncoder.begin(PIN_MOTOR_ENCODER_A, PIN_MOTOR_ENCODER_B); // this object is defined as extern in RotaryEncoder.h
        RotaryEncoder.start();

        set_default();

        xTaskCreate(motorTask, "motorTask", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &motorTaskHandle);

        return 0;
    }

    int8_t deinit()
    {
        RotaryEncoder.stop();
        if (motorTaskHandle != NULL)
        {
            vTaskDelete(motorTaskHandle);
            motorTaskHandle = NULL;
        }
        return 0;
    }

    // setter
    void set_mode(const control_mode_e mode)
    {
        control_mode = mode;
    }

    void set_squeeze_pwm(const float percent)
    {
        if (percent > 1)
            squeeze_pwm = 1;
        else if (percent < -1)
            squeeze_pwm = -1;
        else
            squeeze_pwm = percent;
    }

    void set_clamp_pwm(const float percent)
    {
        if (percent > 1)
            clamp_pwm = 1;
        else if (percent < -1)
            clamp_pwm = -1;
        else
            clamp_pwm = percent;
    }

    void set_target_pwm(const float percent)
    {
        if (percent > 1)
            target_pwm = 1;
        else if (percent < -1)
            target_pwm = -1;
        else
            target_pwm = percent;
    }

    void set_max_position(const float position)
    {
        max_position = position;
    }

    void set_target_position(const float position)
    {
        if (position > max_position)
            target_position = max_position;
        else if (position < -max_position)
            target_position = -max_position;
        else
            target_position = position;
    }

    void set_current_position(const float position)
    {
        current_position = position;
        target_position = position; // prevent sudden change in control input
    }

    void set_gain(const float gain)
    {
        Kp = gain;
    }

    void enable()
    {
        enabled = true;
    }
    void disable()
    {
        enabled = false;
    }

    uint16_t get_current_pwm()
    {
        return current_pwm;
    }

    float get_current_speed()
    {
        return current_speed;
    }

    float get_current_position()
    {
        return current_position;
    }

    void clamp()
    {
        set_mode(CONTROL_CLAMP);
    }
    void squeeze()
    {
        set_mode(CONTROL_SQUEEZE);
    }
    void release()
    {
        set_mode(CONTROL_RELEASE);
    }

} // namespace MotorN20
