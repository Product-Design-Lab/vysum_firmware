#include "PDL_N20_Motor_Control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "RotaryEncoder.h"

namespace PDL_N20_Motor_Control
{
    // private variables
    static uint8_t ENC_A_pin = D2;
    static uint8_t ENC_B_pin = D3;
    static uint8_t PWM_pin = D9;
    static uint8_t DIR_pin = D8;
    static uint8_t DIR_pin2 = D10;
    static bool dir_dual_pin = false;

    static uint32_t max_pwm = 255;
    static uint32_t pwm_raw = 0;
#define FORWARD 0
#define BACKWARD 1
    static bool current_dir = FORWARD;

    static int32_t max_pos = 20000;
    static int32_t min_pos = -20000;
    static int32_t target_position = 0;
    static int32_t current_position = 0;
    static float current_speed = 0;

    static float Kp = 0;

    static bool enabled = false;

    static TaskHandle_t motorTaskHandle = NULL;

    static bool debug_enabled = false;

#define MOTOR_LOOP_DELAY_MS 10

    enum control_mode_e
    {
        CONTROL_PWM,
        CONTROL_POSITION
    };
    control_mode_e control_mode = CONTROL_PWM;

    void setPin(uint8_t ena, uint8_t enb, uint8_t PWM_pin, uint8_t DIR_pin)
    {
        PDL_N20_Motor_Control::ENC_A_pin = ena;
        PDL_N20_Motor_Control::ENC_B_pin = enb;
        PDL_N20_Motor_Control::PWM_pin = PWM_pin;
        PDL_N20_Motor_Control::DIR_pin = DIR_pin;
        dir_dual_pin = false;
    }

    void setPin(uint8_t ena, uint8_t enb, uint8_t PWM_pin, uint8_t DIR_pin1, uint8_t DIR_pin2)
    {
        PDL_N20_Motor_Control::ENC_A_pin = ena;
        PDL_N20_Motor_Control::ENC_B_pin = enb;
        PDL_N20_Motor_Control::PWM_pin = PWM_pin;
        PDL_N20_Motor_Control::DIR_pin = DIR_pin1;
        PDL_N20_Motor_Control::DIR_pin2 = DIR_pin2;
        dir_dual_pin = true;
    }

    void setMaxPwm(uint32_t max_pwm)
    {
        PDL_N20_Motor_Control::max_pwm = max_pwm;
    }

    float getPwmPercent()
    {
        float percent = (float)pwm_raw / (float)max_pwm * (current_dir == FORWARD ? 1 : -1);
        return percent;
    }

    void setPwmPercent(float percent)
    {
        PDL_N20_Motor_Control::control_mode = CONTROL_PWM;
        if (percent > 1)
        {
            percent = 1;
        }
        else if (percent < -1)
        {
            percent = -1;
        }
        current_dir = percent >= 0 ? FORWARD : BACKWARD;
        pwm_raw = (uint32_t)(fabs(percent) * max_pwm);
    }

    void setPositionLimits(int32_t max_pos, int32_t min_pos)
    {
        PDL_N20_Motor_Control::max_pos = max_pos;
        PDL_N20_Motor_Control::min_pos = min_pos;
    }

    void setTargetPosition(int32_t target_position)
    {
        if (target_position > max_pos)
        {
            target_position = max_pos;
        }
        else if (target_position < min_pos)
        {
            target_position = min_pos;
        }
        PDL_N20_Motor_Control::control_mode = CONTROL_POSITION;
        PDL_N20_Motor_Control::target_position = target_position;
    }

    int32_t getCurrentPosition()
    {
        return current_position;
    }

    void setCurrentPosition(int32_t current_position)
    {
        PDL_N20_Motor_Control::current_position = current_position;
    }

    void setGain(float K)
    {
        PDL_N20_Motor_Control::Kp = K;
    }

    void enable()
    {
        PDL_N20_Motor_Control::enabled = true;
    }

    void disable()
    {
        PDL_N20_Motor_Control::enabled = false;
    }

    void enableDebug()
    {
        PDL_N20_Motor_Control::debug_enabled = true;
    }

    void disableDebug()
    {
        PDL_N20_Motor_Control::debug_enabled = false;
    }

    float getCurrentSpeed()
    {
        return current_speed;
    }

    static void set_pwm(float percent)
    {
        if (percent >= 0)
        {
            current_dir = FORWARD;
            pwm_raw = (uint32_t)(percent * max_pwm);
        }
        else
        {
            current_dir = BACKWARD;
            pwm_raw = (uint32_t)(-percent * max_pwm);
        }
        analogWrite(PWM_pin, pwm_raw);
        digitalWrite(DIR_pin, current_dir);
        if (dir_dual_pin)
            digitalWrite(DIR_pin2, !current_dir);
    }

    static void run_diagnostics(float error, float u)
    {
        if (control_mode == CONTROL_PWM)
        {
            // position, speed, target pwm, current pwm
            Serial.printf("MotorN20 pwm mode, current_pos:%ld, current_spd:%6.3f, raw_pwm:%df, max_pwm:%d\n", current_position, current_speed, pwm_raw, max_pwm);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            // target position, current position, error, Kp, u, current pwm, current speed
            Serial.printf("pos mode, target_pos:%ld, current_pos:%ld, spd:%6.3f, err:%6.3f, u:%6.3f,current_pwm:%d\n", target_position, current_position, current_speed, error, u, pwm_raw);
        }
    }

    static void motorTask(void *pvParameters)
    {
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 10; // 10ms
        xLastWakeTime = xTaskGetTickCount();

        float error = 0;
        float u = 0; // control input
        while (1)
        {
            current_speed = 1.0 * (RotaryEncoder.read()) / MOTOR_LOOP_DELAY_MS; // any read action clears current reading. must read before readAbs()
            current_position = RotaryEncoder.readAbs();
            if (enabled)
            {
                if (control_mode == CONTROL_PWM)
                {
                    set_pwm((float)pwm_raw / (float)max_pwm);
                }
                else if (control_mode == CONTROL_POSITION)
                {
                    error = target_position - current_position;
                    u = Kp * error; // typical errpr:50, Kp:-0.02, u:-1
                    u = fmin(fmax(u, -1), 1);
                    set_pwm(u);
                }
            }
            else // disabled
            {
                set_pwm(0);
            }

            if (debug_enabled)
                run_diagnostics(error, u);

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void init()
    {
        // initialize pins
        pinMode(PWM_pin, OUTPUT);
        pinMode(DIR_pin, OUTPUT);
        if (dir_dual_pin)
            pinMode(DIR_pin2, OUTPUT);

        // initialize encoder
        RotaryEncoder.begin(ENC_A_pin, ENC_B_pin);
        RotaryEncoder.start();

        if (motorTaskHandle == NULL)
        {
            xTaskCreate(
                motorTask,         /* Task function. */
                "motorTask",       /* String with name of task. */
                2048,              /* Stack size in words. */
                NULL,              /* Parameter passed as input of the task */
                1,                 /* Priority of the task. */
                &motorTaskHandle); /* Task handle. */
        }
    }

    void deinit()
    {
        RotaryEncoder.stop();
        if (motorTaskHandle != NULL)
        {
            vTaskDelete(motorTaskHandle);
            motorTaskHandle = NULL;
        }
    }
}