#include "PDL_simple_IMU.h"

#include "Adafruit_TinyUSB.h"
#include "MovingAverage.h"

#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#define DEFAULT_VERTICAL_THRESHOLD_DEGREES 10
#define DEFAULT_TASK_PRIORITY 1
#define DEFAULT_LOOP_DELAY_MS 100
#define TASK_STACK_SIZE 1024
#define DEFAULT_ANGLE_X_OFFSET 90
#define DEFAULT_ANGLE_Y_OFFSET 90

LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A
MovingAverage<int16_t, 16> ax_filter;
MovingAverage<int16_t, 16> ay_filter;
MovingAverage<int16_t, 16> az_filter;

TaskHandle_t imu_task_handle;

float angle_x = 0;
float angle_y = 0;
float vertical_threshold = DEFAULT_VERTICAL_THRESHOLD_DEGREES;
bool is_vertical = false;

uint32_t loop_delay = DEFAULT_LOOP_DELAY_MS;

uint8_t debug_status = 0;

TickType_t xLastWakeTime;

float ax_offset = DEFAULT_ANGLE_X_OFFSET;
float ay_offset = DEFAULT_ANGLE_Y_OFFSET;

static void imu_task(void *pvParameters)
{
    int16_t ax_raw, ay_raw, az_raw;
    float ax, ay, az;

    while (true)
    {
        ax_raw = myIMU.readRawAccelX();
        ay_raw = myIMU.readRawAccelY();
        az_raw = myIMU.readRawAccelZ();

        ax = ax_filter.add(ax_raw);
        ay = ay_filter.add(ay_raw);
        az = az_filter.add(az_raw);

        angle_x = atan2(sqrt(ay * ay + az * az), ax) * 180 / M_PI - ax_offset;
        angle_y = atan2(sqrt(ax * ax + az * az), ay) * 180 / M_PI - ay_offset;

        if (abs(angle_x) < vertical_threshold && abs(angle_y) < vertical_threshold)
        {
            is_vertical = true;
        }
        else
        {
            is_vertical = false;
        }

        switch (debug_status)
        {
        case IMU_DEBUG_STATUS_RAW:
            Serial.printf("ax:%6d, ay:%6d, az:%6d\n", ax_raw, ay_raw, az_raw);
            break;
        case IMU_DEBUG_STATUS_FILTERED:
            Serial.printf("ax:%6d, ay:%6d, az:%6d\n", ax, ay, az);
            break;
        case IMU_DEBUG_STATUS_ANGLE:
            Serial.printf("angle_x:%6.3f, angle_y:%6.3f, is_vertical:%d\n", angle_x, angle_y, is_vertical);
            break;
        case IMU_DEBUG_THRESHOLD:
            Serial.printf("is_vertical:%d\n", is_vertical);
            break;
        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delay));
    }
}

void init_imu(uint32_t priority)
{
    if (myIMU.begin() != 0)
    {
        Serial.println("IMU error");
    }
    else
    {
        Serial.println("IMU OK!");
    }

    xTaskCreate(imu_task, "IMU_TASK", TASK_STACK_SIZE, NULL, priority, &imu_task_handle);
}

void calibrate_imu()
{
    ax_offset = angle_x;
    ay_offset = angle_y;
}

void set_debug_status(uint8_t status)
{
    debug_status = status;
}

void set_vertical_threshold(float threshold)
{
    vertical_threshold = threshold;
}

void set_loop_delay(uint32_t delay_ms)
{
    loop_delay = delay_ms;
}

float get_angle_x()
{
    return angle_x;
}

float get_angle_y()
{
    return angle_y;
}

bool isVertical()
{
    return is_vertical;
}