#include "pose_checker.hpp"
#include "global_config.h"

#include "FreeRTOS.h"
#include "LSM6DS3.h"
#include "MovingAverageFloat.h"
#include "task.h"
#include <Arduino.h>

#define VERTICAL_THRESHOLD_DEGREES 10

LSM6DS3 myIMU(I2C_MODE, 0x6A);
TaskHandle_t poseCheckerTaskHandle;

static float alpha;
static bool is_vertical;

static void poseCheckerTask(void *pvParameters)
{
    float ax, ay, az, gx, gy, gz;
    float rollAcc = 0, pitchAcc = 0;
    float rollGyro = 0, pitchGyro = 0;
    float roll = 0, pitch = 0;
    float rollAve = 0, pitchAve = 0;

    MovingAverageFloat<32> filter;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_LOOP_DELAY_MS);
    
    while (1)
    {
        // Read accelerometer data
        ax = myIMU.readFloatAccelX();
        ay = myIMU.readFloatAccelY();
        az = myIMU.readFloatAccelZ();

        // Read gyroscope data
        gx = myIMU.readFloatGyroX();
        gy = myIMU.readFloatGyroY();
        gz = myIMU.readFloatGyroZ();

        // Convert gyroscope data to degrees/s
        gx = gx * 180 / M_PI;
        gy = gy * 180 / M_PI;
        gz = gz * 180 / M_PI;

        // Integrate gyroscope data
        rollGyro += gx * 0.001 * IMU_LOOP_DELAY_MS; // Assuming loop runs at 100Hz
        pitchGyro += gy * 0.001 * IMU_LOOP_DELAY_MS;

        // Compute accelerometer angles
        rollAcc = atan2(ay, az) * 180 / M_PI;
        pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

        // Apply complementary filter
        float roll = rollGyro * alpha + rollAcc * (1 - alpha);
        float pitch = pitchGyro * alpha + pitchAcc * (1 - alpha);

        // Apply moving average filter
        rollAve = filter.add(roll);
        pitchAve = filter.add(pitch);

        // Check if the sensor is held vertically
        if (abs(rollAve) < VERTICAL_THRESHOLD_DEGREES && abs(pitchAve) < VERTICAL_THRESHOLD_DEGREES)
        {
            is_vertical = true;
        }
        else
        {
            is_vertical = false;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

namespace IMU
{

void init(float alphaValue)
{
    // ensure init is called only once
    static bool is_init = false;
    if (is_init)
    {
        return;
    }
    is_init = true;

    alpha = alphaValue;
    myIMU.begin();

    xTaskCreate(poseCheckerTask, "Pose Checker Task", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY,
                &poseCheckerTaskHandle);
}

void deinit()
{
    if (poseCheckerTaskHandle != NULL)
    {
        vTaskDelete(poseCheckerTaskHandle);
        poseCheckerTaskHandle = NULL;
    }
}

bool isVertical()
{
    return is_vertical;
}
} // namespace IMU