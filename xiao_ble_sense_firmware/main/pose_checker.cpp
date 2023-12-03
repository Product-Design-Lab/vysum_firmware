#include "pose_checker.hpp"
#include "global_config.h"
#include "diagnostics.h"

#include "FreeRTOS.h"
#include "LSM6DS3.h"
#include "MovingAverageFloat.h"
#include "task.h"
#include <Arduino.h>

#define VERTICAL_THRESHOLD_DEGREES 10

namespace IMU
{

  static LSM6DS3 myIMU(I2C_MODE, 0x6A);
  static TaskHandle_t imuTaskHandle;
  static bool is_vertical;

  static void imuTask(void *pvParameters)
  {
    float ax, ay, az;
    float angle, angleAve;
    MovingAverageFloat<16> angle_filter;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_LOOP_DELAY_MS);

    while (1)
    {
      // Read accelerometer data
      ax = myIMU.readFloatAccelX();
      ay = myIMU.readFloatAccelY();
      az = myIMU.readFloatAccelZ();

      // Compute accelerometer angles, ax point donwwards
      angle = atan2(sqrt(ay * ay + az * az), ax) * 180 / M_PI;

      // Apply moving average angle_filter
      angleAve = angle_filter.add(angle);

      // Check if the sensor is held vertically
      is_vertical = (abs(angleAve) < VERTICAL_THRESHOLD_DEGREES);

      if (DIAG::get_opt() == DIAG::D_IMU)
      {
        Serial.printf("ax:%f, ay:%f, az:%f, angle:%f, ang_ave:%f, verticle:%d\n", ax, ay, az, angle, angleAve, is_vertical);
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      // delay(10);
    }
  }

  void init()
  {
    // ensure init is called only once
    static bool is_init = false;
    if (is_init)
    {
      return;
    }
    is_init = true;

    myIMU.begin();

    xTaskCreate(imuTask, "imuTask", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY,
                &imuTaskHandle);
  }

  void deinit()
  {
    if (imuTaskHandle != NULL)
    {
      vTaskDelete(imuTaskHandle);
      imuTaskHandle = NULL;
    }
  }

  bool isVertical()
  {
    return is_vertical;
  }

} // namespace IMU