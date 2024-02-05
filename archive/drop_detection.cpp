#include "drop_detection.hpp"
#include "global_config.h"
#include "diagnostics.h"

#include <Wire.h>

#include "SparkFun_VCNL4040_Arduino_Library.h"

#include "FreeRTOS.h"
#include "task.h"

namespace VCNL
{
    static VCNL4040 proximitySensor;
    static TaskHandle_t drop_detection_task_handle;

    static uint16_t drop_count = 0;
    static uint16_t proxValue = 0;
    static uint16_t ambientValue = 0;
    static uint16_t whiteValue = 0;

    static bool detection_enabled = false;

    static void run_diagnostics()
    {
        // print out the sensor readings
        Serial.printf("proximity:%d, ambient:%d, white:%d, drop_count:%d\n", proxValue, ambientValue, whiteValue, drop_count);
    }

    static void run_drop_detection()
    {
        if (proxValue < 100)
        {
            drop_count++;
        }
    }

    static void drop_detection_task(void *pvParameters)
    {
        TickType_t last_wake_time = xTaskGetTickCount();
        TickType_t xFrequency = pdMS_TO_TICKS(DROP_LOOP_DELAY_MS);

        while (true)
        {
            proxValue = proximitySensor.getProximity();
            ambientValue = proximitySensor.getAmbient();
            whiteValue = proximitySensor.getWhite();

            if (detection_enabled)
            {
                run_drop_detection();
            }

            if ((bool)(DIAG::get_opt() & DIAG::D_DROP) == true)
            {
                run_diagnostics();
            }
            vTaskDelayUntil(&last_wake_time, xFrequency);
        }
    }

    void init()
    {
        Wire.begin(); // Join i2c bus

        proximitySensor.begin(); // Initialize the sensor

        // Turn on everything
        proximitySensor.powerOnProximity();
        proximitySensor.powerOnAmbient();
        proximitySensor.enableWhiteChannel();

        xTaskCreate(drop_detection_task, "dropTask", DROP_TASK_STACK_SIZE, NULL, DROP_TASK_PRIORITY, &drop_detection_task_handle);
    }

    void deinit()
    {
        if (drop_detection_task_handle != NULL)
        {
            vTaskDelete(drop_detection_task_handle);
            drop_detection_task_handle = NULL;
        }
    }

    void startDetection()
    {
        detection_enabled = true;
    }

    void stopDetection()
    {
        detection_enabled = false;
    }

    int get_drop_count()
    {
        return drop_count;
    }

    void set_drop_count(const int count)
    {
        drop_count = count;
    }
}