#include "dropDetection.h"
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "Adafruit_TinyUSB.h"

namespace APDS_DropSensor
{
    APDS_Data data;
    uint8_t debug_flag = DEBUG_NONE;
    uint32_t crossing_state = 0;
    int drop_count = 0;
    TaskHandle_t dropDetectionTaskHandle = NULL;
    eTaskState task_state;

    // Caution: Long delay will cause APDS9960 buffer overflow, which will cause the sensor to stop working
    const TickType_t X_FREQUENCY = pdMS_TO_TICKS(20); // task loop delay 20ms,
    const int wait_cycles = 10;                       // 10*20ms = 200ms
    const int DETECTION_THRESHOLD = 4;

    static void dropDetectionTask(void *pvParameters)
    {
        TickType_t xLastWakeTime = 0;
        int wait_count = 0;

        while (1)
        {

            APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
            data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
            data.process();
            crossing_state |= data.get_crossing_state();

            switch (debug_flag)
            {
            case DEBUG_RAW:
                data.printRaw();
                break;
            case DEBUG_CALIB:
                data.printCalib();
                break;
            case DEBUG_ZEROING:
                data.printRaw_i16();
                break;
            case DEBUG_LOWPASS:
                data.printLP();
                break;
            case DEBUG_DOT:
                data.printDot();
                break;
            case DEBUG_LR:
                data.printLR();
                break;
            case DEBUG_CROSSING_STATE_PRINT:
                data.printCrossingState(crossing_state);
                break;
            case DEBUG_CORSSING_STATE_PLOT:
                data.plotCrossingState(crossing_state);
                break;
            default:
                break;
            }

            if (++wait_count < wait_cycles)
            {
                // prevent conseqtive drops
                crossing_state = 0;
            }
            else
            {
                // get the number of bits set
                uint32_t count = 0;
                for (int i = 0; i < 32; i++)
                {
                    count += (crossing_state >> i) & 1;
                }

                if (count > DETECTION_THRESHOLD)
                {
                    drop_count++;
                    // Serial.printf("drop_count=%d\n", drop_count);
                    // data.printCrossingState(crossing_state);
                    // Serial.println(millis());
                    wait_count = 0;
                }
                crossing_state = 0;
            }
            if (debug_flag == DEBUG_FREQ)
                Serial.printf("xLastWakeTime=%d\n", xLastWakeTime);
            vTaskDelayUntil(&xLastWakeTime, X_FREQUENCY);
        }
    }

    void init(uint32_t priority)
    {
        if (priority < 1)
        {
            priority = 1;
        }
        else if (priority > configMAX_PRIORITIES - 1)
        {
            priority = configMAX_PRIORITIES - 1;
        }
        if (!Serial)
        {
            Serial.begin(115200);
            while (!Serial)
                ;
        }

        if (!APDS.begin())
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("Error initializing APDS-9960 sensor!");
        }
        else
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("APDS-9960 initialization complete");
        }

        for (int i = 0; i < 128; i++)
        {
            data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
            data.copy_buffer();
            if (debug_flag == DEBUG_CALIB)
                data.printRaw();
            data.calib(false);
            delay(10);
        }

        data.set_bounds_lr(6, -4);

        data.u.set_bounds_lp(4, -4);
        data.d.set_bounds_lp(4, -4);
        data.l.set_bounds_lp(4, -4);
        data.r.set_bounds_lp(4, -4);

        data.u.set_bounds_dot(4, -4);
        data.d.set_bounds_dot(4, -4);
        data.l.set_bounds_dot(4, -4);
        data.r.set_bounds_dot(4, -4);

        xTaskCreate(dropDetectionTask, "dropDetectionTask", 2048, NULL, priority, &dropDetectionTaskHandle);
        if (debug_flag == DEBUG_INFO)
            Serial.println("Drop Detection Task Started");
    }

    void deinit()
    {
        vTaskDelete(dropDetectionTaskHandle);
        if (debug_flag == DEBUG_INFO)
            Serial.println("Drop Detection Task Stopped");

        APDS.end();
        if (debug_flag == DEBUG_INFO)
            Serial.println("APDS-9960 sensor deinitialized");
    }

    void resume()
    {
        // check if the task is suspended
        task_state = eTaskGetState(dropDetectionTaskHandle); // INCLUDE_eTaskGetState must be set to 1 in FreeRTOSConfig.h for eTaskGetState() to be available.
        if (task_state == eSuspended)
        {
            APDS.begin();
            vTaskResume(dropDetectionTaskHandle);
            if (debug_flag == DEBUG_INFO)
                Serial.println("Drop Detection Task Resumed");
        }
        else
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("Drop Detection Task is not suspended");
        }
    }

    void pause()
    {
        task_state = eTaskGetState(dropDetectionTaskHandle);
        if (task_state != eSuspended)
        {
            APDS.end();
            vTaskSuspend(dropDetectionTaskHandle);
            if (debug_flag == DEBUG_INFO)
                Serial.println("Drop Detection Task suspended");
        }
        else
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("Drop Detection Task is already suspended");
        }
    }

    int get_drop_count()
    {
        return drop_count;
    }

    void set_drop_count(const int count)
    {
        drop_count = count;
    }

    void setDebug(uint8_t debug)
    {
        if (debug >= DEBUG_NONE && debug < DEBUG_MAX)
            debug_flag = debug;
        else
            debug_flag = DEBUG_NONE;
    }
}