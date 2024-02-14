#include "dropDetection.h"
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "Adafruit_TinyUSB.h"

namespace APDS_DropSensor
{
    APDS_Data data;
    bool pauseFlag = false;
    uint32_t crossing_state = 0;
    int dropCount = 0;
    TaskHandle_t dropDetectionTaskHandle = NULL;

    static void dropDetectionTask(void *pvParameters)
    {
        if (!APDS.begin())
        {
            Serial.println("Error initializing APDS-9960 sensor!");
        }
        else
        {
            Serial.println("APDS-9960 initialization complete");
        }

        for (int i = 0; i < 128; i++)
        {
            data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
            data.copy_buffer();
            // data.printRaw();
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

        TickType_t xLastWakeTime = 0;
        const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms
        Serial.println(xFrequency);
        int wait_count = 0;
        const int wait_cycles = 10; // 10*20ms = 200ms
        while (1)
        {
            data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
            data.process();
            crossing_state |= data.get_crossing_state();
            // Serial.printf("value=%d\n", crossing_state);

            // data.printRaw();
            // data.printCalib();
            // data.printRaw_i16();
            // data.printLP();
            // data.printDot();
            // data.printLR();
            // data.printCrossingState();
            // data.plotCrossingState();


            if (pauseFlag || ++wait_count < wait_cycles)
            {
                //prevent conseqtive drops
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

                if (count > 4)
                {
                    dropCount++;
                    // Serial.printf("dropCount=%d\n", dropCount);
                    // data.printCrossingState(crossing_state);
                    // Serial.println(millis());
                    wait_count = 0;
                }
                crossing_state = 0;
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void init()
    {
        if (!Serial)
        {
            Serial.begin(115200);
            while (!Serial)
                ;
        }
        xTaskCreate(dropDetectionTask, "dropDetectionTask", 2048, NULL, 1, &dropDetectionTaskHandle);
    }

    void deinit()
    {
        vTaskDelete(dropDetectionTaskHandle);
        APDS.end();
    }

    void resume()
    {
        pauseFlag = false;
    }
    void pause()
    {
        pauseFlag = true;
    }
    int get_drop_count()
    {
        return dropCount;
    }
    void set_drop_count(const int count)
    {
        dropCount = count;
    }

}