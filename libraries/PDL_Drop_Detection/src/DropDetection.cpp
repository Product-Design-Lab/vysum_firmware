#include "dropDetection.h"
// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "Adafruit_TinyUSB.h"

namespace APDS_DropSensor
{
    constexpr uint8_t LR_THRESHOLD = 4;
    constexpr uint8_t LP_THRESHOLD = 6;
    constexpr uint8_t DOT_THRESHOLD = 5;
    int _debounce_window_size = 50; // 10*20ms = 200ms
    int crossing_count_trig_threshhold = 2;

    APDS_Data data;
    uint8_t debug_flag = DEBUG_INFO;
    APDS_Data::data_crossing_state_t crossing_state = {};
    int drop_count = 0;
    TaskHandle_t dropDetectionTaskHandle = NULL;
    eTaskState task_state;

    // Caution: Long delay will cause APDS9960 buffer overflow, which will cause the sensor to stop working
    TickType_t X_FREQUENCY = pdMS_TO_TICKS(50); // task loop delay 20ms,
    TickType_t xLastWakeTime = 0;


    static void printDebug()
    {
        switch (debug_flag)
        {
        case DEBUG_RAW:
            data.printRaw();
            break;
        case DEBUG_CALIB:
            data.printCalib();
            break;
        case DEBUG_ZEROING:
            data.printRaw_i32();
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
            data.printCrossingState(crossing_state.state);
            break;
        case DEBUG_CORSSING_STATE_PLOT:
            data.plotCrossingState(crossing_state.state);
            break;
        case DEBUG_FREQ:
            Serial.printf("xLastWakeTime=%d, sample_count=%d\n", xLastWakeTime, data.sample_count);
        default:
            break;
        }
    }

    static void dropDetectionTask(void *pvParameters)
    {

        int _debounce_sample_count = 0;

        while (1)
        {
            vTaskDelayUntil(&xLastWakeTime, X_FREQUENCY);

            data.sample_count = APDS.gestureAvailable(data.u.get_raw_u8(), data.d.get_raw_u8(), data.l.get_raw_u8(), data.r.get_raw_u8());
            data.process_all_channel();
            crossing_state.state = data.get_crossing_state().state;

            printDebug();

            _debounce_sample_count += data.sample_count;

            if (_debounce_sample_count >= _debounce_window_size)
            {
                // extract crossing state
                // uint8_t dot_crossing_count = crossing_state.l.DOT_CROSS_UPPER_BOUND + crossing_state.l.DOT_CROSS_LOWER_BOUND +
                //                              crossing_state.r.DOT_CROSS_UPPER_BOUND + crossing_state.r.DOT_CROSS_LOWER_BOUND;

                uint8_t dot_crossing_count = crossing_state.u.DOT_CROSS_UPPER_BOUND + crossing_state.u.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.d.DOT_CROSS_UPPER_BOUND + crossing_state.d.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.l.DOT_CROSS_UPPER_BOUND + crossing_state.l.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.r.DOT_CROSS_UPPER_BOUND + crossing_state.r.DOT_CROSS_LOWER_BOUND;

                uint8_t lp_crossing_count = crossing_state.u.LP_CROSS_UPPER_BOUND + crossing_state.u.LP_CROSS_LOWER_BOUND +
                                            crossing_state.d.LP_CROSS_UPPER_BOUND + crossing_state.d.LP_CROSS_LOWER_BOUND +
                                            crossing_state.l.LP_CROSS_UPPER_BOUND + crossing_state.l.LP_CROSS_LOWER_BOUND +
                                            crossing_state.r.LP_CROSS_UPPER_BOUND + crossing_state.r.LP_CROSS_LOWER_BOUND;

                uint8_t lr_crossing_count = crossing_state.lr.RISE_OVER_UPPER_BOUND + crossing_state.lr.FALL_BELOW_UPPER_BOUND +
                                            crossing_state.lr.RISE_OVER_LOWER_BOUND + crossing_state.lr.FALL_BELOW_LOWER_BOUND;

                uint8_t total_crossing_count = __builtin_popcount((uint32_t)crossing_state.state);

                if (dot_crossing_count > 2)
                {
                    drop_count++;
                    _debounce_sample_count = 0;
                    crossing_state.state = 0;
                    Serial.printf("crossing_count: dot:%d, lp:%d, lr:%d, total:%d, drop_count:%d\n", dot_crossing_count, lp_crossing_count, lr_crossing_count, total_crossing_count, drop_count);
                }

                // crossing_state.state = 0;
            }
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
            data.sample_count = APDS.gestureAvailable(data.u.get_raw_u8(), data.d.get_raw_u8(), data.l.get_raw_u8(), data.r.get_raw_u8());
            if (debug_flag == DEBUG_CALIB)
                data.printRaw();
            data.process_all_channel();
            delay(10);
        }

        setBoundsLR(LR_THRESHOLD);
        setBoundsLP(LP_THRESHOLD);
        setBoundsDot(DOT_THRESHOLD);

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
        {
            debug_flag = debug;
            Serial.printf("Set debug flag: %d\n", debug);
        }
        else
            debug_flag = DEBUG_NONE;
    }

    void setCrossCountTrigThreshold(int threshold)
    {
        if (threshold > 0 && threshold < 8)
            crossing_count_trig_threshhold = threshold;
    }

    void setDebouceWindowSize(uint8_t count)
    {
        _debounce_window_size = count;
    }

    void setLoopDelayMs(uint32_t ms)
    {
        X_FREQUENCY = pdMS_TO_TICKS(ms);
    }

    void setBoundsLR(const uint8_t bound)
    {
        data.set_bounds_lr((int)bound, (int)(-bound));
    }

    void setBoundsLP(const uint8_t bound)
    {
        data.u.set_bounds_lp((int)bound, (int)(-bound));
        data.d.set_bounds_lp((int)bound, (int)(-bound));
        data.l.set_bounds_lp((int)bound, (int)(-bound));
        data.r.set_bounds_lp((int)bound, (int)(-bound));
    }

    void setBoundsDot(const uint8_t bound)
    {
        data.u.set_bounds_dot((int)bound, (int)(-bound));
        data.d.set_bounds_dot((int)bound, (int)(-bound));
        data.l.set_bounds_dot((int)bound, (int)(-bound));
        data.r.set_bounds_dot((int)bound, (int)(-bound));
    }

}
