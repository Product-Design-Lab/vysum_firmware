#include "DropDetection.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Adafruit_TinyUSB.h"

namespace APDS_DropSensor
{
    constexpr uint8_t DEFAULT_LR_THRESHOLD = 4;
    constexpr uint8_t DEFAULT_LP_THRESHOLD = 6;
    constexpr uint8_t DEFAULT_DOT_THRESHOLD = 2.5;
    constexpr uint32_t TASK_STACK_SIZE = 2048;
    constexpr uint8_t MAX_LOOP_DELAY_MS = 80;

    int _debounce_window_size = 100;
    int crossing_count_trig_threshhold = 2;
    uint8_t debug_flag = DEBUG_INFO;
    int drop_count = 0;

    APDS_Data data;
    APDS_Data::data_crossing_state_t crossing_state = {};

    StaticTask_t xTaskBuffer;
    StackType_t xStack[TASK_STACK_SIZE];
    TaskHandle_t dropDetectionTaskHandle = nullptr;
    eTaskState task_state;

    TickType_t X_FREQUENCY = pdMS_TO_TICKS(20); // Task loop delay 20ms
    TickType_t xLastWakeTime = 0;

    void printDebug()
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
        case DEBUG_CROSSING_STATE_PLOT:
            data.plotCrossingState(crossing_state.state);
            break;
        case DEBUG_FREQ:
            Serial.printf("xLastWakeTime=%d, sample_count=%d\n", xLastWakeTime, data.sample_count);
            break;
        default:
            break;
        }
    }

    void dropDetectionTask(void *pvParameters)
    {
        int _debounce_sample_count = 0;
        if (!APDS.begin())
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("Error initializing APDS-9960 sensor!");
            vTaskDelete(nullptr);
        }

        while (true)
        {
            vTaskDelayUntil(&xLastWakeTime, X_FREQUENCY);

            data.sample_count = APDS.gestureAvailable(data.u.get_raw_u8(), data.d.get_raw_u8(), data.l.get_raw_u8(), data.r.get_raw_u8());
            data.process_all_channel();
            crossing_state.state = data.get_crossing_state().state;

            printDebug();

            _debounce_sample_count += data.sample_count;

            if (_debounce_sample_count >= _debounce_window_size)
            {
                uint8_t dot_crossing_count = crossing_state.u.DOT_CROSS_UPPER_BOUND + crossing_state.u.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.d.DOT_CROSS_UPPER_BOUND + crossing_state.d.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.l.DOT_CROSS_UPPER_BOUND + crossing_state.l.DOT_CROSS_LOWER_BOUND +
                                             crossing_state.r.DOT_CROSS_UPPER_BOUND + crossing_state.r.DOT_CROSS_LOWER_BOUND;

                if (dot_crossing_count > 2)
                {
                    drop_count++;
                    _debounce_sample_count = 0;
                    crossing_state.state = 0;
                }
            }
        }
    }

    void init(uint32_t priority)
    {
        priority = std::clamp(priority, 1u, configMAX_PRIORITIES - 1);

        if (!Serial)
        {
            Serial.begin(115200);
        }

        if (!APDS.begin())
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("Error initializing APDS-9960 sensor!");
            return;
        }
        else
        {
            if (debug_flag == DEBUG_INFO)
                Serial.println("APDS-9960 initialization complete");
        }

        TickType_t end_time = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
        while (xTaskGetTickCount() < end_time)
        {
            vTaskDelayUntil(&xLastWakeTime, X_FREQUENCY);
            data.sample_count = APDS.gestureAvailable(data.u.get_raw_u8(), data.d.get_raw_u8(), data.l.get_raw_u8(), data.r.get_raw_u8());
            printf("sample_count: %d\n", data.sample_count);
            printf("calibProgress: %d/%d\n", xTaskGetTickCount(), end_time);
            data.process_all_channel();
        }

        APDS.end();

        setBoundsLR(DEFAULT_LR_THRESHOLD);
        setBoundsLP(DEFAULT_LP_THRESHOLD);
        setBoundsDot(DEFAULT_DOT_THRESHOLD);

        dropDetectionTaskHandle = xTaskCreateStatic(
            dropDetectionTask,   // Task function
            "dropDetectionTask", // Name of the task
            TASK_STACK_SIZE,     // Stack size
            nullptr,             // Task parameter
            priority,            // Priority
            xStack,              // Stack
            &xTaskBuffer         // Task control block
        );

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
        task_state = eTaskGetState(dropDetectionTaskHandle);
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

    void set_drop_count(int count)
    {
        drop_count = count;
    }

    void setDebug(uint8_t debug)
    {
        if (debug >= DEBUG_NONE && debug < DEBUG_MAX)
        {
            debug_flag = debug;
            switch (debug_flag)
            {
            case DEBUG_INFO:
                Serial.println("Debug mode: INFO");
                break;
            case DEBUG_RAW:
                Serial.println("Debug mode: RAW");
                break;
            case DEBUG_CALIB:
                Serial.println("Debug mode: CALIB");
                break;
            case DEBUG_ZEROING:
                Serial.println("Debug mode: ZEROING");
                break;
            case DEBUG_LOWPASS:
                Serial.println("Debug mode: LOWPASS");
                break;
            case DEBUG_DOT:
                Serial.println("Debug mode: DOT");
                break;
            case DEBUG_LR:
                Serial.println("Debug mode: LR");
                break;
            case DEBUG_CROSSING_STATE_PRINT:
                Serial.println("Debug mode: CROSSING_STATE_PRINT");
                break;
            case DEBUG_CROSSING_STATE_PLOT:
                Serial.println("Debug mode: CROSSING_STATE_PLOT");
                break;
            case DEBUG_FREQ:
                Serial.println("Debug mode: FREQ");
                break;
            default:
                break;
            }
        }
        else
        {
            debug_flag = DEBUG_NONE;
        }
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

    void setBoundsLR(uint8_t bound)
    {
        data.set_bounds_lr(static_cast<int>(bound), static_cast<int>(-bound));
    }

    void setBoundsLP(uint8_t bound)
    {
        data.u.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
        data.d.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
        data.l.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
        data.r.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
    }

    void setBoundsDot(uint8_t bound)
    {
        data.u.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
        data.d.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
        data.l.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
        data.r.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
    }
}
