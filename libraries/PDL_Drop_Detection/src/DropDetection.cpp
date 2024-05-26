#include "DropDetection.h"
#include "Adafruit_TinyUSB.h"

WaterDropSensor::WaterDropSensor()
    : debounceWindowSize(DEFAULT_DEBOUNCE_WINDOW_SIZE),
      crossingCountTrigThreshhold(DEFAULT_CROSS_COUNT_TRIG_THRESHOLD),
      debugFlag(DEBUG_INFO),
      dropCount(0),
      dropDetectionTaskHandle(nullptr),
      X_FREQUENCY(pdMS_TO_TICKS(20)),
      xLastWakeTime(0)
{
}

WaterDropSensor::~WaterDropSensor()
{
    deinit();
}

void WaterDropSensor::init(uint32_t priority)
{
    if (dropDetectionTaskHandle == nullptr)
    {
        xLastWakeTime = xTaskGetTickCount();
        dropDetectionTaskHandle = xTaskCreateStatic(
            dropDetectionTask, "DropDetectionTask", TASK_STACK_SIZE, this, priority, xStack, &xTaskBuffer);
    }
}

void WaterDropSensor::deinit()
{
    if (dropDetectionTaskHandle != nullptr)
    {
        vTaskDelete(dropDetectionTaskHandle);
        dropDetectionTaskHandle = nullptr;
    }
}

void WaterDropSensor::resume()
{
    if (dropDetectionTaskHandle != nullptr)
    {
        vTaskResume(dropDetectionTaskHandle);
    }
}

void WaterDropSensor::pause()
{
    if (dropDetectionTaskHandle != nullptr)
    {
        vTaskSuspend(dropDetectionTaskHandle);
    }
}

int WaterDropSensor::getDropCount() const
{
    return dropCount;
}

void WaterDropSensor::setDropCount(int count)
{
    dropCount = count;
}

void WaterDropSensor::setDebug(uint8_t debug)
{
    if (debug >= DEBUG_NONE && debug < DEBUG_MAX)
    {
        debugFlag = debug;
        switch (debugFlag)
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
        debugFlag = DEBUG_NONE;
    }
}

void WaterDropSensor::setCrossCountTrigThreshold(int threshold)
{
    crossingCountTrigThreshhold = threshold;
}

void WaterDropSensor::setDebouceWindowSize(uint8_t count)
{
    debounceWindowSize = count;
}

void WaterDropSensor::setLoopDelayMs(uint32_t ms)
{
    X_FREQUENCY = pdMS_TO_TICKS(ms);
}

void WaterDropSensor::setBoundsLR(uint8_t bound)
{
    data.set_bounds_lr(static_cast<int>(bound), static_cast<int>(-bound));
}

void WaterDropSensor::setBoundsLP(uint8_t bound)
{
    data.u.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
    data.d.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
    data.l.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
    data.r.set_bounds_lp(static_cast<int>(bound), static_cast<int>(-bound));
}

void WaterDropSensor::setBoundsDot(uint8_t bound)
{
    data.u.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
    data.d.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
    data.l.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
    data.r.set_bounds_dot(static_cast<int>(bound), static_cast<int>(-bound));
}

void WaterDropSensor::printDebug()
{
    switch (debugFlag)
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

void WaterDropSensor::dropDetectionTask(void *pvParameters)
{
    WaterDropSensor *sensor = static_cast<WaterDropSensor *>(pvParameters);
    int _debounce_sample_count = 0;
    if (!APDS.begin())
    {
        if (sensor->debugFlag == DEBUG_INFO)
        {
            Serial.println("Error initializing APDS-9960 sensor!");
        }
        vTaskDelete(nullptr);
        return;
    }

    while (true)
    {
        sensor->runMainTaskLogic();
    }
}

void WaterDropSensor::runMainTaskLogic()
{
    static int _debounce_sample_count = 0;

    vTaskDelayUntil(&xLastWakeTime, X_FREQUENCY);

    data.sample_count = APDS.gestureAvailable(data.u.get_raw_u8(), data.d.get_raw_u8(), data.l.get_raw_u8(), data.r.get_raw_u8());
    data.process_all_channel();
    crossing_state.state = data.get_crossing_state().state;

    printDebug();

    _debounce_sample_count += data.sample_count;

    if (_debounce_sample_count >= debounceWindowSize)
    {
        uint8_t dot_crossing_count = crossing_state.u.DOT_CROSS_UPPER_BOUND + crossing_state.u.DOT_CROSS_LOWER_BOUND +
                                     crossing_state.d.DOT_CROSS_UPPER_BOUND + crossing_state.d.DOT_CROSS_LOWER_BOUND +
                                     crossing_state.l.DOT_CROSS_UPPER_BOUND + crossing_state.l.DOT_CROSS_LOWER_BOUND +
                                     crossing_state.r.DOT_CROSS_UPPER_BOUND + crossing_state.r.DOT_CROSS_LOWER_BOUND;

        if (dot_crossing_count > 2)
        {
            dropCount++;
            _debounce_sample_count = 0;
            crossing_state.state = 0;
        }
    }
}
