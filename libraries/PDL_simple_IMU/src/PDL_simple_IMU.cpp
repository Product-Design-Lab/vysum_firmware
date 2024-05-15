#include "PDL_Simple_IMU.h"
#include "Adafruit_TinyUSB.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#define DEFAULT_VERTICAL_THRESHOLD_DEGREES 10
#define DEFAULT_TASK_PRIORITY 2
#define DEFAULT_LOOP_DELAY_MS 100
#define TASK_STACK_SIZE 1024
#define DEFAULT_ANGLE_X_OFFSET 90
#define DEFAULT_ANGLE_Y_OFFSET 90

PDL_Simple_IMU::PDL_Simple_IMU()
    : imu(I2C_MODE, 0x6A), angle_x(0), angle_y(0), 
      x_lower_threshold(-DEFAULT_VERTICAL_THRESHOLD_DEGREES), 
      x_upper_threshold(DEFAULT_VERTICAL_THRESHOLD_DEGREES), 
      y_lower_threshold(-DEFAULT_VERTICAL_THRESHOLD_DEGREES), 
      y_upper_threshold(DEFAULT_VERTICAL_THRESHOLD_DEGREES), 
      is_vertical(false), was_vertical(false), 
      loop_delay(DEFAULT_LOOP_DELAY_MS), debug_status(IMU_DEBUG_STATUS_NONE),
      ax_offset(DEFAULT_ANGLE_X_OFFSET), ay_offset(DEFAULT_ANGLE_Y_OFFSET), imu_task_handle(NULL),
      tilted_callback(nullptr), level_callback(nullptr) {}

void PDL_Simple_IMU::init(uint32_t priority) {
    if (imu.begin() != 0) {
        Serial.println("IMU error");
    } else {
        Serial.println("IMU OK!");
    }

    xTaskCreate(imuTask, "IMU_TASK", TASK_STACK_SIZE, this, priority, &imu_task_handle);
}

void PDL_Simple_IMU::imuTask(void *pvParameters) {
    PDL_Simple_IMU *instance = static_cast<PDL_Simple_IMU*>(pvParameters);
    instance->xLastWakeTime = xTaskGetTickCount();
    while (true) {
        instance->processIMUData();
        vTaskDelayUntil(&(instance->xLastWakeTime), pdMS_TO_TICKS(instance->loop_delay));
    }
}

void PDL_Simple_IMU::processIMUData() {
    int16_t ax_raw = imu.readRawAccelX();
    int16_t ay_raw = imu.readRawAccelY();
    int16_t az_raw = imu.readRawAccelZ();

    float ax = ax_filter.add(ax_raw);
    float ay = ay_filter.add(ay_raw);
    float az = az_filter.add(az_raw);

    angle_x = atan2(sqrt(ay * ay + az * az), ax) * 180 / M_PI - ax_offset;
    angle_y = atan2(sqrt(ax * ax + az * az), ay) * 180 / M_PI - ay_offset;

    checkTiltStatus();

    switch (debug_status) {
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
            Serial.printf("angle_x:%6.3f, angle_y:%6.3f, is_vertical:%d\n", angle_x, angle_y, is_vertical);
            break;
        default:
            break;
    }
}

void PDL_Simple_IMU::calibrate() {
    ax_offset = angle_x;
    ay_offset = angle_y;
}

void PDL_Simple_IMU::setDebugStatus(uint8_t status) {
    debug_status = status;
}

void PDL_Simple_IMU::setVerticalThresholds(float x_lower, float x_upper, float y_lower, float y_upper) {
    x_lower_threshold = x_lower;
    x_upper_threshold = x_upper;
    y_lower_threshold = y_lower;
    y_upper_threshold = y_upper;
}

void PDL_Simple_IMU::setLoopDelay(uint32_t delay_ms) {
    loop_delay = delay_ms;
}

void PDL_Simple_IMU::setSampleSize(uint16_t samples) {
    ax_filter.set_samples(samples);
    ay_filter.set_samples(samples);
    az_filter.set_samples(samples);
}

void PDL_Simple_IMU::setTiltedCallback(IMUEventCallback callback) {
    tilted_callback = callback;
}

void PDL_Simple_IMU::setLevelCallback(IMUEventCallback callback) {
    level_callback = callback;
}

float PDL_Simple_IMU::getAngleX() const {
    return angle_x;
}

float PDL_Simple_IMU::getAngleY() const {
    return angle_y;
}

bool PDL_Simple_IMU::isVertical() const {
    return is_vertical;
}

void PDL_Simple_IMU::checkTiltStatus() {
    bool currently_vertical = (angle_x > x_lower_threshold && angle_x < x_upper_threshold &&
                               angle_y > y_lower_threshold && angle_y < y_upper_threshold);

    if (currently_vertical != was_vertical) {
        was_vertical = currently_vertical;
        is_vertical = currently_vertical;
        if (currently_vertical && level_callback) {
            level_callback();
        } else if (!currently_vertical && tilted_callback) {
            tilted_callback();
        }
    }
}
