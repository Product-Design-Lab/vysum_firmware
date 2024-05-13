#pragma once

#include "LSM6DS3.h"

enum IMU_DEBUG_STATUS {
    IMU_DEBUG_STATUS_NONE = 0,
    IMU_DEBUG_STATUS_RAW = 1,
    IMU_DEBUG_STATUS_FILTERED = 2,
    IMU_DEBUG_STATUS_ANGLE = 3,
    IMU_DEBUG_THRESHOLD = 4,
    IMU_DEBUG_MAX
};


void init_imu(uint32_t priority = 2);
void calibrate_imu();
float get_angle_x();
float get_angle_y();
bool isVertical();
void set_debug_status(uint8_t status);
void set_vertical_threshold(float threshold);
void set_loop_delay(uint32_t delay_ms);