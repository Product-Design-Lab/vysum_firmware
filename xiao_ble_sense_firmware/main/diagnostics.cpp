// project includes
#include "diagnostics.h"
#include "global_config.h"

// Arduino includes
#include "FreeRTOS.h"         // Include FreeRTOS for task creation
#include "task.h"             // Include task functionality
#include <Adafruit_TinyUSB.h> // for Serial
#include <Arduino.h>

// C/C++ includes
#include <string.h>

#define TAG "DIAG"

TaskHandle_t diagTaskHandle = NULL;

static uint32_t diag_opt = DIAG_OFF;

const char *diag_cmd_off = "d_off";
const char *diag_cmd_motor = "d_motor";
const char *diag_cmd_imu = "d_imu";
const char *diag_cmd_3 = "d_3";
const char *diag_cmd_4 = "d_4";
const char *diag_cmd_5 = "d_5";
const char *diag_cmd_rtos = "d_os";

#define STATS_BUFFER_SIZE 1024

static void printFreeRTOSRuntimeStats() {
  char buffer[1024];

  // Generate the runtime stats string and store it in the buffer
  vTaskGetRunTimeStats(buffer);

  // Print the stats to the Serial
  Serial.println("FreeRTOS Runtime Stats:");
  Serial.println(buffer);
}

static void process_diag_cmd(String cmd)
{
    Serial.println(cmd);
    const char *cmd_cstr = cmd.c_str();
    if (strstr(cmd_cstr, diag_cmd_off) != NULL)
    {
        Serial.println("Diagnostic OFF");
        diag_opt = DIAG_OFF;
    }
    else if (strstr(cmd_cstr, diag_cmd_motor) != NULL)
    {
        diag_opt |= DIAG_MOTOR;
    }
    else if (strstr(cmd_cstr, diag_cmd_imu) != NULL)
    {
        diag_opt |= DIAG_IMU;
    }
    else if (strstr(cmd_cstr, diag_cmd_3) != NULL)
    {
        diag_opt |= DIAG_OPT_3;
    }
    else if (strstr(cmd_cstr, diag_cmd_4) != NULL)
    {
        diag_opt |= DIAG_OPT_4;
    }
    else if (strstr(cmd_cstr, diag_cmd_5) != NULL)
    {
        diag_opt |= DIAG_OPT_5;
    }
    else if (strstr(cmd_cstr, diag_cmd_rtos) != NULL)
    {
        printFreeRTOSRuntimeStats();
    }
    else
    {
        Serial.println("Unknown command");
        // print all cmds
        Serial.printf("DIAG_OFF: %s\n", diag_cmd_off);
        Serial.printf("DIAG_MOTOR: %s\n", diag_cmd_motor);
        Serial.printf("DIAG_IMU: %s\n", diag_cmd_imu);
        Serial.printf("DIAG_OPT_3: %s\n", diag_cmd_3);
        Serial.printf("DIAG_OPT_4: %s\n", diag_cmd_4);
        Serial.printf("DIAG_OPT_5: %s\n", diag_cmd_5);
    }
}

static void diagTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(DIAG_LOOP_DELAY_MS);

    while (1)
    {
        if (Serial.available() > 0)
        {
            String command = Serial.readStringUntil('\n');
            process_diag_cmd(command);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int8_t DIAG_init()
{
    // init once only
    static bool init_done = false;
    if (init_done)
        return 0;

    init_done = true;

    if (!Serial)
    {
        Serial.begin(115200);
    }
    xTaskCreate(diagTask, "diagTask", DAIG_TASK_STACK_SIZE, NULL, DAIG_TASK_PRIORITY, &diagTaskHandle);
    return 0;
}

int8_t DIAG_deinit()
{
    if (diagTaskHandle != NULL)
    {
        vTaskDelete(diagTaskHandle);
        diagTaskHandle = NULL;
    }
    return 0;
}

uint32_t DIAG_get_opt()
{
    return diag_opt;
}
