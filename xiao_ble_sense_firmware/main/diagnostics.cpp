// project includes
#include "diagnostics.h"

// Arduino includes
#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include "FreeRTOS.h"         // Include FreeRTOS for task creation
#include "task.h"             // Include task functionality

// C/C++ includes
#include <string.h>

#define TAG "DIAG"

TaskHandle_t diagTaskHandle = NULL;

static uint32_t diag_opt = DIAG_OFF;

const String diag_cmd_1 = "d_1";
const String diag_cmd_2 = "d_2";
const String diag_cmd_3 = "d_3";
const String diag_cmd_4 = "d_4";
const String diag_cmd_5 = "d_5";

static void process_diag_cmd(String cmd)
{
    Serial.println(cmd);

    if (cmd.indexOf("d_off") != -1)
    {
        Serial.println("Diagnostic OFF");
        diag_opt = DIAG_OFF;
    }
    else if (cmd.indexOf(diag_cmd_1) != -1)
    {
        diag_opt |= DIAG_OPT_1;
    }
    else if (cmd.indexOf(diag_cmd_2) != -1)
    {
        diag_opt |= DIAG_OPT_2;
    }
    else if (cmd.indexOf(diag_cmd_3) != -1)
    {
        diag_opt |= DIAG_OPT_3;
    }
    else if (cmd.indexOf(diag_cmd_4) != -1)
    {
        diag_opt |= DIAG_OPT_4;
    }
    else if (cmd.indexOf(diag_cmd_5) != -1)
    {
        diag_opt |= DIAG_OPT_5;
    }
    else
    {
        Serial.println("Unknown command");
    }
}

static void diagTask(void *pvParameters)
{
    while (1)
    {
        if (Serial.available() > 0)
        {
            String command = Serial.readStringUntil('\n');
            process_diag_cmd(command);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent task from consuming too much CPU time
    }
}

int8_t DIAG_init()
{
    if (!Serial)
    {
        Serial.begin(115200);
        while (!Serial)
        {
            delay(10);
        }
    }

    xTaskCreate(diagTask, "Diagnostic Task", 1024, NULL, 1, &diagTaskHandle);
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

void serialPrintf(const char *format, ...)
{
    va_list args;

    // Start processing variadic arguments
    va_start(args, format);

    // Calculate required buffer length
    int length = vsnprintf(nullptr, 0, format, args) + 1; // +1 for null terminator
    va_end(args);

    // Dynamically allocate buffer
    char *buffer = new char[length];

    if (buffer != nullptr)
    {
        // Restart processing variadic arguments
        va_start(args, format);
        vsnprintf(buffer, length, format, args);
        va_end(args);

        // Print to Serial
        Serial.print(buffer);

        // Free the allocated memory
        delete[] buffer;
    }
    else
    {
        // Memory allocation failed
        Serial.println(F("Memory allocation failed for serialPrintf"));
    }
}
