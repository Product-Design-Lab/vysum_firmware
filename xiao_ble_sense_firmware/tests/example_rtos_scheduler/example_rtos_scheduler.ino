#include <Adafruit_TinyUSB.h> // for Serial
#include <Arduino.h>

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t task1Handle;
TaskHandle_t task2Handle;

void setup()
{
    xTaskCreate(task1loop,     /* Task function. */
                "Task1",       /* String with name of task. */
                1000,          /* Stack size in bytes. */
                NULL,          /* Parameter passed as input of the task */
                1,             /* Priority of the task. */
                &task1Handle); /* Task handle. */

    xTaskCreate(task2loop,     /* Task function. */
                "Task2",       /* String with name of task. */
                1000,          /* Stack size in bytes. */
                NULL,          /* Parameter passed as input of the task */
                1,             /* Priority of the task. */
                &task2Handle); /* Task handle. */
}

void loop()
{
    digitalToggle(LED_RED); // Toggle LED
    delay(1000);            // wait for a second
}

void task1loop(void *pvParameters)
{
    while (1)
    {
        Serial.println("Task 1");
        delay(1000);
    }
}

void task2loop(void *pvParameters)
{
    while (1)
    {
        Serial.println("Task 2");
        delay(1000);
    }
}