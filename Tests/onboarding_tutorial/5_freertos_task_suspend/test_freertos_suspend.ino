#include <Arduino.h>

#include "FreeRTOS.h"
#include "task.h"

#include "Adafruit_TinyUSB.h"

TaskHandle_t testHandle;

void testTask(void *pvParameters)
{
    int count = 0;
    while (1)
    {
        Serial.printf("Hello from testTask, count=%d\n", count++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    xTaskCreate(testTask, "testTask", 1024, NULL, 1, &testHandle);
}

void loop()
{
    if(Serial.available())
    {
        char c = Serial.read();
        Serial.println(c);
        if(c == 'p')
        {
            vTaskSuspend(testHandle);
            Serial.println("testTask suspended");
        }
        else if(c == 'r')
        {
            vTaskResume(testHandle);
            Serial.println("testTask resumed");
        }
    }

    Serial.println("Hello from loop");
    delay(1000);
}