#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include "diagnostics.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pins.h"
String mystr="000";

TaskHandle_t sampleTaskHandle;
TaskHandle_t sampleTaskHandle2;

void sampleTask(void *pvParameters)
{
  while (1)
  {
    digitalToggle(LED_GREEN); // Toggle LED
    vTaskDelay(500);          // wait for 500ms
  }
}

void sampleTask2(void *pvParameters)
{
  int i=0;
  while (1)
  {
    digitalToggle(LED_RED); // Toggle LED
    serialPrintf("Hello World%d\n", ++i);
    vTaskDelay(1000);       // wait for 1000ms
  }
}

void setup()
{
  DIAG_init();
  xTaskCreate(sampleTask, "sampleTask", 128, NULL, 1, &sampleTaskHandle);
  xTaskCreate(sampleTask2, "sampleTask", 128, NULL, 1, &sampleTaskHandle2);
}

// main task
void loop()
{
}
