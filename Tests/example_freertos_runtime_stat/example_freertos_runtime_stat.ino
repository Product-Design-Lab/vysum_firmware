#include "Adafruit_TinyUSB.h"

#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t task1handle;
TaskHandle_t task2handle;
TaskHandle_t task3handle;

void cpu_waster()
{
  for(int i=0; i<100000; i++);
}

static void taks1loop(void* pvParameter) {
  while (1) {
    digitalToggle(LED_RED);
    cpu_waster();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void taks2loop(void* pvParameter) {
  while (1) {
    digitalToggle(LED_GREEN);
    cpu_waster();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void taks3loop(void* pvParameter) {
  while (1) {
    digitalToggle(LED_BLUE);
    cpu_waster();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  Serial.begin(9600);
  xTaskCreate(taks1loop, "task1", 1000, NULL, 1, &task1handle);
  xTaskCreate(taks2loop, "task2", 1000, NULL, 1, &task2handle);
  xTaskCreate(taks3loop, "task3", 1000, NULL, 2, &task3handle);
}

void loop() {
  printFreeRTOSRuntimeStats();
  vTaskDelay(pdMS_TO_TICKS(100));
}

void printFreeRTOSRuntimeStats() {
  char buffer[1024];

  // Generate the runtime stats string and store it in the buffer
  vTaskGetRunTimeStats(buffer);

  // Print the stats to the Serial
  Serial.println("FreeRTOS Runtime Stats:");
  Serial.println(buffer);
}