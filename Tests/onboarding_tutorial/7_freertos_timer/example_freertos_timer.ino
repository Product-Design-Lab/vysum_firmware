
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "Adafruit_TinyUSB.h"

TimerHandle_t LongPressTimerHandle;

void longPressTimeUpCallback(TimerHandle_t xTimer)
{

  // _state = LONG_PRESS;
  // longPressFlag = true;
  Serial.println("Long Press");
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  LongPressTimerHandle = xTimerCreate("LongPressTimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, longPressTimeUpCallback);

  xTimerStart(LongPressTimerHandle, 0);
  Serial.println("setup");
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("running");
  delay(1000);
}
