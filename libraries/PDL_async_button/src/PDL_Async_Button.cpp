#include "PDL_Async_Button.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

namespace PDL_Async_Button
{
    static uint8_t _pin;
    static uint32_t _debounceTimerMs = 10;
    static uint32_t _longPressTimerMs = 1000;
    static bool lastPinState = LOW;
    static uint8_t _state = IDLE;
    TaskHandle_t buttonTaskHandle;
    TimerHandle_t LongPressTimerHandle;
    bool longPressFlag = false;

    void setPin(uint8_t pin)
    {
        PDL_Async_Button::_pin = pin;
    }

    void setDebounceTime(uint32_t ms)
    {
        PDL_Async_Button::_debounceTimerMs = ms;
    }
    void setLongPressTime(uint32_t ms)
    {
        PDL_Async_Button::_longPressTimerMs = ms;
    }
    uint8_t getState() // this will clear state
    {
        uint8_t state_temp = _state;
        _state = IDLE;
        return state_temp;
    }

    void pressCallback()
    {
        detachInterrupt(digitalPinToInterrupt(_pin));
        lastPinState = (bool)digitalRead(_pin);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        // Serial.printf("ISR: Button state: %d\n", lastPinState);
    }

    void longPressTimeUpCallback(TimerHandle_t xTimer)
    {
        _state = LONG_PRESS;
        longPressFlag = true;
        // Serial.println("Long Press");
    }

    void buttonTask(void *pvParameters)
    {

        while (1)
        {
            // enable interrupt
            attachInterrupt(digitalPinToInterrupt(_pin), pressCallback, CHANGE);

            // Serial.println("Waiting for interrupt");

            // wait for interrupt
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Serial.println("task notified");

            // debounce
            vTaskDelay(pdMS_TO_TICKS(_debounceTimerMs));
            if (digitalRead(_pin) != lastPinState) // ignore if the pin state has changed
            {
                // Serial.printf("Debounce failed: %d, %d\n", digitalRead(_pin), lastPinState);
                continue;
            }

            // Serial.println("Debounced");

            if (lastPinState == LOW)//pressed
            {
                longPressFlag = false;
                xTimerReset(LongPressTimerHandle, 0);
            }
            else // released
            {
                if (longPressFlag == false)
                {
                    _state = SHORT_PRESS;
                    // Serial.println("Short Press");
                }
                xTimerStop(LongPressTimerHandle, 0);
            }
        }
    }

    void init()
    {
        pinMode(PDL_Async_Button::_pin, INPUT_PULLUP);
        LongPressTimerHandle = xTimerCreate("LongPressTimer", pdMS_TO_TICKS(_longPressTimerMs), pdFALSE, NULL, longPressTimeUpCallback);
        xTaskCreate(buttonTask, "buttonTask", 1024, NULL, 1, &buttonTaskHandle);
    }

} // namespace PDL_Async_Button