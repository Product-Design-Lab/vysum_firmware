#include "PDL_Async_Button.h"

namespace PDL_Async_Button
{
static uint8_t _pin;
static uint32_t _debounceTimerMs = 5;
static uint32_t _longPressTimerMs = 1000;
static bool lastPinState = LOW;
static ButtonSignal_e _state = IDLE;
TaskHandle_t buttonTaskHandle;
TimerHandle_t LongPressTimerHandle;
bool longPressFlag = false;

void setPin(uint8_t pin)
{
    PDL_Async_Button::_pin = pin;
}

void setDebounceTimerValue(uint32_t ms)
{
    PDL_Async_Button::_debounceTimerMs = ms;
}
void setLongPressTimerValue(uint32_t ms)
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void longPressTimeUpCallback(TimerHandle_t xTimer)
{
    _state = LONG_PRESS;
    longPressFlag = true;
}

void buttonTask(void *pvParameters)
{

    while (1)
    {
        //enable interrupt
        if (digitalRead(_pin) == LOW)
        {
            lastPinState = LOW;
            attachInterrupt(digitalPinToInterrupt(_pin), pressCallback, RISING);
        }
        else
        {
            lastPinState = HIGH;
            attachInterrupt(digitalPinToInterrupt(_pin), pressCallback, FALLING);
        }

        // wait for interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // debounce
        vTaskDelay(pdMS_TO_TICKS(_debounceTimerMs));
        if (digitalRead(_pin) != lastPinState) // ignore if the pin state has changed
        {
            continue;
        }

        if (lastPinState == HIGH)
        {
            xTimerReset(LongPressTimerHandle, 0);
        }
        else
        {
            if(longPressFlag == false)
            {
                _state = SHORT_PRESS;
            }
            xTimerStop(LongPressTimerHandle, 0);
        }
    }
}

void init()
{
    pinMode(PDL_Async_Button::_pin, INPUT_PULLUP);
    xTimerCreate("LongPressTimer", pdMS_TO_TICKS(_longPressTimerMs), pdFALSE, NULL, longPressTimeUpCallback);
    attachInterrupt(digitalPinToInterrupt(_pin), pressCallback, RISING);
}

} // namespace PDL_Async_Button