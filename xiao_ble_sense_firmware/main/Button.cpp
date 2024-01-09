#include "Button.hpp"
#include <Arduino.h>
#include "diagnostics.h"
#include "pins.h"
#include "global_config.h"

#include <FreeRTOS.h>
#include <task.h>

namespace Button
{
    static uint32_t ulNotificationValue = 0;

    static TaskHandle_t buttonTaskHandle = NULL;

    // Internal state of each button
    static ButtonState_t buttonWhiteState = {0, 0, false};
    static ButtonState_t buttonBlueState = {0, 0, false};

    void run_diagnostics()
    {
        printButtonState(getButtonWhiteState());
        printButtonState(getButtonBlueState());
    }

    static void buttonTask(void *pvParameters)
    {
        const TickType_t wait_tick = pdMS_TO_TICKS(BUTTON_NOTIFY_WAIT_MS);

        while (1)
        {
            ulTaskNotifyTake(pdTRUE, wait_tick);

            run_diagnostics();
        }
    }

    static inline void notifyTask(void)
    {
        // if (DIAG::get_opt() & DIAG::D_BUTTON)

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    static void buttonWhiteISR()
    {
        buttonWhiteState._is_pressed = (bool)(digitalRead(PIN_BUTTON_WHITE) == LOW);
        detachInterrupt(digitalPinToInterrupt(PIN_BUTTON_WHITE));

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    static void buttonBlueISR()
    {
        detachInterrupt(digitalPinToInterrupt(PIN_BUTTON_BLUE));
        buttonBlueState._is_pressed = (bool)(digitalRead(PIN_BUTTON_BLUE) == LOW);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void init()
    {

        pinMode(PIN_BUTTON_WHITE, INPUT_PULLUP);
        pinMode(PIN_BUTTON_BLUE, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_WHITE), buttonWhiteISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_BLUE), buttonBlueISR, RISING);

        xTaskCreate(buttonTask, "ButTask", 1000, NULL, 1, &buttonTaskHandle);
    }

    ButtonState_t getButtonWhiteState()
    {
        return buttonWhiteState;
    }

    ButtonState_t getButtonBlueState()
    {
        return buttonBlueState;
    }

    void printButtonState(const ButtonState_t buttonState)
    {
        Serial.printf("Button state: pressCount=%d, releaseCount=%d, isPressed=%d\n",
                      buttonState.pressCount, buttonState.releaseCount, buttonState.isPressed);
    }

} // namespace ButtonHandler
