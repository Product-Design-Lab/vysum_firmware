#include "Button.hpp"
#include <Arduino.h>
#include "diagnostics.h"
#include "pins.h"

#include <FreeRTOS.h>
#include <task.h>

namespace Button
{

    // Notify values
    // enum
    // {
    //     BUTTON1_RISE_NOTIFY,
    //     BUTTON1_FALL_NOTIFY,
    //     BUTTON2_RISE_NOTIFY,
    //     BUTTON2_FALL_NOTIFY
    // };

    const uint32_t BUTTON1_RISE_NOTIFY = 1;
    const uint32_t BUTTON1_FALL_NOTIFY = 2;
    const uint32_t BUTTON2_RISE_NOTIFY = 3;
    const uint32_t BUTTON2_FALL_NOTIFY = 4;
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

        while (1)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            switch (ulNotificationValue)
            {
            case BUTTON1_RISE_NOTIFY:
                Serial.println("Button 1 pressed.");
                break;
            case BUTTON1_FALL_NOTIFY:
                Serial.println("Button 1 released.");
                break;
            case BUTTON2_RISE_NOTIFY:
                Serial.println("Button 2 pressed.");
                break;
            case BUTTON2_FALL_NOTIFY:
                Serial.println("Button 2 released.");
                break;
            }

            run_diagnostics();
        }
    }

    void notifyTask(uint32_t notifyValue)
    {
        if (DIAG::get_opt() & DIAG::D_BUTTON)
        {
            ulNotificationValue = notifyValue;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    static void buttonWhiteRiseISR()
    {
        buttonWhiteState.pressCount++;
        buttonWhiteState.isPressed = true;
        notifyTask(BUTTON1_RISE_NOTIFY);
    }

    static void buttonWhiteFallISR()
    {
        buttonWhiteState.releaseCount++;
        buttonWhiteState.isPressed = false;
        notifyTask(BUTTON1_FALL_NOTIFY);
    }

    static void buttonBlueRiseISR()
    {
        buttonBlueState.pressCount++;
        buttonBlueState.isPressed = true;
        notifyTask(BUTTON2_RISE_NOTIFY);
    }

    static void buttonBlueFallISR()
    {
        buttonBlueState.releaseCount++;
        buttonBlueState.isPressed = false;
        notifyTask(BUTTON2_FALL_NOTIFY);
    }

    void init()
    {

        pinMode(PIN_BUTTON_WHITE, INPUT_PULLUP);
        pinMode(PIN_BUTTON_BLUE, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_WHITE), buttonWhiteRiseISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_WHITE), buttonWhiteFallISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_BLUE), buttonBlueRiseISR, RISING);
        attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_BLUE), buttonBlueFallISR, FALLING);

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
