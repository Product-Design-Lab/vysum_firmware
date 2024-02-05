#pragma once

#include <cstdint>
#include <cstdbool>

namespace Button
{

    typedef struct
    {
        uint32_t pressCount;
        uint32_t releaseCount;
        bool isPressed;
        volatile _is_pressed;
    } ButtonState_t;

    // Initialize buttons
    void init();

    // Getters for the state of each button
    ButtonState_t getButtonWhiteState();
    ButtonState_t getButtonBlueState();
    void printButtonState(const ButtonState_t buttonState);

} // namespace ButtonHandler
