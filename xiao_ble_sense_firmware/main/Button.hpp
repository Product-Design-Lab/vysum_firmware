#pragma once

#include <cstdint>
#include <cstdbool>

namespace Button {

    typedef struct {
        volatile uint32_t pressCount;
        volatile uint32_t releaseCount;
        volatile bool isPressed;
    } ButtonState_t;

    // Initialize buttons
    void init();

    // Getters for the state of each button
    ButtonState_t getButtonWhiteState();
    ButtonState_t getButtonBlueState();
    void printButtonState(const ButtonState_t buttonState);

} // namespace ButtonHandler
