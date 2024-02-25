#include "PDL_Async_Button_Group.h"

// In AsyncButtonGroup.cpp
AsyncButtonGroup *AsyncButtonGroup::instances[MAX_PIN_NUM] = {nullptr};

uint8_t AsyncButtonGroup::_instance_count = 0;

void AsyncButtonGroup::set_init_state()
{
    _state = BUTTON_IDLE;
    // attachInterrupt(_pin, _gpioCallback, _idle_logic_level == LOW ? RISING : FALLING);
    attachInterrupt(_pin, _idx == 0 ? _gpioCallback_instance1 : _gpioCallback_instance2, _idle_logic_level == LOW ? RISING : FALLING);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTimerStopFromISR(_timerHandle, &xHigherPriorityTaskWoken);
    xTimerStop(_idx == 0 ? _timerHandle1 : _timerHandle2, pdMS_TO_TICKS(0));
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void AsyncButtonGroup::set_debounce_state()
{
    _state = BUTTON_DEBOUCE;
    detachInterrupt(_pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTimerChangePeriod(_timerHandle, pdMS_TO_TICKS(_debounceTime), xHigherPriorityTaskWoken);
    xTimerChangePeriodFromISR(_idx == 0 ? _timerHandle1 : _timerHandle2, pdMS_TO_TICKS(_debounceTime), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void AsyncButtonGroup::set_short_press_state()
{
    // button is pressed and debounced
    _state = BUTTON_SHORT_PRESS;
    // start waiting for button release
    // attachInterrupt(_pin, _gpioCallback, _idle_logic_level == LOW ? FALLING : RISING);
    attachInterrupt(_pin, _idx == 0 ? _gpioCallback_instance1 : _gpioCallback_instance2, _idle_logic_level == LOW ? FALLING : RISING);

    // start waiitng for long press timeup
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTimerChangePeriodFromISR(_timerHandle, pdMS_TO_TICKS(_longPressTime), &xHigherPriorityTaskWoken);
    xTimerChangePeriodFromISR(_idx == 0 ? _timerHandle1 : _timerHandle2, pdMS_TO_TICKS(_longPressTime), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void AsyncButtonGroup::set_long_press_state()
{
    _state = BUTTON_LONG_PRESS;
    // start waiting for button release
    // attachInterrupt(_pin, _gpioCallback, _idle_logic_level == LOW ? FALLING : RISING);
    attachInterrupt(_pin, _idx == 0 ? _gpioCallback_instance1 : _gpioCallback_instance2, _idle_logic_level == LOW ? FALLING : RISING);

    // timer is no longer needed
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTimerStopFromISR(_timerHandle, &xHigherPriorityTaskWoken);
    xTimerStopFromISR(_idx == 0 ? _timerHandle1 : _timerHandle2, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void AsyncButtonGroup::_gpioCallback()
{
    if (_state == BUTTON_IDLE)
    {
        set_debounce_state();
    }
    else if (_state == BUTTON_SHORT_PRESS) // button released before long press timeup
    {
        _short_press_count++;
        _output_state = BUTTON_SHORT_PRESS;
        set_init_state();
    }
    else if (_state == BUTTON_LONG_PRESS) // button released after long press timeup
    {
        set_init_state();
    }
    else
    {
        // something went wrong. GPIO ISR should not be called in Debounce state
    }
}

void AsyncButtonGroup::_gpioCallback_instance1()
{
    instances[0]->_gpioCallback();
}

void AsyncButtonGroup::_gpioCallback_instance2()
{
    instances[1]->_gpioCallback();
}

void AsyncButtonGroup::_TimerCallback(TimerHandle_t xTimer)
{
    if (_state == BUTTON_DEBOUCE)
    {
        if (digitalRead(_pin) != _idle_logic_level) // valid button press
        {
            set_short_press_state();
        }
        else // invalid button press
        {
            set_init_state();
        }
    }
    else if (_state == BUTTON_SHORT_PRESS) // long press timeup
    {
        _long_press_count++;
        _output_state = BUTTON_LONG_PRESS;
        set_long_press_state();
    }
    else
    {
        // something went wrong. Timer callback should not be called in Idle or Long Press state
    }
}

void AsyncButtonGroup::_TimerCallback_instance1(TimerHandle_t xTimer)
{
    instances[0]->_TimerCallback(xTimer);
}

void AsyncButtonGroup::_TimerCallback_instance2(TimerHandle_t xTimer)
{
    instances[1]->_TimerCallback(xTimer);
}

AsyncButtonGroup::AsyncButtonGroup()
{
    if (_instance_count >= MAX_PIN_NUM)
    {
        return;
    }
    _idx = _instance_count;
    instances[_idx] = this;
    _instance_count++;

    _pin = 0;
    _debounceTime = 50;
    _longPressTime = 1000;
    _idle_logic_level = LOW;
    _short_press_count = 0;
    _long_press_count = 0;
    _state = BUTTON_IDLE;
    _output_state = BUTTON_IDLE;
    if (_idx == 0)
        _timerHandle1 = xTimerCreate("ButtonTimer", pdMS_TO_TICKS(_debounceTime), pdFALSE, (void *)0, _TimerCallback_instance1);
    else
        _timerHandle2 = xTimerCreate("ButtonTimer", pdMS_TO_TICKS(_debounceTime), pdFALSE, (void *)0, _TimerCallback_instance2);
}

void AsyncButtonGroup::setPin(uint8_t pin)
{
    _pin = pin;
    pinMode(_pin, INPUT);
}

void AsyncButtonGroup::setPin(uint8_t pin, bool idle_logic_level)
{
    _pin = pin;
    _idle_logic_level = idle_logic_level;
    pinMode(_pin, idle_logic_level == HIGH ? INPUT_PULLUP : INPUT_PULLDOWN);
}

void AsyncButtonGroup::setDebounceTime(uint32_t ms)
{
    _debounceTime = ms;
}

void AsyncButtonGroup::setLongPressTime(uint32_t ms)
{
    _longPressTime = ms;
}

void AsyncButtonGroup::setIdleLogicLevel(bool logic_level)
{
    _idle_logic_level = logic_level;
    pinMode(_pin, logic_level == HIGH ? INPUT_PULLUP : INPUT_PULLDOWN);
}

uint8_t AsyncButtonGroup::getState(int *short_press_count, int *long_press_count)
{
    *short_press_count = _short_press_count;
    *long_press_count = _long_press_count;
    _short_press_count = 0;
    _long_press_count = 0;
    return getState();
}

uint8_t AsyncButtonGroup::getState()
{
    uint8_t state = _output_state;
    _output_state = BUTTON_IDLE;
    return state;
}

void AsyncButtonGroup::init()
{
    set_init_state();
}