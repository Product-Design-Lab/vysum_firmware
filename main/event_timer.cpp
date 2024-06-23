#include "event_timer.h"

#include "FreeRTOS.h"
#include "timers.h"

static TimerHandle_t event_timer = NULL;
static callback event_timer_cb = NULL;

// Timer callback function
static void event_timer_callback(TimerHandle_t xTimer)
{
    if (event_timer_cb != NULL)
    {
        event_timer_cb();
    }
}

bool event_timer_init(uint32_t timeout_ms, callback cb)
{
    if (event_timer != NULL)
    {
        return false;  // Timer already initialized
    }

    event_timer_cb = cb;
    event_timer = xTimerCreate("event_timer", pdMS_TO_TICKS(timeout_ms), pdFALSE, (void *)0, event_timer_callback);

    return (event_timer != NULL);  // Return true if timer creation is successful
}

bool event_timer_set_timeout(uint32_t timeout_ms)
{
    if (event_timer == NULL)
    {
        return false;
    }

    return (xTimerChangePeriod(event_timer, pdMS_TO_TICKS(timeout_ms), 0) == pdPASS);  // Ensure return type is boolean

}

bool event_timer_set_callback(callback cb)
{
    if (event_timer == NULL)
    {
        return false;
    }

    event_timer_cb = cb;
    return true;
}

bool event_timer_reset(void)
{
    if (event_timer == NULL)
    {
        return false;
    }

    return (xTimerReset(event_timer, 0) == pdPASS);  // Ensure return type is boolean
}

bool event_timer_stop(void)
{
    if (event_timer == NULL)
    {
        return false;
    }

    return (xTimerStop(event_timer, 0) == pdPASS);  // Ensure return type is boolean
}

bool event_timer_delete(void)
{
    if (event_timer == NULL)
    {
        return false;
    }

    if (xTimerDelete(event_timer, 0) == pdPASS)
    {
        event_timer = NULL;
        return true;
    }

    return false;
}

// Return -1 if timer is not initialized
// Return 0 if timer is stopped
// Return 1 if timer is running
int event_timer_get_status(void)
{
    if (event_timer == NULL)
    {
        return -1;
    }

    return xTimerIsTimerActive(event_timer);
}
