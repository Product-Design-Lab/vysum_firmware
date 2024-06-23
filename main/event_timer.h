#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void (*callback)(void);

bool event_timer_init(uint32_t timeout_ms, callback cb);
bool event_timer_set_timeout(uint32_t timeout_ms);
bool event_timer_set_callback(callback cb);
bool event_timer_reset(void);
bool event_timer_stop(void);
bool event_timer_delete(void);
int event_timer_get_status(void);

