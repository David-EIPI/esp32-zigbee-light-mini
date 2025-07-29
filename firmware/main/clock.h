
#pragma once

#include "driver/gptimer.h"

esp_err_t init_timer(gptimer_handle_t *timer_ptr, uint32_t timer_resolution);

uint64_t gpclock(gptimer_handle_t timer);

