
#pragma once

#include <stdint.h>

#define DIM_LEVEL_MAX 100
#define TRANSITION_UNITS 5 // 'th of a second

volatile extern int32_t dimLevel;
volatile extern int32_t phaseCorrection_us;

volatile extern int32_t transitionTime;
volatile extern int stopDimmer;


void triac_dimmer_task(void *arg);
