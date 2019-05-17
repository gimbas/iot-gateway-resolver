#ifndef __RESOLVER_H__
#define __RESOLVER_H__

#include <em_device.h>
#include <stdlib.h>
#include <math.h>
#include "lizard_math.h"
#include "ldma.h"
#include "cmu.h"
#include "systick.h"

// Defines
#define VDAC_FREQ 8500
#define VDAC_N_SAMPLE 50
#define VDAC_DMA_CH 12
#define ADC_DMA_CH 11
#define RESOLVER_DEFAULT_OFFSET 2048
#define RESOLVER_DEFAULT_TOP 1798
#define RESOLVER_DEFAULT_AMPLITUDE 0.9f

// Variables

void resolver_init();

void resolver_task();

int16_t resolver_angle();

#endif  // __RESOLVER_H__