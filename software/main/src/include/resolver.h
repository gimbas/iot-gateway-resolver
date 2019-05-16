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
#define RESOLVER_OFFSET 2179 // 0.532f  // 1330 mV / 2500 mV
#define RESOLVER_TOP 1835 // 4013.f     // 2450 mV

// Variables

void resolver_init();

void resolver_task();

int16_t resolver_angle();

#endif  // __RESOLVER_H__