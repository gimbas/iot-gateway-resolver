#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <em_device.h>
#include <stdlib.h>
#include <math.h>
#include "lizard_math.h"
#include "ldma.h"
#include "cmu.h"
#include "systick.h"

// Defines
#define SVPWM_FREQ 10000

// Variables

// Forward declarations
void svpwm_init();

void svpwm_set(float fValpha, float fVbeta);


#endif // __SVPWM_H__