#ifndef __CLARKE_PARK_H__
#define __CLARKE_PARK_H__

#include <em_device.h>
#include <stdlib.h>
#include <math.h>
#include "lizard_math.h"
#include "ldma.h"
#include "cmu.h"
#include "systick.h"

void clarke_transform(float fIA, float fIB, float fIC, float *pfIAlpha, float *pfIBeta);
void clarke_inverse_transform(float fVAlpha, float fVBeta, float *pfVA, float *pfVB, float *pfVC);

void park_transform(float fIAlpha, float fIBeta, float fCos, float fSin, float *pfID, float *pfIQ);
void park_inverse_transform(float fCos, float fSin, float fVD, float fVQ, float *pfVAlpha, float *pfVBeta);

#endif // __CLARKE_PARK_H__