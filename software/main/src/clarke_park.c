#include "clarke_park.h"

void clarke_transform(float fIA, float fIB, float fIC, float *pfIAlpha, float *pfIBeta)
{
    *pfIAlpha = fIA;
    *pfIBeta = (fIB - fIC) * INVSQRT3;
}
void clarke_inverse_transform(float fVAlpha, float fVBeta, float *pfVA, float *pfVB, float *pfVC)
{
    *pfVA = fVAlpha;
    *pfVB = (-fVAlpha * 0.5f) + (SQRT3OVER2 * fVBeta);
    *pfVC = (-fVAlpha * 0.5f) - (SQRT3OVER2 * fVBeta);
}

void park_transform(float fIAlpha, float fIBeta, float fCos, float fSin, float *pfID, float *pfIQ)
{
    *pfID = (fIAlpha * fCos) + (fIBeta * fSin);
    *pfIQ = (-fIAlpha * fSin) + (fIBeta * fCos);
}
void park_inverse_transform(float fCos, float fSin, float fVD, float fVQ, float *pfVAlpha, float *pfVBeta)
{
    *pfVAlpha = (fVD * fCos) - (fVQ * fSin);
    *pfVBeta = (fVD * fSin) + (fVQ * fCos);
}