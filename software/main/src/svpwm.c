#include "svpwm.h"

void svpwm_init()
{
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;

    TIMER0->CTRL = TIMER_CTRL_RSSCOIST | TIMER_CTRL_PRESC_DIV1 | TIMER_CTRL_CLKSEL_PRESCHFPERCLK | TIMER_CTRL_FALLA_NONE | TIMER_CTRL_RISEA_NONE | TIMER_CTRL_DMACLRACT | TIMER_CTRL_DEBUGRUN | TIMER_CTRL_MODE_UPDOWN;
    TIMER0->TOP = ((HFPERB_CLOCK_FREQ / 2) / SVPWM_FREQ) - 1;
    TIMER0->CNT = 0x0000;


    TIMER0->CC[0].CTRL = TIMER_CC_CTRL_CUFOA_CLEAR | TIMER_CC_CTRL_COFOA_SET | TIMER_CC_CTRL_CMOA_TOGGLE | TIMER_CC_CTRL_MODE_PWM;
    TIMER0->CC[1].CTRL = TIMER_CC_CTRL_CUFOA_CLEAR | TIMER_CC_CTRL_COFOA_SET | TIMER_CC_CTRL_CMOA_TOGGLE | TIMER_CC_CTRL_MODE_PWM;
    TIMER0->CC[2].CTRL = TIMER_CC_CTRL_CUFOA_CLEAR | TIMER_CC_CTRL_COFOA_SET | TIMER_CC_CTRL_CMOA_TOGGLE | TIMER_CC_CTRL_MODE_PWM;

    TIMER0->CC[0].CCV = 0; // U
    TIMER0->CC[1].CCV = 0; // V
    TIMER0->CC[2].CCV = 0; // W

    // HFPERBCLK = 72 MHz
    // DTTIME = HFPERBCLK / DTPRESC = 4.5 MHz = 222 nS / LSB
    TIMER0->DTTIME = TIMER_DTTIME_DTPRESC_DIV16 | ((5 << _TIMER_DTTIME_DTFALLT_SHIFT) & _TIMER_DTTIME_DTFALLT_MASK) | ((5 << _TIMER_DTTIME_DTRISET_SHIFT) & _TIMER_DTTIME_DTRISET_MASK);
    TIMER0->DTFC = TIMER_DTFC_DTFA_CLEAR;
    TIMER0->DTOGEN = TIMER_DTOGEN_DTOGCC0EN | TIMER_DTOGEN_DTOGCC1EN | TIMER_DTOGEN_DTOGCC2EN | TIMER_DTOGEN_DTOGCDTI0EN | TIMER_DTOGEN_DTOGCDTI1EN | TIMER_DTOGEN_DTOGCDTI2EN;
    TIMER0->DTCTRL = TIMER_DTCTRL_DTFATS | TIMER_DTCTRL_DTEN;

    TIMER0->ROUTELOC0 = TIMER_ROUTELOC0_CC2LOC_LOC0 | TIMER_ROUTELOC0_CC1LOC_LOC0 | TIMER_ROUTELOC0_CC0LOC_LOC0;
    TIMER0->ROUTELOC2 = TIMER_ROUTELOC2_CDTI2LOC_LOC0 | TIMER_ROUTELOC2_CDTI1LOC_LOC0 | TIMER_ROUTELOC2_CDTI0LOC_LOC0;
    TIMER0->ROUTEPEN = TIMER_ROUTEPEN_CC0PEN | TIMER_ROUTEPEN_CC1PEN | TIMER_ROUTEPEN_CC2PEN | TIMER_ROUTEPEN_CDTI0PEN | TIMER_ROUTEPEN_CDTI1PEN | TIMER_ROUTEPEN_CDTI2PEN;

    TIMER0->CMD = TIMER_CMD_START;
}

void svpwm_set(float fValpha, float fVbeta)
{
    // variables
    float fI, fJ, fK;
    float fT0, fT1, fT2;

    // modified inverse clark transformation
    fI = (SQRT3OVER2 * fValpha) - (fVbeta * 0.5);
    fJ = fVbeta;
    fK = (-SQRT3OVER2 * fValpha) - (fVbeta * 0.5);

    // determine sector
    // switch(sector)
    switch(SIGN(fI) + (2 * SIGN(fJ)) + (4 * SIGN(fK)))
    {
        case 1: // Sector 6 -> 300º - 360º
            // Determine T0, T1, T2
            fT1 = -fJ;
            fT2 = -fK;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (0.5 * fT0) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (fT1 + (0.5 * fT0)) * TIMER0->TOP;  // W
        break;

        case 2: // Sector 2 -> 60º - 120º
            // Determine T0, T1, T2
            fT1 = -fK;
            fT2 = -fI;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (fT1 + (0.5 * fT0)) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (0.5 * fT0) * TIMER0->TOP;  // W
        break;

        case 3: // Sector 1 -> 0º - 60º
            // Determine T0, T1, T2
            fT1 = fI;
            fT2 = fJ;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (fT2 + (0.5 * fT0)) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (0.5 * fT0) * TIMER0->TOP;  // W

        break;

        case 4: // Sector 4 -> 180º - 240º
            // Determine T0, T1, T2
            fT1 = -fI;
            fT2 = -fJ;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (0.5 * fT0) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (fT1 + (0.5 * fT0)) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP;  // W
        break;

        case 5: // Sector 5 -> 240º - 300º
            // Determine T0, T1, T2
            fT1 = fK;
            fT2 = fI;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (fT2 + (0.5 * fT0)) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (0.5 * fT0) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP;  // W
        break;

        case 6: // Sector 3 -> 120º - 180º
            // Determine T0, T1, T2
            fT1 = fJ;
            fT2 = fK;
            fT0 = 1.f -fT1 -fT2;

            // load values into pwm registers
            TIMER0->CC[0].CCVB = (0.5 * fT0) * TIMER0->TOP; // U
            TIMER0->CC[1].CCVB = (fT1 + fT2 + (0.5 * fT0)) * TIMER0->TOP; // V
            TIMER0->CC[2].CCVB = (fT2 + (0.5 * fT0)) * TIMER0->TOP;  // W
        break;

        default: // erroneous sector
        break;
    }
}