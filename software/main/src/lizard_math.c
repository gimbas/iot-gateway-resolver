#include "lizard_math.h"

uint16_t abs16(int16_t sValue)
{
    uint16_t usSign = sValue >> 15;     // make a mask of the sign bit

    sValue ^= usSign;                   // toggle the bits if value is negative
    sValue += usSign & 1;               // add one if value was negative

    return sValue;
}

int16_t atan2i16(int16_t y, int16_t x)
{
    uint16_t absx = abs16(x);
    uint16_t absy = abs16(y);

    if(absx > INT16_MAX)
        absx = INT16_MAX;

    if(absy > INT16_MAX)
        absy = INT16_MAX;

    int32_t denominator = absy + absx;

    if (denominator == 0)
        return 0; // avoid DBZ

    int32_t numerator = (int32_t)(TAU / 8) * (int32_t)(absy - absx);

    int16_t theta = ((numerator << 3) / denominator) >> 3;

    if (y >= 0) // Note: Cartesian plane quadrants
    {
        if (x >= 0)
            return (TAU * 1 / 8) + theta; // quadrant I    Theta counts 'towards the y axis',
        else
            return (TAU * 3 / 8) - theta; // quadrant II   So, negate it in quadrants II and IV
    }
    else
    {
        if (x < 0)
            return (TAU * -3 / 8) + theta; // quadrant III. -3/8 = 5/8
        else
            return (TAU * -1 / 8) - theta; // quadrant IV.  -1/8 = 7/8
    }
}