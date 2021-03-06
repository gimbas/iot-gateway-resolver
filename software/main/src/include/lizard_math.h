#ifndef __LIZARD_MATH_H__
#define __LIZARD_MATH_H__

#include <em_device.h>
#include <stdlib.h>
#include <stdint.h>

// Sign of, 1 if (+) 0 if (-)
#define SIGN(a)  (!(a < 0))

// Absolute value of
#define ABS(a)  ((a) < 0 ? (-(a)) : (a))

// Macros that get the lesser/greater of two values
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

#define CAP(l,a,h) (MAX(MIN(h,a),l))

// Constants
#define F_PI    3.14159265358979323846f // PI in float
#define TAU (2 * INT16_MAX)
#define INVSQRT3 (0.577350269f)
#define SQRT3 (1.732050808f)
#define SQRT3OVER2 (0.86602540379f)

// Funcions
uint16_t abs16(int16_t sValue);

int16_t atan2i16(int16_t y, int16_t x);

#endif // __LIZARD_MATH_H__