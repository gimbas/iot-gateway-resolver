#ifndef __SK9822_APA102_H__
#define __SK9822_APA102_H__

#include <em_device.h>
#include <stdlib.h>
#include <string.h>
#include "cmu.h"
#include "ldma.h"
#include "utils.h"

#define SK9822_APA102_NUM_LEDS		1
#define SK9822_APA102_DMA_CHANNEL   16

void sk9822_apa102_init();

void sk9822_apa102_set_color(uint16_t usNLed, uint8_t ubBrightness, uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue, uint8_t ubUpdate);

void sk9822_apa102_update();

#endif // __SK9822_APA102_H__
