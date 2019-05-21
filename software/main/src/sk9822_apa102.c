#include "sk9822_apa102.h"

static uint8_t *pubBuffer = NULL;
static ldma_descriptor_t __attribute__ ((aligned (4))) pDMADescriptor[1];

void sk9822_apa102_init()
{
    free(pubBuffer);

    pubBuffer = (uint8_t *)malloc((SK9822_APA102_NUM_LEDS * 4) + 8); // 3 = Colors, 1 = brightness

    if(!pubBuffer)
        return;

    memset(pubBuffer, 0, (SK9822_APA102_NUM_LEDS * 4) + 8);


    ldma_ch_disable(SK9822_APA102_DMA_CHANNEL);
    ldma_ch_peri_req_disable(SK9822_APA102_DMA_CHANNEL);
    ldma_ch_req_clear(SK9822_APA102_DMA_CHANNEL);

    ldma_ch_config(SK9822_APA102_DMA_CHANNEL, LDMA_CH_REQSEL_SOURCESEL_USART3 | LDMA_CH_REQSEL_SIGSEL_USART3TXEMPTY, LDMA_CH_CFG_SRCINCSIGN_POSITIVE, LDMA_CH_CFG_DSTINCSIGN_DEFAULT, LDMA_CH_CFG_ARBSLOTS_DEFAULT, 0);

    pDMADescriptor[0].CTRL = LDMA_CH_CTRL_DSTMODE_ABSOLUTE | LDMA_CH_CTRL_SRCMODE_ABSOLUTE | LDMA_CH_CTRL_DSTINC_NONE | LDMA_CH_CTRL_SIZE_BYTE | LDMA_CH_CTRL_SRCINC_ONE | LDMA_CH_CTRL_REQMODE_BLOCK | LDMA_CH_CTRL_BLOCKSIZE_UNIT1 | ((((SK9822_APA102_NUM_LEDS * 4) + 7) << _LDMA_CH_CTRL_XFERCNT_SHIFT) & _LDMA_CH_CTRL_XFERCNT_MASK) | LDMA_CH_CTRL_STRUCTREQ | LDMA_CH_CTRL_STRUCTTYPE_TRANSFER;
    pDMADescriptor[0].SRC = pubBuffer;
    pDMADescriptor[0].DST = &(USART3->TXDATA);
    pDMADescriptor[0].LINK = 0x00000000;

    ldma_ch_peri_req_enable(SK9822_APA102_DMA_CHANNEL);
    ldma_ch_enable(SK9822_APA102_DMA_CHANNEL);
}

void sk9822_apa102_set_color(uint16_t usNLed, uint8_t ubBrightness, uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue, uint8_t ubUpdate)
{
    if(usNLed >= SK9822_APA102_NUM_LEDS)
        return;

    if(!pubBuffer)
        return;

    *(pubBuffer + (usNLed * 4) + 4) = 0xE0 + (ubBrightness & 0x1F); // offset the preamble packet
    *(pubBuffer + (usNLed * 4) + 5) = ubBlue; // offset the preamble packet and 1 register
    *(pubBuffer + (usNLed * 4) + 6) = ubGreen; // offset the preamble packet and 2 registers
    *(pubBuffer + (usNLed * 4) + 7) = ubRed; // offset the preamble packet and 3 registers

    if(ubUpdate)
        sk9822_apa102_update();
}

void sk9822_apa102_update()
{
    ldma_ch_req_clear(SK9822_APA102_DMA_CHANNEL);
    ldma_ch_load(SK9822_APA102_DMA_CHANNEL, pDMADescriptor);
}