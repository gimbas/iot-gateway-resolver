#include "resolver.h"

// Variables
static uint16_t *pusResolverSineBuffer = NULL;
static ldma_descriptor_t __attribute__ ((aligned (4))) pResolverVDACDMADescriptor[3];
static ldma_descriptor_t __attribute__ ((aligned (4))) pResolverADCDMADescriptor[3];

static volatile uint32_t ulResolverCalibrate = 1;
static volatile uint32_t ulResolverDataAvailable = 1;
static volatile int16_t usResolverMaxRead = 0;
static volatile int16_t usResolverTop = 0;
static volatile uint32_t ulResolverSinSample = 0;
static volatile uint32_t ulResolverCosSample = 0;
static volatile int16_t ulResolverAbsoluteAngle = 0;
static volatile float fResolverAmplitude = 1;


void resolver_init()
{
// ----------------- init DAC ----------------- //
    CMU->HFPERCLKEN1 |= CMU_HFPERCLKEN1_VDAC0;

    VDAC0->CTRL = VDAC_CTRL_WARMUPMODE_KEEPINSTANDBY | (0x23 << _VDAC_CTRL_PRESC_SHIFT) | VDAC_CTRL_REFSEL_2V5LN;  // VDAC Keep in Stdby, DAC_CLK = HFPERCLK / 36, 2.5V low noise Ref
    VDAC0->CH0CTRL = VDAC_CH0CTRL_TRIGMODE_SW | VDAC_CH0CTRL_CONVMODE_CONTINUOUS; // DACconversion triggered by sw write to data, continuous conversion

    VDAC0->CAL = DEVINFO->VDAC0MAINCAL;

    VDAC0->OPA[0].CTRL = VDAC_OPA_CTRL_OUTSCALE_FULL | (0x3 << _VDAC_OPA_CTRL_DRIVESTRENGTH_SHIFT); // Enable full drive strength, rail-to-rail inputs
    VDAC0->OPA[0].TIMER = (0x000<< _VDAC_OPA_TIMER_SETTLETIME_SHIFT) | (0x05 << _VDAC_OPA_TIMER_WARMUPTIME_SHIFT) | (0x00 << _VDAC_OPA_TIMER_STARTUPDLY_SHIFT); // Recommended settings
    VDAC0->OPA[0].MUX = VDAC_OPA_MUX_RESSEL_RES0 | VDAC_OPA_MUX_GAIN3X | VDAC_OPA_MUX_RESINMUX_VSS | VDAC_OPA_MUX_NEGSEL_OPATAP | VDAC_OPA_MUX_POSSEL_DAC; // Unity gain with DAC as non-inverting input
    VDAC0->OPA[0].OUT = VDAC_OPA_OUT_MAINOUTEN; // Drive the main outp
    VDAC0->OPA[0].CAL = DEVINFO->OPA0CAL7; // Calibration for DRIVESTRENGTH = 0x3, INCBW = 0

    VDAC0->CMD = VDAC_CMD_OPA0EN; // Enable OPA0
    while(!(VDAC0->STATUS & VDAC_STATUS_OPA0ENS)); // Wait for it to be enabled

    VDAC0->CMD = VDAC_CMD_CH0EN;    // Enable VDAC0CH0
    while(!(VDAC0->STATUS & VDAC_STATUS_CH0ENS)); // Wait for it to be enabled
    // ----------------- init DAC ----------------- //

    // ----------------- init ADC ----------------- //

    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC0;
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC1;

    CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKINV | CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO | (1 << _CMU_ADCCTRL_ADC0CLKDIV_SHIFT) | CMU_ADCCTRL_ADC1CLKINV | CMU_ADCCTRL_ADC1CLKSEL_AUXHFRCO | (1 << _CMU_ADCCTRL_ADC1CLKDIV_SHIFT);

    cmu_update_clocks();

    ADC0->SCANCTRL = ADC_SCANCTRL_AT_16CYCLES | ADC_SCANCTRL_REF_2V5 | ADC_SCANCTRL_RES_12BIT;
    ADC0->SCANCTRLX = ADC_SCANCTRLX_FIFOOFACT_OVERWRITE;
    ADC0->SCANMASK = ADC_SCANMASK_SCANINPUTEN_INPUT5 | ADC_SCANMASK_SCANINPUTEN_INPUT4;
    ADC0->SCANINPUTSEL = ADC_SCANINPUTSEL_INPUT0TO7SEL_APORT0CH0TO7;

    ADC0->CAL &= ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SINGLEOFFSETINV_MASK);
    ADC0->CAL |= (DEVINFO->ADC0CAL0 & 0x7FFF0000) >> 16; // Calibration for 2V5 reference
    ADC0->BIASPROG = (ADC0->BIASPROG & ~_ADC_BIASPROG_ADCBIASPROG_MASK) | ADC_BIASPROG_GPBIASACC_HIGHACC;
    ADC1->BIASPROG = (ADC0->BIASPROG & ~_ADC_BIASPROG_ADCBIASPROG_MASK) | ADC_BIASPROG_GPBIASACC_HIGHACC;

    // ADC_CLK is 16 MHz
    // adc_sar_clk is 16 MHz (ADC_CLK / (PRESC + 1)) PRESC = 0
    // TIMEBASE period is 1 us (1 MHz) (ADC_CLK / (TIMEBASE + 1)) TIMEBASE = 15
    ADC0->CTRL = ADC_CTRL_OVSRSEL_X2 | ADC_CTRL_CHCONREFWARMIDLE_KEEPPREV | (15 << _ADC_CTRL_TIMEBASE_SHIFT) | (0 << _ADC_CTRL_PRESC_SHIFT) | ADC_CTRL_ASYNCCLKEN_ALWAYSON | ADC_CTRL_ADCCLKMODE_ASYNC | ADC_CTRL_WARMUPMODE_KEEPADCWARM;

    // ----------------- init ADC ----------------- //

    // ----------------- init TIM ----------------- //
    CMU->HFPERCLKEN1 |= CMU_HFPERCLKEN1_WTIMER1;

    WTIMER1->CTRL = WTIMER_CTRL_RSSCOIST | WTIMER_CTRL_PRESC_DIV1 | WTIMER_CTRL_CLKSEL_PRESCHFPERCLK | WTIMER_CTRL_FALLA_NONE | WTIMER_CTRL_RISEA_NONE | WTIMER_CTRL_DMACLRACT | WTIMER_CTRL_MODE_UP;
    WTIMER1->TOP = (HFPER_CLOCK_FREQ / (VDAC_FREQ * VDAC_N_SAMPLE)) - 1;
    // ----------------- init TIM ----------------- //

    // ----------------- init DMA ----------------- //
    pusResolverSineBuffer = (uint16_t *)malloc(VDAC_N_SAMPLE * sizeof(uint16_t));

    if(!pusResolverSineBuffer)
        while(1);

    for(uint16_t usNSample = 0; usNSample < VDAC_N_SAMPLE; usNSample++)
        *(pusResolverSineBuffer + VDAC_N_SAMPLE - usNSample - 1) = fResolverAmplitude * (2048 + (2047 * cos(((2.f*3.14159f)/VDAC_N_SAMPLE) * usNSample)));

    ldma_ch_disable(VDAC_DMA_CH);
    ldma_ch_peri_req_disable(VDAC_DMA_CH);
    ldma_ch_req_clear(VDAC_DMA_CH);

    ldma_ch_disable(ADC_DMA_CH);
    ldma_ch_peri_req_disable(ADC_DMA_CH);
    ldma_ch_req_clear(ADC_DMA_CH);

    ldma_ch_config(VDAC_DMA_CH, LDMA_CH_REQSEL_SOURCESEL_WTIMER1 | LDMA_CH_REQSEL_SIGSEL_WTIMER1UFOF, LDMA_CH_CFG_SRCINCSIGN_POSITIVE, LDMA_CH_CFG_DSTINCSIGN_DEFAULT, LDMA_CH_CFG_ARBSLOTS_DEFAULT, 0);

    ldma_ch_config(ADC_DMA_CH, LDMA_CH_REQSEL_SOURCESEL_ADC0 | LDMA_CH_REQSEL_SIGSEL_ADC0SCAN, LDMA_CH_CFG_SRCINCSIGN_POSITIVE, LDMA_CH_CFG_DSTINCSIGN_DEFAULT, LDMA_CH_CFG_ARBSLOTS_DEFAULT, 0);

    // VDAC DMA descriptor

    pResolverVDACDMADescriptor[0].CTRL = LDMA_CH_CTRL_DSTMODE_ABSOLUTE | LDMA_CH_CTRL_SRCMODE_ABSOLUTE | LDMA_CH_CTRL_DSTINC_NONE | LDMA_CH_CTRL_SIZE_HALFWORD | LDMA_CH_CTRL_SRCINC_ONE | LDMA_CH_CTRL_REQMODE_BLOCK | LDMA_CH_CTRL_BLOCKSIZE_UNIT1 | (((VDAC_N_SAMPLE - 1) << _LDMA_CH_CTRL_XFERCNT_SHIFT) & _LDMA_CH_CTRL_XFERCNT_MASK) | LDMA_CH_CTRL_STRUCTTYPE_TRANSFER;
    pResolverVDACDMADescriptor[0].SRC = pusResolverSineBuffer;
    pResolverVDACDMADescriptor[0].DST = &VDAC0->CH0DATA;
    pResolverVDACDMADescriptor[0].LINK = 0x00000010 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    pResolverVDACDMADescriptor[1].CTRL = LDMA_CH_CTRL_STRUCTTYPE_WRITE;
    pResolverVDACDMADescriptor[1].IMMVAL = ADC_CMD_SCANSTART;
    pResolverVDACDMADescriptor[1].DST = (void *)&ADC0->CMD;
    pResolverVDACDMADescriptor[1].LINK = 0x00000010 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    pResolverVDACDMADescriptor[2].CTRL = LDMA_CH_CTRL_STRUCTTYPE_WRITE;
    pResolverVDACDMADescriptor[2].IMMVAL = 0;
    pResolverVDACDMADescriptor[2].DST = &ulResolverDataAvailable;
    pResolverVDACDMADescriptor[2].LINK = 0xFFFFFFE0 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    // ADC DMA descriptor

    pResolverADCDMADescriptor[0].CTRL = LDMA_CH_CTRL_DSTMODE_ABSOLUTE | LDMA_CH_CTRL_SRCMODE_ABSOLUTE | LDMA_CH_CTRL_DSTINC_NONE | LDMA_CH_CTRL_SIZE_HALFWORD | LDMA_CH_CTRL_SRCINC_ONE | LDMA_CH_CTRL_REQMODE_BLOCK | LDMA_CH_CTRL_BLOCKSIZE_UNIT1 | ((0 << _LDMA_CH_CTRL_XFERCNT_SHIFT) & _LDMA_CH_CTRL_XFERCNT_MASK) | LDMA_CH_CTRL_STRUCTTYPE_TRANSFER;
    pResolverADCDMADescriptor[0].SRC = (void *)&ADC0->SCANDATA;
    pResolverADCDMADescriptor[0].DST = &(ulResolverCosSample);
    pResolverADCDMADescriptor[0].LINK = 0x00000010 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    pResolverADCDMADescriptor[1].CTRL = LDMA_CH_CTRL_DSTMODE_ABSOLUTE | LDMA_CH_CTRL_SRCMODE_ABSOLUTE | LDMA_CH_CTRL_DSTINC_NONE | LDMA_CH_CTRL_SIZE_HALFWORD | LDMA_CH_CTRL_SRCINC_ONE | LDMA_CH_CTRL_REQMODE_BLOCK | LDMA_CH_CTRL_BLOCKSIZE_UNIT1 | ((0 << _LDMA_CH_CTRL_XFERCNT_SHIFT) & _LDMA_CH_CTRL_XFERCNT_MASK) | LDMA_CH_CTRL_STRUCTTYPE_TRANSFER;
    pResolverADCDMADescriptor[1].SRC = (void *)&ADC0->SCANDATA;
    pResolverADCDMADescriptor[1].DST = &(ulResolverSinSample);
    pResolverADCDMADescriptor[1].LINK = 0x00000010 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    pResolverADCDMADescriptor[2].CTRL = LDMA_CH_CTRL_STRUCTTYPE_WRITE;
    pResolverADCDMADescriptor[2].IMMVAL = 1;
    pResolverADCDMADescriptor[2].DST = &ulResolverDataAvailable;
    pResolverADCDMADescriptor[2].LINK = 0xFFFFFFe0 | LDMA_CH_LINK_LINK | LDMA_CH_LINK_LINKMODE_RELATIVE;

    // each descriptor is 4 words 1 word = 4 bytes, each descriptor step is +-16 (in this case 2 * 16 = -32 = 0xFFFFFFE0)

    ldma_ch_peri_req_enable(VDAC_DMA_CH);
    ldma_ch_enable(VDAC_DMA_CH);
    ldma_ch_load(VDAC_DMA_CH, pResolverVDACDMADescriptor);

    ldma_ch_peri_req_enable(ADC_DMA_CH);
    ldma_ch_enable(ADC_DMA_CH);
    ldma_ch_load(ADC_DMA_CH, pResolverADCDMADescriptor);

    // ----------------- init DMA ----------------- //

    WTIMER1->CMD |= WTIMER_CMD_START;
}

void resolver_task()
{
    if(ulResolverDataAvailable)
    {
        if(ulResolverCalibrate)
        {
            if((ulResolverCosSample & 0xFFF) > usResolverMaxRead)
            {
                usResolverMaxRead *= 0.7;
                usResolverMaxRead += 0.3 * ulResolverCosSample;
                usResolverTop = usResolverMaxRead - RESOLVER_OFFSET;
            }
            if((ulResolverSinSample & 0xFFF)  > usResolverMaxRead)
            {
                usResolverMaxRead *= 0.7;
                usResolverMaxRead += 0.3 * ulResolverSinSample;
                usResolverTop = usResolverMaxRead - RESOLVER_OFFSET;
            }

            static uint64_t ullCalibrationTime = 0;
            if((g_ullSystemTick > (ullCalibrationTime + 5000)))
            {
                ulResolverCalibrate = 0;
            }
        }

        int16_t sResolverCosine = CAP(INT16_MIN, INT16_MAX * ((int16_t)(ulResolverSinSample) - RESOLVER_OFFSET) / usResolverTop, INT16_MAX);
        int16_t sResolverSine = CAP(INT16_MIN, INT16_MAX * ((int16_t)(ulResolverCosSample) - RESOLVER_OFFSET) / usResolverTop, INT16_MAX);

        ulResolverAbsoluteAngle = atan2i16(sResolverSine, sResolverCosine);

        ulResolverDataAvailable = 0;
    }
}

int16_t resolver_angle()
{
    return ulResolverAbsoluteAngle;
}