#include "gpio.h"

static void gpio_isr(uint32_t ulFlags)
{
    //if(ulFlags & BIT(4))


}
void _gpio_even_isr()
{
    uint32_t ulFlags = GPIO->IF;

    gpio_isr(ulFlags & 0x55555555);

    GPIO->IFC = 0x55555555; // Clear all even flags
}
void _gpio_odd_isr()
{
    uint32_t ulFlags = GPIO->IF;

    gpio_isr(ulFlags & 0xAAAAAAAA);

    GPIO->IFC = 0xAAAAAAAA; // Clear all odd flags
}

void gpio_init()
{
    CMU->HFBUSCLKEN0 |= CMU_HFBUSCLKEN0_GPIO;

    // NC - Not Connected (not available in mcu package)
    // NR - Not routed (no routing to pin on pcb, floating)
    // NU - Not used (not currently in use)

    // Port A
    GPIO->P[0].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[0].MODEL  = GPIO_P_MODEL_MODE0_PUSHPULL                 // US3_MOSI_RFM - Location 0
                      | GPIO_P_MODEL_MODE1_PUSHPULL                 // US3_MISO_RFM - Location 0
                      | GPIO_P_MODEL_MODE2_PUSHPULL                 // US3_CLK_RFM - Location 0
                      | GPIO_P_MODEL_MODE3_PUSHPULL                 // US3_CS_RFM - Location 0
                      | GPIO_P_MODEL_MODE4_PUSHPULL                 // RFM_RESET
                      | GPIO_P_MODEL_MODE5_PUSHPULL                 // NR
                      | GPIO_P_MODEL_MODE6_DISABLED                 // RFM_IRQ
                      | GPIO_P_MODEL_MODE7_DISABLED;                // GSM_STATUS
    GPIO->P[0].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // GSM_RF_SYNC
                      | GPIO_P_MODEH_MODE9_DISABLED                 // GSM_PWR_KEY
                      | GPIO_P_MODEH_MODE10_DISABLED                // GSM_LED_EN
                      | GPIO_P_MODEH_MODE11_DISABLED                // NR
                      | GPIO_P_MODEH_MODE12_DISABLED                // TFT_RESET
                      | GPIO_P_MODEH_MODE13_DISABLED                // TFT_IRQ
                      | GPIO_P_MODEH_MODE14_DISABLED                // TFT_DC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // TIM3_CC2_BUZZ - Location 0
    GPIO->P[0].DOUT   = 0;
    GPIO->P[0].OVTDIS = 0;

    // Port B
    GPIO->P[1].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[1].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // LED_DI
                      | GPIO_P_MODEL_MODE1_DISABLED                 // BTN_1
                      | GPIO_P_MODEL_MODE2_DISABLED                 // BTN_2
                      | GPIO_P_MODEL_MODE3_DISABLED                 // BTN_3
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE7_DISABLED;                // MAIN_LFXTAL_P
    GPIO->P[1].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // MAIN_LFXTAL_N
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NR
                      | GPIO_P_MODEH_MODE10_DISABLED                // WTIM2_CC1_TFT_BL - Location 2
                      | GPIO_P_MODEH_MODE11_DISABLED                // I2C1_SDA_TFT - Location 1
                      | GPIO_P_MODEH_MODE12_DISABLED                // I2C1_SCL_TFT - Location 1
                      | GPIO_P_MODEH_MODE13_DISABLED                // MAIN_HFXTAL_P
                      | GPIO_P_MODEH_MODE14_DISABLED                // MAIN_HFXTAL_N
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[1].DOUT   = 0;
    GPIO->P[1].OVTDIS = 0;

    // Port C
    GPIO->P[2].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (7 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[2].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // US2_RTS_GSM - Location 0
                      | GPIO_P_MODEL_MODE1_DISABLED                 // US2_CTS_GSM - Location 0
                      | GPIO_P_MODEL_MODE2_DISABLED                 // US2_TX_GSM - Location 0
                      | GPIO_P_MODEL_MODE3_DISABLED                 // US2_RX_GSM - Location 0
                      | GPIO_P_MODEL_MODE4_DISABLED                 // GSM_RING
                      | GPIO_P_MODEL_MODE5_DISABLED                 // GSM_DTR
                      | GPIO_P_MODEL_MODE6_DISABLED                 // BAT_STDBY
                      | GPIO_P_MODEL_MODE7_DISABLED;                // BAT_CHRG
    GPIO->P[2].MODEH  = GPIO_P_MODEH_MODE8_PUSHPULL                 // US0_CS_WIFI - Location 2
                      | GPIO_P_MODEH_MODE9_DISABLED                 // US0_CLK_WIFI - Location 2
                      | GPIO_P_MODEH_MODE10_DISABLED                // US0_MISO_WIFI - Location 2
                      | GPIO_P_MODEH_MODE11_DISABLED                // US0_MOSI_WIFI - Location 2
                      | GPIO_P_MODEH_MODE12_DISABLED                // NC
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[2].DOUT   = 0;
    GPIO->P[2].OVTDIS = 0;

    // Port D
    GPIO->P[3].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[3].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // US1_MOSI_TFT - Location 1
                      | GPIO_P_MODEL_MODE1_DISABLED                 // US1_MISO_TFT - Location 1
                      | GPIO_P_MODEL_MODE2_DISABLED                 // US1_CLK_TFT - Location 1
                      | GPIO_P_MODEL_MODE3_DISABLED                 // US1_CS_TFT - Location 1
                      | GPIO_P_MODEL_MODE4_DISABLED                 // 5V0_SENSE
                      | GPIO_P_MODEL_MODE5_DISABLED                 // 4V2_SENSE
                      | GPIO_P_MODEL_MODE6_DISABLED                 // VBAT_SENSE
                      | GPIO_P_MODEL_MODE7_DISABLED;                // VIN_SENSE
    GPIO->P[3].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NR
                      | GPIO_P_MODEH_MODE9_DISABLED                 // QSPI0_DQ0 - Location 0
                      | GPIO_P_MODEH_MODE10_DISABLED                // QSPI0_DQ1 - Location 0
                      | GPIO_P_MODEH_MODE11_DISABLED                // QSPI0_DQ2 - Location 0
                      | GPIO_P_MODEH_MODE12_DISABLED                // QSPI0_DQ3 - Location 0
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[3].DOUT   = 0;
    GPIO->P[3].OVTDIS = 0;

    // Port E
    GPIO->P[4].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[4].MODEL  = GPIO_P_MODEL_MODE0_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE1_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE2_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE3_DISABLED                 // WIFI_IRQ
                      | GPIO_P_MODEL_MODE4_DISABLED                 // WIFI_EN
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE6_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE7_DISABLED;                // NR
    GPIO->P[4].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NR
                      | GPIO_P_MODEH_MODE9_DISABLED                 // CCS811_IRQ
                      | GPIO_P_MODEH_MODE10_DISABLED                // CCS811_WAKE
                      | GPIO_P_MODEH_MODE11_DISABLED                // CCS811_RESET
                      | GPIO_P_MODEH_MODE12_DISABLED                // I2C0_SDA_SENS - Location 6
                      | GPIO_P_MODEH_MODE13_DISABLED                // I2C0_SCL_SENS - Location 6
                      | GPIO_P_MODEH_MODE14_DISABLED                // NR
                      | GPIO_P_MODEH_MODE15_DISABLED;               // MAG_ALERT
    GPIO->P[4].DOUT   = 0;
    GPIO->P[4].OVTDIS = 0;

    // Port F
    GPIO->P[5].CTRL   = GPIO_P_CTRL_DRIVESTRENGTHALT_STRONG | (6 << _GPIO_P_CTRL_SLEWRATEALT_SHIFT)
                      | GPIO_P_CTRL_DRIVESTRENGTH_STRONG | (6 << _GPIO_P_CTRL_SLEWRATE_SHIFT);
    GPIO->P[5].MODEL  = GPIO_P_MODEL_MODE0_PUSHPULL                 // DBG_SWCLK - Location 0
                      | GPIO_P_MODEL_MODE1_PUSHPULL                 // DBG_SWDIO - Location 0
                      | GPIO_P_MODEL_MODE2_PUSHPULL                 // DBG_SWO - Location 0
                      | GPIO_P_MODEL_MODE3_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE4_DISABLED                 // NC
                      | GPIO_P_MODEL_MODE5_DISABLED                 // NR
                      | GPIO_P_MODEL_MODE6_DISABLED                 // QSPI0_SCLK - Location 0
                      | GPIO_P_MODEL_MODE7_DISABLED;                // QSPI0_CS0 - Location 0
    GPIO->P[5].MODEH  = GPIO_P_MODEH_MODE8_DISABLED                 // NR
                      | GPIO_P_MODEH_MODE9_DISABLED                 // NR
                      | GPIO_P_MODEH_MODE10_DISABLED                // NR
                      | GPIO_P_MODEH_MODE11_INPUTPULLFILTER         // VREG_ERR
                      | GPIO_P_MODEH_MODE12_DISABLED                // VIN_DET - FIXME: Hardware fix needed
                      | GPIO_P_MODEH_MODE13_DISABLED                // NC
                      | GPIO_P_MODEH_MODE14_DISABLED                // NC
                      | GPIO_P_MODEH_MODE15_DISABLED;               // NC
    GPIO->P[5].DOUT   = 0;
    GPIO->P[5].OVTDIS = 0;

    // Debugger Route
    GPIO->ROUTEPEN &= ~(GPIO_ROUTEPEN_TDIPEN | GPIO_ROUTEPEN_TDOPEN);   // Disable JTAG
    GPIO->ROUTEPEN |= GPIO_ROUTEPEN_SWVPEN;                             // Enable SWO
    GPIO->ROUTELOC0 = GPIO_ROUTELOC0_SWVLOC_LOC0;                       // SWO on PF2

    // External interrupts
    GPIO->EXTIPSELL = GPIO_EXTIPSELL_EXTIPSEL0_PORTE            // WIFI_IRQ
                    | GPIO_EXTIPSELL_EXTIPSEL1_PORTB            // BTN_1
                    | GPIO_EXTIPSELL_EXTIPSEL2_PORTB            // BTN_2
                    | GPIO_EXTIPSELL_EXTIPSEL3_PORTB            // BTN_3
                    | GPIO_EXTIPSELL_EXTIPSEL4_PORTA            // RFM_IRQ
                    | GPIO_EXTIPSELL_EXTIPSEL5_PORTA            // GSM_STATUS
                    | GPIO_EXTIPSELL_EXTIPSEL6_PORTC            // GSM_RING
                    | GPIO_EXTIPSELL_EXTIPSEL7_PORTC;           // BAT_CHRG
    GPIO->EXTIPSELH = GPIO_EXTIPSELH_EXTIPSEL8_PORTA            // GSM_RF_SYNC
                    | GPIO_EXTIPSELH_EXTIPSEL9_PORTE            // CCS811_IRQ
                    | GPIO_EXTIPSELH_EXTIPSEL10_PORTF           // VREG_ERR
                    | GPIO_EXTIPSELH_EXTIPSEL11_PORTA           // NU
                    | GPIO_EXTIPSELH_EXTIPSEL12_PORTA           // TFT_IRQ
                    | GPIO_EXTIPSELH_EXTIPSEL13_PORTE           // MAG_ALERT
                    | GPIO_EXTIPSELH_EXTIPSEL14_PORTF           // VIN_DET
                    | GPIO_EXTIPSELH_EXTIPSEL15_PORTA;          // NU

    GPIO->EXTIPINSELL = GPIO_EXTIPINSELL_EXTIPINSEL0_PIN3       // WIFI_IRQ
                      | GPIO_EXTIPINSELL_EXTIPINSEL1_PIN1       // BTN_1
                      | GPIO_EXTIPINSELL_EXTIPINSEL2_PIN2       // BTN_2
                      | GPIO_EXTIPINSELL_EXTIPINSEL3_PIN3       // BTN_3
                      | GPIO_EXTIPINSELL_EXTIPINSEL4_PIN6       // RFM_IRQ
                      | GPIO_EXTIPINSELL_EXTIPINSEL5_PIN7       // GSM_STATUS
                      | GPIO_EXTIPINSELL_EXTIPINSEL6_PIN4       // GSM_RING
                      | GPIO_EXTIPINSELL_EXTIPINSEL7_PIN7;      // BAT_CHRG
    GPIO->EXTIPINSELH = GPIO_EXTIPINSELH_EXTIPINSEL8_PIN8       // GSM_RF_SYNC
                      | GPIO_EXTIPINSELH_EXTIPINSEL9_PIN9       // CCS811_IRQ
                      | GPIO_EXTIPINSELH_EXTIPINSEL10_PIN11     // VREG_ERR
                      | GPIO_EXTIPINSELH_EXTIPINSEL11_PIN8      // NU
                      | GPIO_EXTIPINSELH_EXTIPINSEL12_PIN13     // TFT_IRQ
                      | GPIO_EXTIPINSELH_EXTIPINSEL13_PIN15     // MAG_ALERT
                      | GPIO_EXTIPINSELH_EXTIPINSEL14_PIN12     // VIN_DET
                      | GPIO_EXTIPINSELH_EXTIPINSEL15_PIN12;    // NU

    GPIO->EXTIRISE = BIT(0) | BIT(4) | BIT(8) | BIT(13) | BIT(14); // WIFI_IRQ, RFM_IRQ, GSM_RF_SYNC, MAG_ALERT, VIN_DET
    GPIO->EXTIFALL = BIT(1) | BIT(2) | BIT(3) | BIT(5) | BIT(6) | BIT(9) | BIT(10) | BIT(12); // BTN_1, BTN_2, BTN_3, GSM_STATUS, GSM_RING, CCS811_IRQ, VREG_ERR, TFT_IRQ

    GPIO->IFC = _GPIO_IFC_MASK; // Clear pending IRQs
    IRQ_CLEAR(GPIO_EVEN_IRQn); // Clear pending vector
    IRQ_CLEAR(GPIO_ODD_IRQn); // Clear pending vector
    IRQ_SET_PRIO(GPIO_EVEN_IRQn, 0, 0); // Set priority 0,0 (max)
    IRQ_SET_PRIO(GPIO_ODD_IRQn, 0, 0); // Set priority 0,0 (max)
    IRQ_ENABLE(GPIO_EVEN_IRQn); // Enable vector
    IRQ_ENABLE(GPIO_ODD_IRQn); // Enable vector
    GPIO->IEN = 0; // Enable interrupts
}

void play_sound(uint16_t usFrequency, uint32_t ulTime)
{
    static uint8_t ubInit = 0;

    if(!ubInit)
    {
        CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER3;

        TIMER3->CTRL = TIMER_CTRL_RSSCOIST | TIMER_CTRL_PRESC_DIV1 | TIMER_CTRL_CLKSEL_PRESCHFPERCLK | TIMER_CTRL_FALLA_NONE | TIMER_CTRL_RISEA_NONE | TIMER_CTRL_MODE_UP;
        TIMER3->CNT = 0x0000;

        TIMER3->CC[2].CTRL = TIMER_CC_CTRL_PRSCONF_LEVEL | TIMER_CC_CTRL_CUFOA_NONE | TIMER_CC_CTRL_COFOA_TOGGLE | TIMER_CC_CTRL_CMOA_NONE | TIMER_CC_CTRL_MODE_OUTPUTCOMPARE;

        TIMER3->ROUTELOC0 = TIMER_ROUTELOC0_CC2LOC_LOC0;
        TIMER3->ROUTEPEN |= TIMER_ROUTEPEN_CC2PEN;

        ubInit = 1;
    }

    if(!usFrequency)
    {
        TIMER3->CMD = TIMER_CMD_STOP;

        return;
    }

    TIMER3->TOP = (HFPER_CLOCK_FREQ / (usFrequency << 1)) - 1; // Double the frequency

    TIMER3->CMD = TIMER_CMD_START;

    if(!ulTime)
        return;

    delay_ms(ulTime);

    TIMER3->CMD = TIMER_CMD_STOP;
}