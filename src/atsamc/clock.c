// Code to setup peripheral clocks on the SAMD21
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // DECL_CONSTANT_STR
#include "compiler.h" // DIV_ROUND_CLOSEST
#include "internal.h" // enable_pclock

// The "generic clock generators" that are configured
#define GCLK_MAIN 0
#define CLKGEN_ULP32K 2

#define FREQ_MAIN 48000000
#define FREQ_32K 32768

// Configure a clock generator using a given source as input
static inline void
gen_clock(uint32_t clkgen_id, uint32_t flags)
{
    GCLK->GENCTRL[clkgen_id].reg = flags | GCLK_GENCTRL_GENEN;
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(1 << clkgen_id))
        ;
}

// Route a peripheral clock to a given clkgen
static inline void
route_pclock(uint32_t pclk_id, uint32_t clkgen_id)
{
    uint32_t val = GCLK_PCHCTRL_GEN(clkgen_id) | GCLK_PCHCTRL_CHEN;
    GCLK->PCHCTRL[pclk_id].reg = val;
    while (GCLK->PCHCTRL[pclk_id].reg != val)
        ;
}

// Enable a peripheral clock and power to that peripheral
void
enable_pclock(uint32_t pclk_id, uint32_t pm_id)
{
    route_pclock(pclk_id, GCLK_MAIN);
    uint32_t pm_port = pm_id / 32, pm_bit = 1 << (pm_id % 32);
    (&MCLK->APBAMASK.reg)[pm_port] |= pm_bit;
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t pclk_id)
{
    return FREQ_MAIN;
}

#if CONFIG_CLOCK_REF_X32K
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PA0,PA1");
#endif

// Initialize the clocks using an external 32K crystal
static void
clock_init_x32k(void)
{
    // Enable external 32Khz crystal
    OSC32KCTRL->XOSC32K.reg = OSC32KCTRL_XOSC32K_STARTUP(6)
                           | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K;
    OSC32KCTRL->XOSC32K.reg |= OSC32KCTRL_XOSC32K_ENABLE;
    while (!(OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY))
        ;

    // Generate 48Mhz clock on DPLL (with XOSC32K as reference)
    OSCCTRL->DPLLCTRLA.reg = 0;
    uint32_t mul = DIV_ROUND_CLOSEST(FREQ_MAIN, FREQ_32K);
    OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDR(mul - 1);
    OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_LBYPASS;
    OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    uint32_t mask = OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK;
    while ((OSCCTRL->DPLLSTATUS.reg & mask) != mask)
        ;

    // Switch main clock to DPLL clock
    gen_clock(GCLK_MAIN, GCLK_GENCTRL_SRC_DPLL96M);
}

// Initialize the clocks using an external 25M crystal
static void
clock_init_x25m(void)
{
    // Enable XOSC
    uint32_t freq_xosc = 25000000;
    OSCCTRL->XOSCCTRL.reg = (OSCCTRL_XOSCCTRL_ENABLE | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_STARTUP(3));
    while (!(OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY))
        ;

    // Generate 48Mhz clock on PLL1 (with XOSC1 as reference)
    uint32_t p1div = 50;
    uint32_t p1mul = DIV_ROUND_CLOSEST(FREQ_MAIN, freq_xosc/p1div);
    uint32_t p1ctrlb = OSCCTRL_DPLLCTRLB_DIV(p1div / 2 - 1);

    OSCCTRL->DPLLCTRLA.reg = 0;
    while (OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_ENABLE)
        ;
    OSCCTRL->DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDR(p1mul - 1);
    while (OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO)
        ;
    OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_DIV(p1div / 2 - 1)
            | OSCCTRL_DPLLCTRLB_LBYPASS
            | OSCCTRL_DPLLCTRLB_REFCLK(1);

    OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
    uint32_t mask = OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK;
    while ((OSCCTRL->DPLLSTATUS.reg & mask) != mask)
        ;

    gen_clock(GCLK_MAIN, GCLK_GENCTRL_SRC_DPLL96M);
}

// Initialize clocks from factory calibrated internal clock
static void
clock_init_internal(void)
{
    /*
     * From SAM C20/C21 family datasheet:
     * On a power reset the GCLK starts to their initial state:
     *  •  All generic clock generators disabled except:
     *      – The generator 0 (GCLK_MAIN) using OSC48M as source, with no division
     *  •  All generic clocks disabled
     *
     * We don't have to do much here at all.
     * Keeping for now since there is probably some clock fallback to be configured.
     */
}

void
SystemInit(void)
{
    // Setup flash to work with 48Mhz clock
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_HALF;

    // Reset GCLK
    GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
    while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_SWRST)
        ;

    // Init clocks
    if (CONFIG_SAMC21_CLOCK_REF_X32K)
        clock_init_x32k();
    else if (CONFIG_SAMC21_CLOCK_REF_X25M) {
        clock_init_x25m();
    } else {
        clock_init_internal();
    }

}
