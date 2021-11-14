// Code to setup clocks and gpio on stm32f2/stm32f4
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/irq.h" // irq_disable
#include "board/usb_cdc.h" // usb_request_bootloader
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main

#define FREQ_PERIPH_DIV 1
#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV)

// NOTE: If you are using CMSIS, the registers can also be
// accessed through CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk
#define HALT_IF_DEBUGGING()                              \
  do {                                                   \
    if ((*(volatile uint32_t *)0xE000EDF0) & (1 << 0)) { \
      __asm("bkpt 1");                                   \
    }                                                    \
} while (0)

void
HardFault_Handler(void)
{
    HALT_IF_DEBUGGING();
}

// Enable a peripheral clock
void
enable_pclock(uint32_t periph_base)
{
    if (periph_base < AHBPERIPH_BASE) {
        //APB range
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        if (pos < 32U) {
            RCC->APBENR1 |= (1<<pos);
            RCC->APBENR1;
        } else if (pos < 64U) {
            RCC->APBENR2 |= (1<<(pos-32U));
            RCC->APBENR2;
        }
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        if (pos < 32U) {
            RCC->AHBENR |= (1<<pos);
        }
    }
}

// Check if a peripheral clock has been enabled
int
is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        if (pos < 32U) {
            return RCC->APBENR1 & (1<<pos);
        } else if (periph_base < 64U) {
            return RCC->APBENR2 & (1<<(pos - 32U));
        }
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        if (pos < 32U) {
            return RCC->AHBENR & (1<<pos);
        }
    }
    return 0;
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    if ((uint32_t)regs < AHBPERIPH_BASE) {
        uint32_t rcc_pos = ((uint32_t)regs - APBPERIPH_BASE) / 0x400;
        if (rcc_pos < 32U) {
            RCC->APBENR1 |= 1 << rcc_pos;
            RCC->APBENR1;
        } else if (rcc_pos < 64U) {
            RCC->APBENR2 |= 1 << (rcc_pos - 32U);
            RCC->APBENR2;
        }
    } else {
        uint32_t rcc_pos = ((uint32_t)regs - AHBPERIPH_BASE) / 0x400;
        if (rcc_pos < 32U) {
            RCC->AHBENR |= 1 << rcc_pos;
            RCC->AHBENR;
        }
    }

}

#define STM_OSPEED 0x2 // ~50Mhz at 40pF on STM32F4 (~25Mhz at 40pF on STM32F2)

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf, od = mode >> 8;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (STM_OSPEED << m_shift);
}

#define USB_BOOT_FLAG_ADDR (CONFIG_RAM_START + CONFIG_RAM_SIZE - 4096)
#define USB_BOOT_FLAG 0x55534220424f4f54 // "USB BOOT"

// Handle USB reboot requests
void
usb_request_bootloader(void)
{
    irq_disable();
    if (CONFIG_STM32_FLASH_START_4000) {
        // HID Bootloader
        RCC->APBENR1 |= RCC_APBENR1_PWREN;
        RCC->APBENR1;
        PWR->CR1 |= PWR_CR1_DBP;
        // HID Bootloader magic key
        TAMP->BKP4R = 0x424C;
        PWR->CR1 &= ~PWR_CR1_DBP;
    } else {
        // System DFU Bootloader
        *(uint64_t*)USB_BOOT_FLAG_ADDR = USB_BOOT_FLAG;
    }
    NVIC_SystemReset();
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PC14,PC15");
#endif

DECL_CONSTANT_STR("RESERVE_PINS_swd", "PA13,PA14");

// Clock configuration
static void
enable_clock_stm32g0x(void)
{
    RCC->CR &= ~RCC_CR_PLLON;
    // Wait for PLL to stop
    while (RCC->CR & RCC_CR_PLLRDY)
        ;

    uint32_t pll_base = 4000000, pll_freq = CONFIG_CLOCK_FREQ * 2;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Configure 4Mhz PLL base from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_REF_FREQ / pll_base;
        RCC->CR |= RCC_CR_HSEON;
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE | (div << RCC_PLLCFGR_PLLM_Pos);
    } else {
        // Configure 4Mhz PLL base from internal 16Mhz oscillator (HSI)
        uint32_t div = 16000000 / pll_base;
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI | (div << RCC_PLLCFGR_PLLM_Pos);
    }
    // Multiply pll_base up to target freq.
    RCC->PLLCFGR |= ((pll_freq/pll_base) << RCC_PLLCFGR_PLLN_Pos)
                    | ((pll_freq / CONFIG_CLOCK_FREQ) << RCC_PLLCFGR_PLLP_Pos)
                    | ((pll_freq / CONFIG_CLOCK_FREQ) << RCC_PLLCFGR_PLLR_Pos)
                    | (1 << RCC_PLLCFGR_PLLQ_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLPEN;
    RCC->CR |= RCC_CR_PLLON;
}


// Main clock setup called at chip startup
static void
clock_setup(void)
{
    // Set flash latency
    FLASH->ACR |= (FLASH_ACR_LATENCY_2 | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN);

    // Configure and enable PLL
    enable_clock_stm32g0x();

    // Wait for PLL lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Switch system clock to PLL
    if (FREQ_PERIPH_DIV == 1)
        RCC->CFGR = (0U << RCC_CFGR_PPRE_Pos) | (2 << RCC_CFGR_SW_Pos);
    else
        RCC->CFGR = (4<<RCC_CFGR_PPRE_Pos) | (2 << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk)>>RCC_CFGR_SWS_Pos != 2)
        ;
}


/**
 * Configure boot options so the system bootloader and SWD can work.
 * @return
 */
void
flash_setup(void) {
    FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL & ~FLASH_OPTR_nBOOT1;
    FLASH->OPTR &= ~FLASH_OPTR_RAM_PARITY_CHECK; // Clear enables parity
    FLASH->OPTR |= FLASH_OPTR_NRST_MODE_0 | FLASH_OPTR_NRST_MODE_1;
}

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{

    if (CONFIG_USBSERIAL && *(uint64_t*)USB_BOOT_FLAG_ADDR == USB_BOOT_FLAG) {
        *(uint64_t*)USB_BOOT_FLAG_ADDR = 0;
        uint32_t *sysbase = (uint32_t*)0x1fff0000;
        asm volatile("mov sp, %0\n bx %1"
                     : : "r"(sysbase[0]), "r"(sysbase[1]));
    }

    // Run SystemInit() and then restore VTOR
    SystemInit();
    SCB->VTOR = (uint32_t)VectorTable;

    flash_setup();
    clock_setup();
    sched_main();
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
_Noreturn void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */