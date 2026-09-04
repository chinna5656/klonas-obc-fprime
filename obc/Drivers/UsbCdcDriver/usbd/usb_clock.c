/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB Clock Tree Configuration for STM32F411CEU6 (WeAct Black Pill)
 * ============================================================================
 */

#include "usb_clock.h"

#if defined(__arm__) || defined(STM32F411xE)

#define FLASH_BASE_ADDR         0x40023C00U
#define FLASH_ACR               (*(volatile uint32_t*)(FLASH_BASE_ADDR + 0x00U))
#define FLASH_ACR_LATENCY_3WS   (0x03U)
#define FLASH_ACR_PRFTEN        (1U << 8)
#define FLASH_ACR_ICEN          (1U << 9)
#define FLASH_ACR_DCEN          (1U << 10)

#define RCC_BASE_ADDR           0x40023800U
#define RCC_CR                  (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x00U))
#define RCC_PLLCFGR             (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x04U))
#define RCC_CFGR                (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x08U))

#define RCC_CR_HSEON            (1U << 16)
#define RCC_CR_HSERDY           (1U << 17)
#define RCC_CR_PLLON            (1U << 24)
#define RCC_CR_PLLRDY           (1U << 25)

#define RCC_PLLCFGR_PLLSRC_HSE  (1U << 22)

#define RCC_CFGR_SW_PLL         (0x02U)
#define RCC_CFGR_SWS_PLL        (0x08U)
#define RCC_CFGR_HPRE_DIV1      (0x00U)
#define RCC_CFGR_PPRE1_DIV2     (0x04U << 10)
#define RCC_CFGR_PPRE2_DIV1     (0x00U)

void SystemClock_Config_USB(void) {
    /* 1. Enable HSE external crystal (25 MHz) */
    RCC_CR |= RCC_CR_HSEON;
    uint32_t timeout = 50000;
    while (!(RCC_CR & RCC_CR_HSERDY) && --timeout) {
        __asm__ volatile("nop");
    }

    /* 2. Configure Flash Latency: 3 Wait States for 96 MHz @ 3.3V */
    FLASH_ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;

    /* 3. Configure PLL:
     *    HSE = 25 MHz
     *    PLL_M = 25 -> 1 MHz VCO In
     *    PLL_N = 192 -> 192 MHz VCO Out
     *    PLL_P = 2 (bits 17:16 = 00) -> 96 MHz SYSCLK
     *    PLL_Q = 4 (bits 27:24 = 0100) -> 48 MHz USB OTG FS Clock
     *    PLLSRC = 1 (HSE)
     */
    RCC_PLLCFGR = (25U << 0)               /* PLLM = 25 */
                | (192U << 6)              /* PLLN = 192 */
                | (0U << 16)               /* PLLP = 2 (00 = /2) */
                | RCC_PLLCFGR_PLLSRC_HSE   /* PLL Source = HSE */
                | (4U << 24);              /* PLLQ = 4 (48 MHz USB) */

    /* 4. Enable PLL and wait for lock */
    RCC_CR |= RCC_CR_PLLON;
    timeout = 50000;
    while (!(RCC_CR & RCC_CR_PLLRDY) && --timeout) {
        __asm__ volatile("nop");
    }

    /* 5. Configure Bus Prescalers:
     *    AHB Prescaler = /1 -> HCLK = 96 MHz
     *    APB1 Prescaler = /2 -> PCLK1 = 48 MHz (max 50 MHz)
     *    APB2 Prescaler = /1 -> PCLK2 = 96 MHz (max 100 MHz)
     */
    RCC_CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    /* 6. Switch system clock source to PLL */
    RCC_CFGR = (RCC_CFGR & ~0x03U) | RCC_CFGR_SW_PLL;
    timeout = 50000;
    while (((RCC_CFGR & 0x0CU) != RCC_CFGR_SWS_PLL) && --timeout) {
        __asm__ volatile("nop");
    }
}

uint32_t GetSystemCoreClock(void) {
    return 96000000U;
}

uint32_t GetUsbClock(void) {
    return 48000000U;
}

#else

/* Host Native Simulation / Unit Test Stub */
void SystemClock_Config_USB(void) {
    /* No-op on host Linux */
}

uint32_t GetSystemCoreClock(void) {
    return 96000000U;
}

uint32_t GetUsbClock(void) {
    return 48000000U;
}

#endif
