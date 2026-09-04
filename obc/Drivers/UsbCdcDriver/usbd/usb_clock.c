/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB Clock Tree Configuration for STM32F411CEU6 (WeAct Black Pill)
 * ============================================================================
 */

#include "usb_clock.h"

uint32_t SystemCoreClock = 16000000U;

#if defined(__arm__) || defined(STM32F411xE)

#define FLASH_BASE_ADDR         0x40023C00U
#define FLASH_ACR               (*(volatile uint32_t*)(FLASH_BASE_ADDR + 0x00U))
#define FLASH_ACR_LATENCY_0WS   (0x00U)
#define FLASH_ACR_LATENCY_3WS   (0x03U)
#define FLASH_ACR_PRFTEN        (1U << 8)
#define FLASH_ACR_ICEN          (1U << 9)
#define FLASH_ACR_DCEN          (1U << 10)
#define FLASH_ACR_ICRST         (1U << 11)
#define FLASH_ACR_DCRST         (1U << 12)

#define RCC_BASE_ADDR           0x40023800U
#define RCC_CR                  (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x00U))
#define RCC_PLLCFGR             (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x04U))
#define RCC_CFGR                (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x08U))
#define RCC_APB1ENR             (*(volatile uint32_t*)(RCC_BASE_ADDR + 0x40U))

#define PWR_BASE_ADDR           0x40007000U
#define PWR_CR                  (*(volatile uint32_t*)(PWR_BASE_ADDR + 0x00U))

#define RCC_CR_HSION            (1U << 0)
#define RCC_CR_HSIRDY           (1U << 1)
#define RCC_CR_HSEON            (1U << 16)
#define RCC_CR_HSERDY           (1U << 17)
#define RCC_CR_PLLON            (1U << 24)
#define RCC_CR_PLLRDY           (1U << 25)

#define RCC_PLLCFGR_PLLSRC_HSE  (1U << 22)

#define RCC_CFGR_SW_HSI         (0x00U)
#define RCC_CFGR_SW_PLL         (0x02U)
#define RCC_CFGR_SWS_HSI        (0x00U)
#define RCC_CFGR_SWS_PLL        (0x08U)
#define RCC_CFGR_HPRE_DIV1      (0x00U)
#define RCC_CFGR_PPRE1_DIV1     (0x00U)
#define RCC_CFGR_PPRE1_DIV2     (0x04U << 10)
#define RCC_CFGR_PPRE2_DIV1     (0x00U)

void SystemClock_Config_USB(void) {
    /* 1. Ensure HSI (16 MHz) is enabled and stabilized */
    RCC_CR |= RCC_CR_HSION;
    uint32_t timeout = 500000U;
    while (!(RCC_CR & RCC_CR_HSIRDY) && --timeout) {
        __asm__ volatile("nop");
    }

    /* 2. Set Flash Latency to 0 wait states for 16 MHz HSI */
    FLASH_ACR = FLASH_ACR_LATENCY_0WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    /* 3. Ensure Clock Source is HSI and bus prescalers are /1 */
    RCC_CFGR = RCC_CFGR_SW_HSI | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;

    /* 4. Update SystemCoreClock */
    SystemCoreClock = 16000000U;

    /* 5. Memory barrier */
    __asm__ volatile("dsb 0xF\n" "isb 0xF" ::: "memory");
}

uint32_t GetSystemCoreClock(void) {
    return SystemCoreClock;
}

uint32_t GetUsbClock(void) {
    return 48000000U;
}

#else

/* Host Native Simulation / Unit Test Stub */
void SystemClock_Config_USB(void) {
    SystemCoreClock = 16000000U;
}

uint32_t GetSystemCoreClock(void) {
    return SystemCoreClock;
}

uint32_t GetUsbClock(void) {
    return 48000000U;
}

#endif
