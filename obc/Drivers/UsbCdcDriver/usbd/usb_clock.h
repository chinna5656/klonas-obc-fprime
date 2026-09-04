/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB Clock Tree Configuration for STM32F411CEU6 (WeAct Black Pill)
 *
 * External HSE Crystal: 25.0 MHz
 * Desired SYSCLK:       96.0 MHz (Cortex-M4 CPU)
 * Required USB CLK:     48.0 MHz (USB OTG FS)
 * ============================================================================
 */

#ifndef OBC_USB_CLOCK_H_
#define OBC_USB_CLOCK_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configure STM32F411 PLL clock tree from 25 MHz HSE crystal:
 * - PLL_M = 25  (VCO In = 1.0 MHz)
 * - PLL_N = 192 (VCO Out = 192.0 MHz)
 * - PLL_P = 2   (SYSCLK = 96.0 MHz)
 * - PLL_Q = 4   (USB OTG FS Clock = 48.0 MHz exact)
 * - AHB Prescaler = 1 (96 MHz)
 * - APB1 Prescaler = 2 (48 MHz)
 * - APB2 Prescaler = 1 (96 MHz)
 * - Flash Latency: 3 wait states (WS) at 3.3V
 */
void SystemClock_Config_USB(void);

/**
 * Return current System Core Clock frequency in Hz
 */
uint32_t GetSystemCoreClock(void);

/**
 * Return current USB OTG FS Clock frequency in Hz (should be 48000000)
 */
uint32_t GetUsbClock(void);

#ifdef __cplusplus
}
#endif

#endif /* OBC_USB_CLOCK_H_ */
