/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB Device Low-Level Configuration Header
 * ============================================================================
 */

#ifndef OBC_USBD_CONF_H_
#define OBC_USBD_CONF_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize low-level USB OTG FS hardware:
 * - GPIO PA11 (DM) and PA12 (DP) in AF10
 * - USB OTG FS peripheral clock in RCC_AHB2ENR
 * - USB Core soft disconnect / connect
 */
void USBD_LL_Init(void);

/**
 * Low-level endpoint transmit
 */
uint8_t USBD_LL_Transmit(uint8_t ep_addr, const uint8_t* pbuf, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* OBC_USBD_CONF_H_ */
