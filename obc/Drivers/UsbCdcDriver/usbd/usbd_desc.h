/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC ACM Descriptors Header
 * ============================================================================
 */

#ifndef OBC_USBD_DESC_H_
#define OBC_USBD_DESC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USBD_VID                      0x0483U  /* STMicroelectronics */
#define USBD_PID                      0x5740U  /* Virtual COM Port */
#define USBD_LANGID_STRING            0x0409U  /* US English */

#define CDC_IN_EP                     0x81U    /* EP1 IN: Bulk transmission to Host */
#define CDC_OUT_EP                    0x01U    /* EP1 OUT: Bulk reception from Host */
#define CDC_CMD_EP                    0x82U    /* EP2 IN: Interrupt CDC commands */

#define CDC_DATA_FS_MAX_PACKET_SIZE   64U      /* Full-Speed 64-byte max bulk packet */
#define CDC_CMD_PACKET_SIZE           8U

/* USB Standard Descriptors */
extern const uint8_t USBD_DeviceDesc[18];
extern const uint8_t USBD_ConfigDesc[67];
extern const uint8_t USBD_LangIDDesc[4];

/* String Descriptors */
const uint8_t* USBD_GetStringDesc(uint8_t index, uint16_t* length);

#ifdef __cplusplus
}
#endif

#endif /* OBC_USBD_DESC_H_ */
