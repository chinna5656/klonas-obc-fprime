/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC Interface Header (Virtual COM Port Interface)
 * ============================================================================
 */

#ifndef OBC_USBD_CDC_IF_H_
#define OBC_USBD_CDC_IF_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_RX_RING_BUFFER_SIZE  512U
#define USB_CDC_TX_BUFFER_SIZE       256U

typedef struct {
    uint32_t bitrate;
    uint8_t  format;      /* Stop bits: 0=1 stop bit, 1=1.5 stop bits, 2=2 stop bits */
    uint8_t  paritytype;  /* 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
    uint8_t  datatype;    /* Data bits: 5, 6, 7, 8 or 16 */
} USBD_CDC_LineCodingTypeDef;

/**
 * Initialize USB CDC Device peripheral, descriptors, and ring buffer
 */
void UsbCdc_Init(void);

/**
 * Non-blocking transmit of data packet to host via USB CDC Bulk IN (EP1 IN)
 * \param Buf Pointer to data buffer
 * \param Len Number of bytes to transmit
 * \return 0 on success, 1 on busy, 2 on failure
 */
uint8_t CDC_Transmit_FS(const uint8_t* Buf, uint16_t Len);

/**
 * Return number of bytes currently available in the RX ring buffer
 */
uint16_t CDC_Receive_Available(void);

/**
 * Read received bytes out of the thread-safe circular RX ring buffer
 * \param Buf Destination buffer
 * \param MaxLen Maximum bytes to read
 * \return Actual number of bytes read
 */
uint16_t CDC_Read_FS(uint8_t* Buf, uint16_t MaxLen);

/**
 * Return true if USB CDC Virtual COM Port is configured and connected
 */
bool CDC_IsConnected(void);

/**
 * Internal / Interrupt callback when new data packet arrives from host (EP1 OUT)
 */
void CDC_Receive_Callback(const uint8_t* Buf, uint16_t Len);

/**
 * Test helper: inject synthetic RX data into ring buffer
 */
void UsbCdc_InjectRxData(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* OBC_USBD_CDC_IF_H_ */
