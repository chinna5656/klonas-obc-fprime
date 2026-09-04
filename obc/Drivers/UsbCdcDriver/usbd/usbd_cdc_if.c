/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC Interface Implementation (Virtual COM Port Ring Buffer & Dispatcher)
 * ============================================================================
 */

#include "usbd_cdc_if.h"
#include "usb_clock.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include <string.h>

/* Circular Ring Buffer for Incoming Telemetry & Commands */
static uint8_t s_rxRingBuffer[USB_CDC_RX_RING_BUFFER_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;

/* Transmit Working Buffer */
static uint8_t s_txBuffer[USB_CDC_TX_BUFFER_SIZE];
static volatile bool s_txBusy = false;
static volatile bool s_connected = false;

/* Active Line Coding (115200 8N1 default) */
static USBD_CDC_LineCodingTypeDef s_lineCoding = {
    115200U, /* Baud rate */
    0x00U,   /* 1 Stop bit */
    0x00U,   /* None parity */
    0x08U    /* 8 Data bits */
};

void UsbCdc_Init(void) {
    s_rxHead = 0;
    s_rxTail = 0;
    s_txBusy = false;
    memset(s_rxRingBuffer, 0, sizeof(s_rxRingBuffer));
    memset(s_txBuffer, 0, sizeof(s_txBuffer));

    /* Configure 48 MHz USB clock tree */
    SystemClock_Config_USB();

    /* Initialize low-level USB OTG FS hardware */
    USBD_LL_Init();

    s_connected = true;
}

uint8_t CDC_Transmit_FS(const uint8_t* Buf, uint16_t Len) {
    if (Buf == NULL || Len == 0) {
        return 0; /* OK */
    }

    if (Len > USB_CDC_TX_BUFFER_SIZE) {
        Len = USB_CDC_TX_BUFFER_SIZE;
    }

    if (s_txBusy) {
        return 1; /* Busy */
    }

    s_txBusy = true;
    memcpy(s_txBuffer, Buf, Len);

    /* Dispatch to low-level hardware endpoint or mock */
    uint8_t result = USBD_LL_Transmit(CDC_IN_EP, s_txBuffer, Len);
    s_txBusy = false;

    return (result == 0) ? 0 : 2;
}

uint16_t CDC_Receive_Available(void) {
    uint16_t head = s_rxHead;
    uint16_t tail = s_rxTail;

    if (head >= tail) {
        return head - tail;
    } else {
        return (USB_CDC_RX_RING_BUFFER_SIZE - tail) + head;
    }
}

uint16_t CDC_Read_FS(uint8_t* Buf, uint16_t MaxLen) {
    if (Buf == NULL || MaxLen == 0) {
        return 0;
    }

    uint16_t bytesRead = 0;
    while ((s_rxTail != s_rxHead) && (bytesRead < MaxLen)) {
        Buf[bytesRead++] = s_rxRingBuffer[s_rxTail];
        s_rxTail = (s_rxTail + 1) % USB_CDC_RX_RING_BUFFER_SIZE;
    }

    return bytesRead;
}

bool CDC_IsConnected(void) {
    return s_connected;
}

void CDC_Receive_Callback(const uint8_t* Buf, uint16_t Len) {
    if (Buf == NULL || Len == 0) {
        return;
    }

    for (uint16_t i = 0; i < Len; i++) {
        uint16_t nextHead = (s_rxHead + 1) % USB_CDC_RX_RING_BUFFER_SIZE;
        if (nextHead != s_rxTail) {
            s_rxRingBuffer[s_rxHead] = Buf[i];
            s_rxHead = nextHead;
        } else {
            /* Ring buffer full: overflow drops oldest or newest */
            break;
        }
    }
}

void UsbCdc_InjectRxData(const uint8_t* data, uint16_t len) {
    CDC_Receive_Callback(data, len);
}
