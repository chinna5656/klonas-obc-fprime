/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC ACM Descriptors Implementation
 * ============================================================================
 */

#include "usbd_desc.h"

/* USB Standard Device Descriptor */
const uint8_t USBD_DeviceDesc[18] = {
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType = Device */
    0x00, 0x02,                 /* bcdUSB = 2.00 */
    0x02,                       /* bDeviceClass = CDC (Communications) */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    CDC_DATA_FS_MAX_PACKET_SIZE,/* bMaxPacketSize0 = 64 */
    (uint8_t)(USBD_VID & 0xFF), /* idVendor LSB (0x83) */
    (uint8_t)(USBD_VID >> 8),   /* idVendor MSB (0x04) */
    (uint8_t)(USBD_PID & 0xFF), /* idProduct LSB (0x40) */
    (uint8_t)(USBD_PID >> 8),   /* idProduct MSB (0x57) */
    0x00, 0x02,                 /* bcdDevice = 2.00 */
    0x01,                       /* Index of manufacturer string */
    0x02,                       /* Index of product string */
    0x03,                       /* Index of serial number string */
    0x01                        /* bNumConfigurations = 1 */
};

/* USB CDC ACM Configuration Descriptor (67 bytes total) */
const uint8_t USBD_ConfigDesc[67] = {
    /* Configuration Descriptor */
    0x09,                       /* bLength: Configuration Descriptor size */
    0x02,                       /* bDescriptorType: Configuration */
    0x43, 0x00,                 /* wTotalLength: 67 bytes */
    0x02,                       /* bNumInterfaces: 2 interfaces (Control + Data) */
    0x01,                       /* bConfigurationValue: Configuration 1 */
    0x00,                       /* iConfiguration: No string */
    0xC0,                       /* bmAttributes: Self Powered */
    0x32,                       /* MaxPower: 100 mA */

    /* ------------------------------------------------------------------ */
    /* Interface 0: Communication Class Interface (CDC Control)            */
    /* ------------------------------------------------------------------ */
    0x09,                       /* bLength: Interface Descriptor size */
    0x04,                       /* bDescriptorType: Interface */
    0x00,                       /* bInterfaceNumber: Number of Interface = 0 */
    0x00,                       /* bAlternateSetting: Alternate setting */
    0x01,                       /* bNumEndpoints: 1 endpoint (Notification) */
    0x02,                       /* bInterfaceClass: Communication Interface Class */
    0x02,                       /* bInterfaceSubClass: Abstract Control Model */
    0x01,                       /* bInterfaceProtocol: Common AT commands */
    0x00,                       /* iInterface: No string */

    /* Header Functional Descriptor */
    0x05,                       /* bFunctionLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x00,                       /* bDescriptorSubtype: Header Func Desc */
    0x10, 0x01,                 /* bcdCDC: spec release 1.10 */

    /* Call Management Functional Descriptor */
    0x05,                       /* bFunctionLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x01,                       /* bDescriptorSubtype: Call Management Func Desc */
    0x00,                       /* bmCapabilities: D0+D1 */
    0x01,                       /* bDataInterface: 1 */

    /* ACM Functional Descriptor */
    0x04,                       /* bFunctionLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x02,                       /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,                       /* bmCapabilities: Support Set_Line_Coding, Get_Line_Coding */

    /* Union Functional Descriptor */
    0x05,                       /* bFunctionLength */
    0x24,                       /* bDescriptorType: CS_INTERFACE */
    0x06,                       /* bDescriptorSubtype: Union func desc */
    0x00,                       /* bMasterInterface: Communication class interface (0) */
    0x01,                       /* bSlaveInterface0: Data class interface (1) */

    /* Endpoint 2 (Interrupt IN: Notifications) */
    0x07,                       /* bLength: Endpoint Descriptor size */
    0x05,                       /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                 /* bEndpointAddress: (IN2) */
    0x03,                       /* bmAttributes: Interrupt */
    CDC_CMD_PACKET_SIZE, 0x00,  /* wMaxPacketSize: 8 bytes */
    0x10,                       /* bInterval: 16 ms polling */

    /* ------------------------------------------------------------------ */
    /* Interface 1: Data Class Interface                                  */
    /* ------------------------------------------------------------------ */
    0x09,                       /* bLength: Interface Descriptor size */
    0x04,                       /* bDescriptorType: Interface */
    0x01,                       /* bInterfaceNumber: Number of Interface = 1 */
    0x00,                       /* bAlternateSetting: Alternate setting */
    0x02,                       /* bNumEndpoints: 2 endpoints (Bulk OUT + Bulk IN) */
    0x0A,                       /* bInterfaceClass: CDC Data Class */
    0x00,                       /* bInterfaceSubClass */
    0x00,                       /* bInterfaceProtocol */
    0x00,                       /* iInterface: No string */

    /* Endpoint 1 OUT (Bulk OUT: Host -> Device) */
    0x07,                       /* bLength: Endpoint Descriptor size */
    0x05,                       /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                 /* bEndpointAddress: (OUT1) */
    0x02,                       /* bmAttributes: Bulk */
    CDC_DATA_FS_MAX_PACKET_SIZE, 0x00, /* wMaxPacketSize: 64 bytes */
    0x00,                       /* bInterval: Ignore for bulk */

    /* Endpoint 1 IN (Bulk IN: Device -> Host) */
    0x07,                       /* bLength: Endpoint Descriptor size */
    0x05,                       /* bDescriptorType: Endpoint */
    CDC_IN_EP,                  /* bEndpointAddress: (IN1) */
    0x02,                       /* bmAttributes: Bulk */
    CDC_DATA_FS_MAX_PACKET_SIZE, 0x00, /* wMaxPacketSize: 64 bytes */
    0x00                        /* bInterval: Ignore for bulk */
};

/* Language ID Descriptor */
const uint8_t USBD_LangIDDesc[4] = {
    0x04,
    0x03,
    (uint8_t)(USBD_LANGID_STRING & 0xFF),
    (uint8_t)(USBD_LANGID_STRING >> 8)
};

/* String Descriptors in UTF-16LE */
static const uint8_t s_manufacturerString[30] = {
    30, 0x03,
    'K', 0, 'L', 0, 'O', 0, 'N', 0, 'A', 0, 'S', 0, ' ', 0,
    'P', 0, 'h', 0, 'a', 0, 's', 0, 'e', 0, '-', 0, '1', 0
};

static const uint8_t s_productString[44] = {
    44, 0x03,
    'O', 0, 'B', 0, 'C', 0, ' ', 0, 'F', 0, 'l', 0, 'i', 0, 'g', 0, 'h', 0, 't', 0,
    ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0
};

static const uint8_t s_serialString[26] = {
    26, 0x03,
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0,
    '0', 0, '0', 0, '0', 0, '0', 0, '1', 0, 'A', 0
};

const uint8_t* USBD_GetStringDesc(uint8_t index, uint16_t* length) {
    if (length == NULL) return NULL;
    switch (index) {
        case 0:
            *length = sizeof(USBD_LangIDDesc);
            return USBD_LangIDDesc;
        case 1:
            *length = sizeof(s_manufacturerString);
            return s_manufacturerString;
        case 2:
            *length = sizeof(s_productString);
            return s_productString;
        case 3:
            *length = sizeof(s_serialString);
            return s_serialString;
        default:
            *length = 0;
            return NULL;
    }
}
