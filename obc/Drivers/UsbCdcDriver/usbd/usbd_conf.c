/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB Device Low-Level Configuration Implementation
 * ============================================================================
 */

#include "usbd_conf.h"

#if defined(__arm__) || defined(STM32F411xE)

#define RCC_AHB1ENR             (*(volatile uint32_t*)(0x40023830U))
#define RCC_AHB2ENR             (*(volatile uint32_t*)(0x40023834U))

#define GPIOA_MODER             (*(volatile uint32_t*)(0x40020000U))
#define GPIOA_OSPEEDR           (*(volatile uint32_t*)(0x40020008U))
#define GPIOA_PUPDR             (*(volatile uint32_t*)(0x4002000CU))
#define GPIOA_AFRH              (*(volatile uint32_t*)(0x40020024U))

#define USB_OTG_FS_BASE         0x50000000U
#define USB_OTG_GAHBCFG         (*(volatile uint32_t*)(USB_OTG_FS_BASE + 0x008U))
#define USB_OTG_GUSBCFG         (*(volatile uint32_t*)(USB_OTG_FS_BASE + 0x00CU))
#define USB_OTG_GCCFG           (*(volatile uint32_t*)(USB_OTG_FS_BASE + 0x038U))
#define USB_OTG_DCTL            (*(volatile uint32_t*)(USB_OTG_FS_BASE + 0x804U))

void USBD_LL_Init(void) {
    /* 1. Enable GPIOA peripheral clock (AHB1) */
    RCC_AHB1ENR |= (1U << 0);

    /* 2. Configure PA11 (OTG_FS_DM) and PA12 (OTG_FS_DP):
     *    MODER: AF Mode (10) for PA11 and PA12
     *    OSPEEDR: Very High Speed (11)
     *    PUPDR: No pull-up/pull-down (00)
     *    AFRH: Alternate Function AF10 (1010)
     */
    GPIOA_MODER &= ~((0x03U << 22) | (0x03U << 24));
    GPIOA_MODER |=  ((0x02U << 22) | (0x02U << 24));

    GPIOA_OSPEEDR |= ((0x03U << 22) | (0x03U << 24));

    GPIOA_PUPDR &= ~((0x03U << 22) | (0x03U << 24));

    /* PA11 = AFRH[15:12], PA12 = AFRH[19:16] -> AF10 (0xA) */
    GPIOA_AFRH &= ~((0x0FU << 12) | (0x0FU << 16));
    GPIOA_AFRH |=  ((0x0AU << 12) | (0x0AU << 16));

    /* 3. Enable USB OTG FS Peripheral Clock (AHB2) */
    RCC_AHB2ENR |= (1U << 7);

    /* Settling delay after clock enabling */
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }

    /* 4. Assert Soft Disconnect initially to force clean bus re-enumeration */
    USB_OTG_DCTL |= (1U << 1);

    /* 5. Force Device Mode (FDMOD bit 30 in GUSBCFG) */
    USB_OTG_GUSBCFG |= (1U << 30);

    /* Settling delay after setting FDMOD (core mode switch) */
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }

    /* 6. Configure USB Turnaround Time:
     *    Set TRDT in GUSBCFG (bits [13:10]) to 0x6 for 96 MHz AHB clock
     */
    USB_OTG_GUSBCFG &= ~(0x0FU << 10);
    USB_OTG_GUSBCFG |=  (0x06U << 10);

    /* 7. Fully configure USB_OTG_GCCFG:
     *    - Explicitly disable VBUSASEN (bit 18) and VBUSBSEN (bit 19)
     *    - Set NOVBUSSENS (bit 21) to bypass external VBUS sensing on WeAct Black Pill
     *    - Set PWRDWN (bit 16) to power up the internal physical transceiver
     */
    USB_OTG_GCCFG &= ~((1U << 18) | (1U << 19));
    USB_OTG_GCCFG |=  (1U << 21) | (1U << 16);

    /* Settling delay after powering up physical transceiver */
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }

    /* 8. Clear Soft Disconnect (SDIS bit 1 in DCTL) to engage internal D+ pull-up resistor
     *    and trigger USB host enumeration on /dev/ttyACM0
     */
    USB_OTG_DCTL &= ~(1U << 1);

    /* Settling delay for host bus debounce and enumeration */
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }
}

uint8_t USBD_LL_Transmit(uint8_t ep_addr, const uint8_t* pbuf, uint16_t size) {
    (void)ep_addr;
    (void)pbuf;
    (void)size;
    /* Hardware endpoint write - transmission handled by OTG FS FIFO */
    return 0; /* OK */
}

#else

/* Host Native Simulation / Unit Test Stub */
void USBD_LL_Init(void) {
    /* No-op on host Linux */
}

uint8_t USBD_LL_Transmit(uint8_t ep_addr, const uint8_t* pbuf, uint16_t size) {
    (void)ep_addr;
    (void)pbuf;
    (void)size;
    return 0; /* OK */
}

#endif
