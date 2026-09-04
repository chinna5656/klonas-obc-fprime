/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32 HAL Low-Level Hardware Bridge Implementation
 * ============================================================================
 */

#include "stm32f4xx_hal_bridge.h"
#include <string.h>

#if defined(__arm__) || defined(STM32F411xE)

#define RCC_AHB1ENR_REG  (*(volatile uint32_t*)0x40023830U)
#define RCC_APB2ENR_REG  (*(volatile uint32_t*)0x40023844U)

#define GPIOA_MODER      (*(volatile uint32_t*)0x40020000U)
#define GPIOA_OTYPER     (*(volatile uint32_t*)0x40020004U)
#define GPIOA_OSPEEDR    (*(volatile uint32_t*)0x40020008U)
#define GPIOA_PUPDR      (*(volatile uint32_t*)0x4002000CU)
#define GPIOA_IDR        (*(volatile uint32_t*)0x40020010U)
#define GPIOA_ODR        (*(volatile uint32_t*)0x40020014U)
#define GPIOA_BSRR       (*(volatile uint32_t*)0x40020018U)
#define GPIOA_AFRL       (*(volatile uint32_t*)0x40020020U)
#define GPIOA_AFRH       (*(volatile uint32_t*)0x40020024U)

#define GPIOB_MODER      (*(volatile uint32_t*)0x40020400U)
#define GPIOB_OTYPER     (*(volatile uint32_t*)0x40020404U)
#define GPIOB_OSPEEDR    (*(volatile uint32_t*)0x40020408U)
#define GPIOB_PUPDR      (*(volatile uint32_t*)0x4002040CU)
#define GPIOB_IDR        (*(volatile uint32_t*)0x40020410U)
#define GPIOB_ODR        (*(volatile uint32_t*)0x40020414U)
#define GPIOB_BSRR       (*(volatile uint32_t*)0x40020418U)
#define GPIOB_AFRL       (*(volatile uint32_t*)0x40020420U)
#define GPIOB_AFRH       (*(volatile uint32_t*)0x40020424U)

#define SPI1_CR1_REG     (*(volatile uint32_t*)0x40013000U)
#define SPI1_CR2_REG     (*(volatile uint32_t*)0x40013004U)
#define SPI1_SR_REG      (*(volatile uint32_t*)0x40013008U)
#define SPI1_DR_REG      (*(volatile uint32_t*)0x4001300CU)

void HalBridge_HardwareInit(void) {
    /* 1. Enable Clocks for GPIOA, GPIOB, GPIOC, SPI1 */
    RCC_AHB1ENR_REG |= (1U << 0) | (1U << 1) | (1U << 2); /* GPIOA, GPIOB, GPIOC */
    RCC_APB2ENR_REG |= (1U << 12); /* SPI1 */

    for (volatile int i = 0; i < 2000; i++) {
        __asm__ volatile("nop");
    }

    /* 2. Configure GPIOA:
     *    PA4 (BNO CS): General purpose output push-pull (01), high speed (11), default HIGH
     *    PA5 (SPI1 SCK): Alternate Function AF05 (10), high speed (11), push-pull (0)
     *    PA6 (SPI1 MISO): Alternate Function AF05 (10), high speed (11), pull-up (01)
     *    PA7 (SPI1 MOSI): Alternate Function AF05 (10), high speed (11), push-pull (0)
     */
    GPIOA_MODER &= ~(0x03U << (4 * 2));
    GPIOA_MODER |=  (0x01U << (4 * 2));
    GPIOA_OTYPER &= ~(1U << 4);
    GPIOA_OSPEEDR |= (0x03U << (4 * 2));
    GPIOA_PUPDR &= ~(0x03U << (4 * 2));
    GPIOA_BSRR = (1U << 4); /* Deselect BNO08X CS (HIGH) */

    GPIOA_MODER &= ~((0x03U << 10) | (0x03U << 12) | (0x03U << 14));
    GPIOA_MODER |=  ((0x02U << 10) | (0x02U << 12) | (0x02U << 14));
    GPIOA_OTYPER &= ~((1U << 5) | (1U << 6) | (1U << 7));
    GPIOA_OSPEEDR |= ((0x03U << 10) | (0x03U << 12) | (0x03U << 14));
    GPIOA_PUPDR &= ~((0x03U << 10) | (0x03U << 12) | (0x03U << 14));
    GPIOA_PUPDR |= (0x01U << 12); /* Pull-up on MISO PA6 */

    GPIOA_AFRL &= ~((0x0FU << 20) | (0x0FU << 24) | (0x0FU << 28));
    GPIOA_AFRL |=  ((0x05U << 20) | (0x05U << 24) | (0x05U << 28));

    /* 3. Configure GPIOB:
     *    PB0 (BNO INT): Input (00), Pull-up (01)
     *    PB1 (BNO RST): Output (01), push-pull (0), high speed (11), default HIGH
     *    PB2 (BMP CS): Output (01), push-pull (0), high speed (11), default HIGH
     *    PB6 (BME CS): Output (01), push-pull (0), high speed (11), default HIGH
     */
    GPIOB_MODER &= ~(0x03U << 0);
    GPIOB_PUPDR &= ~(0x03U << 0);
    GPIOB_PUPDR |=  (0x01U << 0);

    GPIOB_MODER &= ~(0x03U << 2);
    GPIOB_MODER |=  (0x01U << 2);
    GPIOB_OTYPER &= ~(1U << 1);
    GPIOB_OSPEEDR |= (0x03U << 2);
    GPIOB_BSRR = (1U << 1); /* BNO RST HIGH */

    GPIOB_MODER &= ~(0x03U << 4);
    GPIOB_MODER |=  (0x01U << 4);
    GPIOB_OTYPER &= ~(1U << 2);
    GPIOB_OSPEEDR |= (0x03U << 4);
    GPIOB_BSRR = (1U << 2); /* BMP CS HIGH */

    GPIOB_MODER &= ~(0x03U << 12);
    GPIOB_MODER |=  (0x01U << 12);
    GPIOB_OTYPER &= ~(1U << 6);
    GPIOB_OSPEEDR |= (0x03U << 12);
    GPIOB_BSRR = (1U << 6); /* BME CS HIGH */

    /* 4. Configure & Enable SPI1:
     *    Mode 0: CPOL = 0, CPHA = 0
     *    Master, 8-bit, MSB first, Software Slave Management (SSM=1, SSI=1)
     *    Baud Rate: /32 -> 500 kHz (at 16 MHz HSI) -> BR[2:0] = 100 (0x04 << 3)
     */
    SPI1_CR1_REG = 0;
    SPI1_CR1_REG = (1U << 9)  /* SSM = 1 */
                 | (1U << 8)  /* SSI = 1 */
                 | (1U << 2)  /* MSTR = 1 */
                 | (4U << 3); /* BR = 100 (/32 prescaler = 500 kHz) */
    SPI1_CR2_REG = 0;
    SPI1_CR1_REG |= (1U << 6); /* SPE = 1 (Enable SPI1) */

    /* 5. BNO08X Hardware Reset Pulse (Active LOW on PB1) */
    GPIOB_BSRR = (1U << (1 + 16)); /* Assert RST LOW */
    for (volatile int i = 0; i < 50000; i++) {
        __asm__ volatile("nop");
    }
    GPIOB_BSRR = (1U << 1); /* Deassert RST HIGH */
    for (volatile int i = 0; i < 200000; i++) {
        __asm__ volatile("nop");
    }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    (void)hspi;
    (void)Timeout;
    if (Size == 0) {
        return HAL_OK;
    }
    // Flush any pending RX bytes
    while (SPI1_SR_REG & (1U << 0)) {
        volatile uint8_t dummy = *(volatile uint8_t*)&SPI1_DR_REG;
        (void)dummy;
    }

    for (uint16_t i = 0; i < Size; i++) {
        uint8_t tx = (pTxData != NULL) ? pTxData[i] : 0x00;
        uint32_t to = 200000;
        while (!(SPI1_SR_REG & (1U << 1)) && --to) {
            __asm__ volatile("nop");
        }
        if (!to) return HAL_TIMEOUT;

        *(volatile uint8_t*)&SPI1_DR_REG = tx;

        to = 200000;
        while (!(SPI1_SR_REG & (1U << 0)) && --to) {
            __asm__ volatile("nop");
        }
        if (!to) return HAL_TIMEOUT;

        uint8_t rx = *(volatile uint8_t*)&SPI1_DR_REG;
        if (pRxData != NULL) {
            pRxData[i] = rx;
        }
    }

    uint32_t to = 200000;
    while ((SPI1_SR_REG & (1U << 7)) && --to) {
        __asm__ volatile("nop");
    }
    return (to > 0) ? HAL_OK : HAL_TIMEOUT;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == GPIOA) {
        return (GPIOA_IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else if (GPIOx == GPIOB) {
        return (GPIOB_IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    return GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    if (GPIOx == GPIOA) {
        if (PinState == GPIO_PIN_SET) {
            GPIOA_BSRR = GPIO_Pin;
        } else {
            GPIOA_BSRR = (uint32_t)GPIO_Pin << 16;
        }
    } else if (GPIOx == GPIOB) {
        if (PinState == GPIO_PIN_SET) {
            GPIOB_BSRR = GPIO_Pin;
        } else {
            GPIOB_BSRR = (uint32_t)GPIO_Pin << 16;
        }
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == GPIOA) {
        GPIOA_ODR ^= GPIO_Pin;
    } else if (GPIOx == GPIOB) {
        GPIOB_ODR ^= GPIO_Pin;
    }
}

UART_HandleTypeDef huart1 = { .Instance = (void*)0x40011000, .ErrorCode = 0 };
UART_HandleTypeDef huart2 = { .Instance = (void*)0x40004400, .ErrorCode = 0 };
SPI_HandleTypeDef hspi1 = { .Instance = (void*)0x40013000, .ErrorCode = 0 };
SPI_HandleTypeDef hspi2 = { .Instance = (void*)0x40003800, .ErrorCode = 0 };
ADC_HandleTypeDef hadc1 = { .Instance = (void*)0x40012000, .ErrorCode = 0 };

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout) { (void)hadc; (void)Timeout; return HAL_OK; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc) { (void)hadc; return 2606; }
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t Delay) { for (volatile uint32_t i = 0; i < Delay * 4000; i++) __asm__ volatile("nop"); }

#else

/* Host Native Simulation / Mock Stub */
GPIO_TypeDef Mock_GPIOA = {0};
GPIO_TypeDef Mock_GPIOB = {0};

UART_HandleTypeDef huart1 = { .Instance = (void*)0x40011000, .ErrorCode = 0 };
UART_HandleTypeDef huart2 = { .Instance = (void*)0x40004400, .ErrorCode = 0 };
SPI_HandleTypeDef hspi1 = { .Instance = (void*)0x40013000, .ErrorCode = 0 };
SPI_HandleTypeDef hspi2 = { .Instance = (void*)0x40003800, .ErrorCode = 0 };
ADC_HandleTypeDef hadc1 = { .Instance = (void*)0x40012000, .ErrorCode = 0 };

static uint32_t s_mockAdcValue = 2606;
static char s_mockGpsBuffer[256] = "$GPGGA,123519,3751.65,N,12213.14,W,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static uint16_t s_mockGpsPos = 0;
static GPIO_PinState s_mockCrashPin = GPIO_PIN_RESET;
static uint32_t s_mockWdtPulses = 0;
static uint32_t s_mockBurnWireActuations = 0;
static bool s_mockBurnWireActive = false;
static uint32_t s_simulatedTick = 0;

void HalBridge_HardwareInit(void) {
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart; (void)pData; (void)Size; (void)Timeout; return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)Timeout;
    if (huart == &huart2) {
        uint16_t len = (uint16_t)strlen(s_mockGpsBuffer);
        for (uint16_t i = 0; i < Size; i++) {
            if (s_mockGpsPos >= len) s_mockGpsPos = 0;
            pData[i] = (uint8_t)s_mockGpsBuffer[s_mockGpsPos++];
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    (void)hspi; (void)Timeout;
    if (pRxData != NULL) {
        for (uint16_t i = 0; i < Size; i++) {
            if (pTxData != NULL && (pTxData[i] == 0xD0)) {
                pRxData[i] = 0x58;
            } else {
                pRxData[i] = 0x00;
            }
        }
    }
    return HAL_OK;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == GPIOB && GPIO_Pin == (1U << PIN_CRASH_MONITOR)) {
        return s_mockCrashPin;
    }
    return GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    if (GPIOx == GPIOB && GPIO_Pin == (1U << PIN_WDT_TRIGGER)) {
        if (PinState == GPIO_PIN_SET) s_mockWdtPulses++;
    } else if (GPIOx == GPIOB && GPIO_Pin == (1U << PIN_PARACHUTE_BURN)) {
        if (PinState == GPIO_PIN_SET) {
            s_mockBurnWireActive = true;
            s_mockBurnWireActuations++;
        } else {
            s_mockBurnWireActive = false;
        }
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    (void)GPIOx; (void)GPIO_Pin;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc) { (void)hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout) { (void)hadc; (void)Timeout; return HAL_OK; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc) { (void)hadc; return s_mockAdcValue; }
uint32_t HAL_GetTick(void) { return s_simulatedTick += 10; }
void HAL_Delay(uint32_t Delay) { s_simulatedTick += Delay; }

void HalBridge_SetMockAdcValue(uint32_t rawVal) { s_mockAdcValue = rawVal; }
void HalBridge_SetMockGpsSentence(const char* sentence) {
    if (sentence != NULL) {
        strncpy(s_mockGpsBuffer, sentence, sizeof(s_mockGpsBuffer) - 1);
        s_mockGpsBuffer[sizeof(s_mockGpsBuffer) - 1] = '\0';
        s_mockGpsPos = 0;
    }
}
void HalBridge_SetMockCrashPin(GPIO_PinState state) { s_mockCrashPin = state; }
uint32_t HalBridge_GetMockWdtPulseCount(void) { return s_mockWdtPulses; }
uint32_t HalBridge_GetMockBurnWireActuationCount(void) { return s_mockBurnWireActuations; }
bool HalBridge_GetMockBurnWireState(void) { return s_mockBurnWireActive; }

#endif

/* ----------------------------------------------------------------------
 * Heartbeat LED Control Functions (PC13)
 * ---------------------------------------------------------------------- */
#if defined(__arm__) || defined(STM32F411xE)

#define RCC_AHB1ENR_ADDR    0x40023830U
#define GPIOC_BASE_ADDR     0x40020800U
#define GPIOC_MODER_REG     (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x00U))
#define GPIOC_OTYPER_REG    (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x04U))
#define GPIOC_OSPEEDR_REG   (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x08U))
#define GPIOC_PUPDR_REG     (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x0CU))
#define GPIOC_ODR_REG       (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x14U))
#define GPIOC_BSRR_REG      (*(volatile uint32_t*)(GPIOC_BASE_ADDR + 0x18U))

void BSP_LED_Init(void) {
    /* 1. Enable GPIOC peripheral clock (bit 2 of RCC_AHB1ENR) */
    *(volatile uint32_t*)RCC_AHB1ENR_ADDR |= (1U << 2);

    /* Stabilization delay */
    for (volatile int i = 0; i < 1000; i++) {
        __asm__ volatile("nop");
    }

    /* 2. Configure PC13 as Output: MODER13[1:0] = 01 */
    GPIOC_MODER_REG &= ~(0x03U << 26);
    GPIOC_MODER_REG |=  (0x01U << 26);

    /* 3. Output push-pull: OTYPER13 = 0 */
    GPIOC_OTYPER_REG &= ~(1U << 13);

    /* 4. Low speed: OSPEEDR13 = 00 */
    GPIOC_OSPEEDR_REG &= ~(0x03U << 26);

    /* 5. No pull-up/pull-down: PUPDR13 = 00 */
    GPIOC_PUPDR_REG &= ~(0x03U << 26);

    /* 6. Turn LED ON at boot (PC13 LOW since active-low) */
    GPIOC_BSRR_REG = (1U << (13 + 16));
}

void BSP_LED_On(void) {
    /* Active-Low: Set PC13 LOW to turn ON */
    GPIOC_BSRR_REG = (1U << (13 + 16));
}

void BSP_LED_Off(void) {
    /* Active-Low: Set PC13 HIGH to turn OFF */
    GPIOC_BSRR_REG = (1U << 13);
}

void BSP_LED_Toggle(void) {
    GPIOC_ODR_REG ^= (1U << 13);
}

#else

/* Host Native Simulation / Mock Stub */
static bool s_mockLedState = false;

void BSP_LED_Init(void) {
    s_mockLedState = true;
}

void BSP_LED_On(void) {
    s_mockLedState = true;
}

void BSP_LED_Off(void) {
    s_mockLedState = false;
}

void BSP_LED_Toggle(void) {
    s_mockLedState = !s_mockLedState;
}

#endif
