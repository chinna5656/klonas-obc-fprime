/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32 HAL Low-Level Hardware Bridge Implementation
 * ============================================================================
 */

#include "stm32f4xx_hal_bridge.h"
#include <string.h>

#if !defined(__has_include) || !__has_include("stm32f4xx_hal.h")

GPIO_TypeDef Mock_GPIOA = {0};
GPIO_TypeDef Mock_GPIOB = {0};

UART_HandleTypeDef huart1 = { .Instance = (void*)0x40011000, .ErrorCode = 0 };
UART_HandleTypeDef huart2 = { .Instance = (void*)0x40004400, .ErrorCode = 0 };

SPI_HandleTypeDef hspi1 = { .Instance = (void*)0x40013000, .ErrorCode = 0 };
SPI_HandleTypeDef hspi2 = { .Instance = (void*)0x40003800, .ErrorCode = 0 };

ADC_HandleTypeDef hadc1 = { .Instance = (void*)0x40012000, .ErrorCode = 0 };

static uint32_t s_mockAdcValue = 2606; /* ~4.2V battery with 1:2 divider: (4.2/2)/3.3 * 4095 ~= 2606 */
static char s_mockGpsBuffer[256] = "$GPGGA,123519,3751.65,N,12213.14,W,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
static uint16_t s_mockGpsPos = 0;
static GPIO_PinState s_mockCrashPin = GPIO_PIN_RESET;
static uint32_t s_mockWdtPulses = 0;
static uint32_t s_mockBurnWireActuations = 0;
static bool s_mockBurnWireActive = false;
static uint32_t s_simulatedTick = 0;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)Timeout;
    if (huart == &huart2) {
        /* Mock GPS data source */
        uint16_t len = (uint16_t)strlen(s_mockGpsBuffer);
        for (uint16_t i = 0; i < Size; i++) {
            if (s_mockGpsPos >= len) {
                s_mockGpsPos = 0;
            }
            pData[i] = (uint8_t)s_mockGpsBuffer[s_mockGpsPos++];
        }
        return HAL_OK;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    (void)hspi;
    (void)Timeout;
    if (pRxData != NULL) {
        for (uint16_t i = 0; i < Size; i++) {
            /* Mock responses for sensor IDs */
            if (pTxData != NULL && (pTxData[i] == 0xD0)) {
                /* BMP280 chip ID register = 0xD0 -> returns 0x58 */
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
        if (PinState == GPIO_PIN_SET) {
            s_mockWdtPulses++;
        }
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
    (void)GPIOx;
    (void)GPIO_Pin;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc) {
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc) {
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout) {
    (void)hadc;
    (void)Timeout;
    return HAL_OK;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc) {
    (void)hadc;
    return s_mockAdcValue;
}

uint32_t HAL_GetTick(void) {
    return s_simulatedTick += 10;
}

void HAL_Delay(uint32_t Delay) {
    s_simulatedTick += Delay;
}

void HalBridge_SetMockAdcValue(uint32_t rawVal) {
    s_mockAdcValue = rawVal;
}

void HalBridge_SetMockGpsSentence(const char* sentence) {
    if (sentence != NULL) {
        strncpy(s_mockGpsBuffer, sentence, sizeof(s_mockGpsBuffer) - 1);
        s_mockGpsBuffer[sizeof(s_mockGpsBuffer) - 1] = '\0';
        s_mockGpsPos = 0;
    }
}

void HalBridge_SetMockCrashPin(GPIO_PinState state) {
    s_mockCrashPin = state;
}

uint32_t HalBridge_GetMockWdtPulseCount(void) {
    return s_mockWdtPulses;
}

uint32_t HalBridge_GetMockBurnWireActuationCount(void) {
    return s_mockBurnWireActuations;
}

bool HalBridge_GetMockBurnWireState(void) {
    return s_mockBurnWireActive;
}

#endif /* !STM32F411xE && !USE_HAL_DRIVER */

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
