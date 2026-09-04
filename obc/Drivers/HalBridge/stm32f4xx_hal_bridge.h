/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32 HAL Low-Level Hardware Bridge Header
 *
 * Microcontroller: STM32F411CEU6 (ARM Cortex-M4 @ 100 MHz)
 * Memory Limits: 512 KB Flash, 128 KB RAM (Zero dynamic allocation)
 * ============================================================================
 */

#ifndef OBC_STM32F4XX_HAL_BRIDGE_H_
#define OBC_STM32F4XX_HAL_BRIDGE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------
 * Pinout & Peripheral Mapping Definitions (KLONAS Phase-1)
 * ---------------------------------------------------------------------- */

/* SPI1 - Shared Multi-Sensor Bus (PA5 SCK, PA6 MISO, PA7 MOSI) */
#define PIN_SPI1_SCK            5   /* Port A */
#define PIN_SPI1_MISO           6   /* Port A */
#define PIN_SPI1_MOSI           7   /* Port A */

/* Sensor Chip Selects & Control on SPI1 */
#define PIN_BNO08X_CS           4   /* Port A */
#define PIN_BNO08X_INT          0   /* Port B */
#define PIN_BNO08X_RST          1   /* Port B */

#define PIN_BMP280_CS           2   /* Port B */
#define PIN_BME680_CS           6   /* Port B */

/* SPI2 - Data Logging Bus (MicroSD Card) */
#define PIN_SPI2_CS             12  /* Port B */
#define PIN_SPI2_SCK            13  /* Port B */
#define PIN_SPI2_MISO           14  /* Port B */
#define PIN_SPI2_MOSI           15  /* Port B */

/* USART1 - LoRa Transceiver (E22-900T30D @ 115200 bps) */
#define PIN_USART1_TX           9   /* Port A */
#define PIN_USART1_RX           10  /* Port A */

/* USART2 - GPS Module (Ublox NEO-M8N @ 9600 bps) */
#define PIN_USART2_TX           2   /* Port A */
#define PIN_USART2_RX           3   /* Port A */

/* Analog & Digital I/O */
#define PIN_ADC_VBAT_SENSE      0   /* Port A, ADC1 Channel 0 */
#define PIN_WDT_TRIGGER         10  /* Port B, NE555P WDT pulse */
#define PIN_CRASH_MONITOR       8   /* Port B, Impact detection */
#define PIN_PARACHUTE_BURN      9   /* Port B, Thermal burn wire MOSFET */
#define PIN_LED_HEARTBEAT       13  /* Port C, Onboard LED (Active Low on WeAct Black Pill) */

/* ----------------------------------------------------------------------
 * Heartbeat LED Control Functions (PC13)
 * ---------------------------------------------------------------------- */
void BSP_LED_Init(void);
void BSP_LED_On(void);
void BSP_LED_Off(void);
void BSP_LED_Toggle(void);

/* ----------------------------------------------------------------------
 * STM32 HAL Type Definitions (Native Bridge & Target Abstraction)
 * ---------------------------------------------------------------------- */

#if defined(__has_include) && __has_include("stm32f4xx_hal.h")
    /* Include actual vendor CMSIS and HAL when available */
    #include "stm32f4xx_hal.h"
#else
    /* Clean fallback types for native emulation & unit tests */
    typedef enum {
        HAL_OK       = 0x00U,
        HAL_ERROR    = 0x01U,
        HAL_BUSY     = 0x02U,
        HAL_TIMEOUT  = 0x03U
    } HAL_StatusTypeDef;

    typedef enum {
        GPIO_PIN_RESET = 0,
        GPIO_PIN_SET
    } GPIO_PinState;

    typedef struct {
        uint32_t Pin;
        uint32_t Mode;
        uint32_t Pull;
        uint32_t Speed;
        uint32_t Alternate;
    } GPIO_InitTypeDef;

    typedef struct {
        volatile uint32_t MODER;
        volatile uint32_t OTYPER;
        volatile uint32_t OSPEEDR;
        volatile uint32_t PUPDR;
        volatile uint32_t IDR;
        volatile uint32_t ODR;
        volatile uint32_t BSRR;
        volatile uint32_t LCKR;
        volatile uint32_t AFR[2];
    } GPIO_TypeDef;

    typedef struct {
        void* Instance;
        uint32_t ErrorCode;
    } UART_HandleTypeDef;

    typedef struct {
        void* Instance;
        uint32_t ErrorCode;
    } SPI_HandleTypeDef;

    typedef struct {
        void* Instance;
        uint32_t ErrorCode;
    } ADC_HandleTypeDef;

    /* Peripheral Handles */
    extern GPIO_TypeDef Mock_GPIOA;
    extern GPIO_TypeDef Mock_GPIOB;
    #define GPIOA (&Mock_GPIOA)
    #define GPIOB (&Mock_GPIOB)

    extern UART_HandleTypeDef huart1; /* USART1 (LoRa) */
    extern UART_HandleTypeDef huart2; /* USART2 (GPS)  */

    extern SPI_HandleTypeDef hspi1;   /* SPI1 (Sensors) */
    extern SPI_HandleTypeDef hspi2;   /* SPI2 (MicroSD) */

    extern ADC_HandleTypeDef hadc1;   /* ADC1 (Battery) */

    /* HAL Functions Prototypes */
    HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

    HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

    GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
    void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
    HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
    HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
    uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

    uint32_t HAL_GetTick(void);
    void HAL_Delay(uint32_t Delay);

    /* Simulation injection hooks for testing without real hardware */
    void HalBridge_SetMockAdcValue(uint32_t rawVal);
    void HalBridge_SetMockGpsSentence(const char* sentence);
    void HalBridge_SetMockCrashPin(GPIO_PinState state);
    uint32_t HalBridge_GetMockWdtPulseCount(void);
    uint32_t HalBridge_GetMockBurnWireActuationCount(void);
    bool HalBridge_GetMockBurnWireState(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* OBC_STM32F4XX_HAL_BRIDGE_H_ */
