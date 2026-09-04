/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 GPIO Driver Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/GpioDriver/GpioDriver.hpp"
#include <Fw/Types/Assert.hpp>

namespace Obc {

  GpioDriver::GpioDriver(const char* const compName) :
    GpioDriverComponentBase(compName),
    m_writes(0),
    m_reads(0)
  {
  }

  void GpioDriver::init(FwEnumStoreType instance) {
    GpioDriverComponentBase::init(instance);
  }

  void GpioDriver::getPinMapping(FwIndexType portNum, GPIO_TypeDef*& port, uint16_t& pinMask) {
    switch (portNum) {
      case PIN_IDX_WDT:
        port = GPIOB;
        pinMask = (1U << PIN_WDT_TRIGGER); // PB10
        break;
      case PIN_IDX_PARACHUTE:
        port = GPIOB;
        pinMask = (1U << PIN_PARACHUTE_BURN); // PB9
        break;
      case PIN_IDX_CS_BNO:
        port = GPIOA;
        pinMask = (1U << PIN_BNO08X_CS); // PA4
        break;
      case PIN_IDX_CS_BMP:
        port = GPIOB;
        pinMask = (1U << PIN_BMP280_CS); // PB2
        break;
      case PIN_IDX_CS_BME:
        port = GPIOB;
        pinMask = (1U << PIN_BME680_CS); // PB6
        break;
      case PIN_IDX_CS_SD:
        port = GPIOB;
        pinMask = (1U << PIN_SPI2_CS); // PB12
        break;
      case PIN_IDX_CRASH:
        port = GPIOB;
        pinMask = (1U << PIN_CRASH_MONITOR); // PB8
        break;
      case PIN_IDX_RST_BNO:
        port = GPIOB;
        pinMask = (1U << PIN_BNO08X_RST); // PB1
        break;
      default:
        port = nullptr;
        pinMask = 0;
        break;
    }
  }

  Drv::GpioStatus GpioDriver::gpioWrite_handler(
      FwIndexType portNum,
      const Fw::Logic& state
  ) {
    FW_ASSERT(portNum < 8);

    GPIO_TypeDef* port = nullptr;
    uint16_t pinMask = 0;
    this->getPinMapping(portNum, port, pinMask);

    if (port == nullptr) {
      return Drv::GpioStatus::UNKNOWN_ERROR;
    }

    GPIO_PinState pinState = (state == Fw::Logic::HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(port, pinMask, pinState);

    m_writes++;
    this->tlmWrite_GpioWrites(m_writes);

    return Drv::GpioStatus::OP_OK;
  }

  Drv::GpioStatus GpioDriver::gpioRead_handler(
      FwIndexType portNum,
      Fw::Logic& state
  ) {
    FW_ASSERT(portNum < 8);

    GPIO_TypeDef* port = nullptr;
    uint16_t pinMask = 0;
    this->getPinMapping(portNum, port, pinMask);

    if (port == nullptr) {
      return Drv::GpioStatus::UNKNOWN_ERROR;
    }

    GPIO_PinState pinState = HAL_GPIO_ReadPin(port, pinMask);
    state = (pinState == GPIO_PIN_SET) ? Fw::Logic::HIGH : Fw::Logic::LOW;

    m_reads++;
    this->tlmWrite_GpioReads(m_reads);

    return Drv::GpioStatus::OP_OK;
  }

}
