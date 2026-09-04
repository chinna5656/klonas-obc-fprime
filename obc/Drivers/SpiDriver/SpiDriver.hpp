/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 SPI Driver Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_SPIDRIVER_HPP_
#define OBC_DRIVERS_SPIDRIVER_HPP_

#include "obc/Drivers/SpiDriver/SpiDriverComponentAc.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"

namespace Obc {

  class SpiDriver : public SpiDriverComponentBase {

    public:

      //! Construct SpiDriver instance
      SpiDriver(const char* const compName);

      //! Destructor
      ~SpiDriver() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

      //! Configure SPI peripheral instance
      void configure(uint8_t busId);

    private:

      //! Handler implementation for SpiWriteRead
      Drv::SpiStatus SpiWriteRead_handler(
          FwIndexType portNum,
          Fw::Buffer& writeBuffer,
          Fw::Buffer& readBuffer
      ) override;

      //! Hardware SPI handle pointer
      SPI_HandleTypeDef* m_hspi;

      //! Hardware Bus ID (1 for SPI1 Sensors, 2 for SPI2 MicroSD)
      uint8_t m_busId;

      //! Configuration flag
      bool m_configured;

      //! Telemetry counters
      U32 m_bytesTransferred;
      U32 m_spiErrors;

  };

}

#endif /* OBC_DRIVERS_SPIDRIVER_HPP_ */
