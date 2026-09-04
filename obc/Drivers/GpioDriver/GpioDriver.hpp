/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 GPIO Driver Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_GPIODRIVER_HPP_
#define OBC_DRIVERS_GPIODRIVER_HPP_

#include "obc/Drivers/GpioDriver/GpioDriverComponentAc.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"

namespace Obc {

  class GpioDriver : public GpioDriverComponentBase {

    public:

      enum PinIndex : FwIndexType {
        PIN_IDX_WDT       = 0, //!< PB10: NE555P WDT trigger pulse
        PIN_IDX_PARACHUTE = 1, //!< PB9:  Thermal burn-wire gate
        PIN_IDX_CS_BNO    = 2, //!< PA4:  BNO08X IMU Chip Select
        PIN_IDX_CS_BMP    = 3, //!< PB2:  BMP280 Baro Chip Select
        PIN_IDX_CS_BME    = 4, //!< PB6:  BME680 Env Chip Select
        PIN_IDX_CS_SD     = 5, //!< PB12: MicroSD Card Chip Select
        PIN_IDX_CRASH     = 6, //!< PB8:  Crash impact sensor input
        PIN_IDX_RST_BNO   = 7  //!< PB1:  BNO08X IMU Reset pin
      };

      //! Construct GpioDriver instance
      GpioDriver(const char* const compName);

      //! Destructor
      ~GpioDriver() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

    private:

      //! Handler implementation for gpioWrite
      Drv::GpioStatus gpioWrite_handler(
          FwIndexType portNum,
          const Fw::Logic& state
      ) override;

      //! Handler implementation for gpioRead
      Drv::GpioStatus gpioRead_handler(
          FwIndexType portNum,
          Fw::Logic& state
      ) override;

      //! Helper to map index to STM32 port and pin mask
      void getPinMapping(FwIndexType portNum, GPIO_TypeDef*& port, uint16_t& pinMask);

      //! Telemetry counters
      U32 m_writes;
      U32 m_reads;

  };

}

#endif /* OBC_DRIVERS_GPIODRIVER_HPP_ */
