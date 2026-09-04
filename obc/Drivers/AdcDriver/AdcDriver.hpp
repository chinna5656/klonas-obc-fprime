/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 ADC Driver Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_ADCDRIVER_HPP_
#define OBC_DRIVERS_ADCDRIVER_HPP_

#include "obc/Drivers/AdcDriver/AdcDriverComponentAc.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"

namespace Obc {

  class AdcDriver : public AdcDriverComponentBase {

    public:

      //! Construct AdcDriver instance
      AdcDriver(const char* const compName);

      //! Destructor
      ~AdcDriver() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

    private:

      //! Handler implementation for adcSample
      Fw::Success adcSample_handler(
          FwIndexType portNum,
          U32 channel,
          U16& rawVal
      ) override;

      //! Telemetry counters
      U32 m_conversions;
      U32 m_conversionErrors;

  };

}

#endif /* OBC_DRIVERS_ADCDRIVER_HPP_ */
