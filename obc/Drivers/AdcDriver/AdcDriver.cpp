/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 ADC Driver Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/AdcDriver/AdcDriver.hpp"
#include <Fw/Types/Assert.hpp>

namespace Obc {

  AdcDriver::AdcDriver(const char* const compName) :
    AdcDriverComponentBase(compName),
    m_conversions(0),
    m_conversionErrors(0)
  {
  }

  void AdcDriver::init(FwEnumStoreType instance) {
    AdcDriverComponentBase::init(instance);
  }

  Fw::Success AdcDriver::adcSample_handler(
      FwIndexType portNum,
      U32 channel,
      U16& rawVal
  ) {
    FW_ASSERT(portNum == 0);
    (void)channel;

    HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK) {
      m_conversionErrors++;
      this->log_WARNING_HI_AdcConversionError(channel, static_cast<U8>(status));
      this->tlmWrite_ConversionErrors(m_conversionErrors);
      return Fw::Success::FAILURE;
    }

    status = HAL_ADC_PollForConversion(&hadc1, 10);
    if (status != HAL_OK) {
      HAL_ADC_Stop(&hadc1);
      m_conversionErrors++;
      this->log_WARNING_HI_AdcConversionError(channel, static_cast<U8>(status));
      this->tlmWrite_ConversionErrors(m_conversionErrors);
      return Fw::Success::FAILURE;
    }

    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    rawVal = static_cast<U16>(val & 0x0FFF); // 12-bit ADC result
    m_conversions++;
    this->tlmWrite_Conversions(m_conversions);

    return Fw::Success::SUCCESS;
  }

}
