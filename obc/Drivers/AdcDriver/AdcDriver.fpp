module Obc {

  @ Hardware driver for STM32F411 ADC1 (Battery voltage sense on PA0)
  passive component AdcDriver {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port to sample analog-to-digital channel
    guarded input port adcSample: Obc.AdcSample

    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    event port Log
    text event port LogText
    time get port Time
    telemetry port Tlm

    # ----------------------------------------------------------------------
    # Telemetry
    # ----------------------------------------------------------------------

    @ Total ADC conversions executed
    telemetry Conversions: U32

    @ Total ADC conversion errors
    telemetry ConversionErrors: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Warning event emitted when conversion fails
    event AdcConversionError(channel: U32, status: U8) \
      severity warning high \
      id 0 \
      format "ADC channel {} conversion failed with status {}"

  }

}
