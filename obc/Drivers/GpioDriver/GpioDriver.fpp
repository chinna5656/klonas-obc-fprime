module Obc {

  @ Hardware driver for STM32F411 GPIO pins (WDT, Parachute, CS lines, Crash sensor)
  passive component GpioDriver {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Array of ports to write logic state to discrete GPIO pins
    sync input port gpioWrite: [8] Drv.GpioWrite

    @ Array of ports to read logic state from discrete GPIO pins
    sync input port gpioRead: [8] Drv.GpioRead

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

    @ Total GPIO write operations
    telemetry GpioWrites: U32

    @ Total GPIO read operations
    telemetry GpioReads: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Diagnostic event when pin state changes
    event PinStateChanged(pinIndex: U8, logicState: Fw.Logic) \
      severity diagnostic \
      id 0 \
      format "GPIO index {} set to state {}"

  }

}
