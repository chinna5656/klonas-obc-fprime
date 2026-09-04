module Obc {

  @ Hardware driver for STM32F411 SPI buses (SPI1 for Sensors, SPI2 for MicroSD)
  passive component SpiDriver {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port to perform a synchronous full-duplex write/read over the SPI bus
    guarded input port SpiWriteRead: Drv.SpiWriteRead

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

    @ Total bytes transferred over SPI
    telemetry BytesTransferred: U32

    @ Total SPI transaction errors
    telemetry SpiErrors: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when SPI bus is initialized
    event SpiBusOpened(busId: U8) \
      severity activity high \
      id 0 \
      format "SPI Bus {} initialized"

    @ Event emitted when SPI transfer fails
    event SpiTransferError(busId: U8, status: U8) \
      severity warning high \
      id 1 \
      format "SPI Bus {} transfer error: status {}"

  }

}
