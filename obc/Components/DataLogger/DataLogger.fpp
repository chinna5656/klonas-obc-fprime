module Obc {

  @ MicroSD flight data logger with static double-buffered ping-pong staging
  active component DataLogger {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Input port to receive structured flight log telemetry records
    async input port logRecordIn: Obc.LogRecord

    @ Periodic schedule port to evaluate buffer thresholds and perform flush
    sync input port schedIn: Svc.Sched

    @ SPI write/read transaction port on SPI2 logging bus
    output port spiOut: Drv.SpiWriteRead

    @ Chip Select GPIO line for SparkFun MicroSD adapter (PB12)
    output port csOut: Drv.GpioWrite

    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    event port Log
    text event port LogText
    time get port Time
    telemetry port Tlm

    # ----------------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------------

    @ Force immediate flush of in-memory telemetry buffer to MicroSD card
    async command LOG_FORCE_FLUSH()

    @ Enable or disable continuous MicroSD flight logging
    async command LOG_ENABLE(
      enable: Fw.Enabled @< Enable logging flag
    )

    @ Reset logging counters and metrics
    async command LOG_CLEAR_COUNTERS()

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ Total telemetry records formatted and committed
    telemetry RecordsLogged: U32

    @ Total bytes successfully written to MicroSD
    telemetry BytesWritten: U32

    @ Total 512-byte physical sectors committed to MicroSD
    telemetry SectorsWritten: U32

    @ Current active staging buffer utilization percentage (0.0% to 100.0%)
    telemetry BufferUtilizationPct: F32

    @ MicroSD write failure counter
    telemetry SdWriteErrors: U32

    @ MicroSD card initialized status flag
    telemetry CardReady: bool

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when MicroSD card is initialized in SPI mode
    event SdCardInitialized \
      severity activity high \
      id 0 \
      format "MicroSD card initialized in SPI mode on SPI2"

    @ Event emitted when MicroSD write error occurs
    event SdWriteError(sector: U32, status: U8) \
      severity warning high \
      id 1 \
      format "MicroSD write error at sector {}: status {}"

    @ Event emitted when buffer is flushed to flash memory
    event FlightLogFlushed(sectors: U32, records: U32) \
      severity activity high \
      id 2 \
      format "Flushed flight log: {} total sectors, {} records"

    @ Event emitted when staging buffer overflows
    event BufferOverrunWarning \
      severity warning high \
      id 3 \
      format "Flight log staging buffer overflow: record dropped"

  }

}
