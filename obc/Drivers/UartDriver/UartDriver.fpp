module Obc {

  @ Hardware driver for STM32F411 USART peripherals (USART1 for LoRa, USART2 for GPS)
  passive component UartDriver {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port invoked to transmit bytes out the UART interface
    guarded input port $send: Drv.ByteStreamSend

    @ Port invoked to deliver received bytes upstream
    output port $recv: Drv.ByteStreamData

    @ Signal indicating the UART driver is initialized and ready
    output port ready: Drv.ByteStreamReady

    @ Return ownership of buffer sent via $recv
    guarded input port recvReturnIn: Fw.BufferSend

    @ Allocation for received data
    output port allocate: Fw.BufferGet

    @ Deallocation of allocated buffers
    output port deallocate: Fw.BufferSend

    @ Scheduler port to poll received characters from UART ring buffer
    sync input port schedIn: Svc.Sched

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

    @ Total bytes transmitted
    telemetry BytesSent: U32

    @ Total bytes received
    telemetry BytesRecv: U32

    @ Transmission error counter
    telemetry TxErrors: U32

    @ Receive error counter
    telemetry RxErrors: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Informational event when UART is opened
    event UartOpened(portId: U8, baudRate: U32) \
      severity activity high \
      id 0 \
      format "UART port {} initialized at {} baud"

    @ Warning event when UART transmit error occurs
    event UartTxError(portId: U8, status: U8) \
      severity warning high \
      id 1 \
      format "UART port {} transmit error: status {}"

    @ Warning event when UART receive overrun occurs
    event UartRxOverrun(portId: U8) \
      severity warning high \
      id 2 \
      format "UART port {} receive buffer overrun"

  }

}
