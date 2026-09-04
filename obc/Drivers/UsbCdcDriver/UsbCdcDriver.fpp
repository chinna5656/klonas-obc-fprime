module Obc {

  @ Hardware driver for STM32F411 USB OTG FS in CDC ACM mode (Virtual COM Port /dev/ttyACM0)
  passive component UsbCdcDriver {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Port invoked to transmit bytes out the USB CDC interface
    guarded input port $send: Drv.ByteStreamSend

    @ Port invoked to deliver received bytes upstream
    output port $recv: Drv.ByteStreamData

    @ Signal indicating the USB CDC driver is initialized and ready
    output port ready: Drv.ByteStreamReady

    @ Return ownership of buffer sent via $recv
    guarded input port recvReturnIn: Fw.BufferSend

    @ Allocation for received data
    output port allocate: Fw.BufferGet

    @ Deallocation of allocated buffers
    output port deallocate: Fw.BufferSend

    @ Scheduler port to poll received characters from USB CDC ring buffer
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

    @ Total bytes transmitted over USB CDC
    telemetry BytesSent: U32

    @ Total bytes received over USB CDC
    telemetry BytesRecv: U32

    @ Transmission error counter
    telemetry TxErrors: U32

    @ Receive error counter
    telemetry RxErrors: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Informational event when USB CDC is opened
    event UsbOpened() \
      severity activity high \
      id 0 \
      format "USB CDC Virtual COM Port (/dev/ttyACM0) initialized and enumerated"

    @ Warning event when a transmit error occurs
    event UsbTxError(status: U8) \
      severity warning high \
      id 1 \
      format "USB CDC TX error with status {}"

    @ Warning event when a receive error or overflow occurs
    event UsbRxError(status: U8) \
      severity warning high \
      id 2 \
      format "USB CDC RX error with status {}"

  }

}
