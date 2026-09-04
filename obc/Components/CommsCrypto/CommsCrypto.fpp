module Obc {

  @ Communications framing, Tiny-AES-128-CBC encryption, and LoRa packet dispatcher
  active component CommsCrypto {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Input port receiving raw byte stream from USART1 LoRa driver
    async input port comDataIn: Drv.ByteStreamData

    @ Output port transmitting framed and encrypted packets to USART1 LoRa driver
    output port comSendOut: Drv.ByteStreamSend

    @ Periodic 1Hz schedule tick for comms beacon/timeout handling
    sync input port schedIn: Svc.Sched

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

    @ Update 128-bit AES cryptographic key using four 32-bit words
    async command COMMS_SET_KEY(
      k0: U32 @< Key word 0
      k1: U32 @< Key word 1
      k2: U32 @< Key word 2
      k3: U32 @< Key word 3
    )

    @ Enable or disable payload encryption (plaintext bypass mode for test/debug)
    async command COMMS_ENABLE_ENCRYPTION(
      enable: Fw.Enabled @< Enable encryption flag
    )

    @ Transmit immediate downlink ping packet
    async command COMMS_SEND_PING()

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ Total framed downlink packets transmitted over LoRa
    telemetry FramesDownlinked: U32

    @ Total valid uplink telecommand packets received
    telemetry FramesUplinked: U32

    @ Total cryptographic decryption or padding verification failures
    telemetry DecryptionFailures: U32

    @ Total Frame Check Sequence (CRC-16) validation errors
    telemetry CrcErrors: U32

    @ Encryption active status flag
    telemetry EncryptionEnabled: bool

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when a frame is packaged, encrypted, and transmitted
    event FrameTransmitted(seqId: U16, payloadBytes: U8) \
      severity activity high \
      id 0 \
      format "Downlinked frame seq {} ({} bytes payload)"

    @ Event emitted when a valid telecommand frame is received and decrypted
    event FrameReceived(seqId: U16, payloadBytes: U8) \
      severity activity high \
      id 1 \
      format "Uplink frame received seq {} ({} bytes payload)"

    @ Event emitted when decryption or padding verification fails
    event DecryptionError(seqId: U16) \
      severity warning high \
      id 2 \
      format "Decryption or padding failure on frame seq {}"

    @ Event emitted when CRC-16 check fails
    event FrameCrcError(expectedCrc: U16, computedCrc: U16) \
      severity warning high \
      id 3 \
      format "Frame CRC mismatch: expected {}, computed {}"

    @ Event emitted when cryptographic key is successfully updated
    event CryptoKeyUpdated \
      severity activity high \
      id 4 \
      format "AES-128 cryptographic key updated successfully"

  }

}
