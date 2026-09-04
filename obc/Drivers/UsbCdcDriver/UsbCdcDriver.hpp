/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC ACM Driver Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_USBCDCDRIVER_HPP_
#define OBC_DRIVERS_USBCDCDRIVER_HPP_

#include "obc/Drivers/UsbCdcDriver/UsbCdcDriverComponentAc.hpp"
#include "obc/Drivers/UsbCdcDriver/usbd/usbd_cdc_if.h"

namespace Obc {

  class UsbCdcDriver : public UsbCdcDriverComponentBase {

    public:

      //! Buffer size for incoming USB CDC data chunks (sized for 128KB SRAM)
      static constexpr FwSizeType RX_BUFFER_SIZE = 128;

      //! Construct UsbCdcDriver instance
      UsbCdcDriver(const char* const compName);

      //! Destructor
      ~UsbCdcDriver() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

      //! Configure and initialize target USB CDC device hardware
      void configure();

    private:

      //! Handler implementation for send
      Drv::ByteStreamStatus send_handler(
          FwIndexType portNum,
          Fw::Buffer& sendBuffer
      ) override;

      //! Handler implementation for schedIn
      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      //! Handler implementation for recvReturnIn
      void recvReturnIn_handler(
          FwIndexType portNum,
          Fw::Buffer& buffer
      ) override;

      uint8_t m_rxBuffer[RX_BUFFER_SIZE];
      Fw::Buffer m_fprimeRxBuffer;

      bool m_configured;
      U32 m_bytesSent;
      U32 m_bytesRecv;
      U32 m_txErrors;
      U32 m_rxErrors;

  };

}

#endif /* OBC_DRIVERS_USBCDCDRIVER_HPP_ */
