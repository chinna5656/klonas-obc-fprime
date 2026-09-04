/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * USB CDC ACM Driver Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/UsbCdcDriver/UsbCdcDriver.hpp"
#include <Fw/Types/Assert.hpp>
#include <cstring>

namespace Obc {

  UsbCdcDriver::UsbCdcDriver(const char* const compName) :
    UsbCdcDriverComponentBase(compName),
    m_fprimeRxBuffer(m_rxBuffer, sizeof(m_rxBuffer)),
    m_configured(false),
    m_bytesSent(0),
    m_bytesRecv(0),
    m_txErrors(0),
    m_rxErrors(0)
  {
    memset(m_rxBuffer, 0, sizeof(m_rxBuffer));
  }

  void UsbCdcDriver::init(FwEnumStoreType instance) {
    UsbCdcDriverComponentBase::init(instance);
  }

  void UsbCdcDriver::configure() {
    UsbCdc_Init();
    m_configured = true;

    this->log_ACTIVITY_HI_UsbOpened();

    if (this->isConnected_ready_OutputPort(0)) {
      this->ready_out(0);
    }
  }

  Drv::ByteStreamStatus UsbCdcDriver::send_handler(
      FwIndexType portNum,
      Fw::Buffer& sendBuffer
  ) {
    FW_ASSERT(portNum == 0);

    if (!m_configured) {
      m_txErrors++;
      this->tlmWrite_TxErrors(m_txErrors);
      return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    const uint8_t* data = sendBuffer.getData();
    const FwSizeType size = sendBuffer.getSize();

    if (data == nullptr || size == 0) {
      return Drv::ByteStreamStatus::OP_OK;
    }

    uint8_t status = CDC_Transmit_FS(data, static_cast<uint16_t>(size));

    if (status != 0) {
      m_txErrors++;
      this->log_WARNING_HI_UsbTxError(status);
      this->tlmWrite_TxErrors(m_txErrors);
      return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    m_bytesSent += static_cast<U32>(size);
    this->tlmWrite_BytesSent(m_bytesSent);

    return Drv::ByteStreamStatus::OP_OK;
  }

  void UsbCdcDriver::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    if (!m_configured) {
      return;
    }

    uint16_t available = CDC_Receive_Available();
    if (available == 0) {
      return;
    }

    if (available > sizeof(m_rxBuffer)) {
      available = sizeof(m_rxBuffer);
    }

    uint16_t bytesRead = CDC_Read_FS(m_rxBuffer, available);

    if (bytesRead > 0 && this->isConnected_recv_OutputPort(0)) {
      m_fprimeRxBuffer.setSize(bytesRead);
      m_bytesRecv += bytesRead;
      this->tlmWrite_BytesRecv(m_bytesRecv);
      this->recv_out(0, m_fprimeRxBuffer, Drv::ByteStreamStatus::OP_OK);
    }
  }

  void UsbCdcDriver::recvReturnIn_handler(
      FwIndexType portNum,
      Fw::Buffer& buffer
  ) {
    FW_ASSERT(portNum == 0);
    if (this->isConnected_deallocate_OutputPort(0)) {
      this->deallocate_out(0, buffer);
    }
  }

}
