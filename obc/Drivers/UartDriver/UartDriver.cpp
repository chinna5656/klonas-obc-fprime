/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 UART Driver Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/UartDriver/UartDriver.hpp"
#include <Fw/Types/Assert.hpp>

namespace Obc {

  UartDriver::UartDriver(const char* const compName) :
    UartDriverComponentBase(compName),
    m_huart(nullptr),
    m_portId(0),
    m_baudRate(0),
    m_configured(false),
    m_fprimeRxBuffer(m_rxBuffer, sizeof(m_rxBuffer)),
    m_bytesSent(0),
    m_bytesRecv(0),
    m_txErrors(0),
    m_rxErrors(0)
  {
  }

  void UartDriver::init(FwEnumStoreType instance) {
    UartDriverComponentBase::init(instance);
  }

  void UartDriver::configure(uint8_t portId, uint32_t baudRate) {
    m_portId = portId;
    m_baudRate = baudRate;

    if (m_portId == 1) {
      m_huart = &huart1; // USART1: PA9 TX, PA10 RX (LoRa @ 115200)
    } else if (m_portId == 2) {
      m_huart = &huart2; // USART2: PA2 TX, PA3 RX (GPS @ 9600)
    } else {
      m_huart = nullptr;
    }

    m_configured = (m_huart != nullptr);

    if (m_configured) {
      this->log_ACTIVITY_HI_UartOpened(m_portId, m_baudRate);
      if (this->isConnected_ready_OutputPort(0)) {
        this->ready_out(0);
      }
    }
  }

  Drv::ByteStreamStatus UartDriver::send_handler(
      FwIndexType portNum,
      Fw::Buffer& sendBuffer
  ) {
    FW_ASSERT(portNum == 0);

    if (!m_configured || m_huart == nullptr) {
      m_txErrors++;
      this->tlmWrite_TxErrors(m_txErrors);
      return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    const uint8_t* data = sendBuffer.getData();
    const FwSizeType size = sendBuffer.getSize();

    if (data == nullptr || size == 0) {
      return Drv::ByteStreamStatus::OP_OK;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        m_huart,
        data,
        static_cast<uint16_t>(size),
        100
    );

    if (status != HAL_OK) {
      m_txErrors++;
      this->log_WARNING_HI_UartTxError(m_portId, static_cast<U8>(status));
      this->tlmWrite_TxErrors(m_txErrors);
      return Drv::ByteStreamStatus::OTHER_ERROR;
    }

    m_bytesSent += static_cast<U32>(size);
    this->tlmWrite_BytesSent(m_bytesSent);

    return Drv::ByteStreamStatus::OP_OK;
  }

  void UartDriver::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    if (!m_configured || m_huart == nullptr) {
      return;
    }

    // Polled receive into static buffer from peripheral
    uint16_t bytesRead = 0;
    HAL_StatusTypeDef status = HAL_UART_Receive(
        m_huart,
        m_rxBuffer,
        sizeof(m_rxBuffer),
        10 // 10ms bounded timeout
    );

    if (status == HAL_OK) {
      bytesRead = sizeof(m_rxBuffer);
    }

    if (bytesRead > 0 && this->isConnected_recv_OutputPort(0)) {
      m_fprimeRxBuffer.setSize(bytesRead);
      m_bytesRecv += bytesRead;
      this->tlmWrite_BytesRecv(m_bytesRecv);
      this->recv_out(0, m_fprimeRxBuffer, Drv::ByteStreamStatus::OP_OK);
    }
  }

  void UartDriver::recvReturnIn_handler(
      FwIndexType portNum,
      Fw::Buffer& buffer
  ) {
    FW_ASSERT(portNum == 0);
    if (this->isConnected_deallocate_OutputPort(0)) {
      this->deallocate_out(0, buffer);
    }
  }

}
