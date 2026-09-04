/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 UART Driver Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_UARTDRIVER_HPP_
#define OBC_DRIVERS_UARTDRIVER_HPP_

#include "obc/Drivers/UartDriver/UartDriverComponentAc.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"

namespace Obc {

  class UartDriver : public UartDriverComponentBase {

    public:

      //! Buffer size for incoming UART data chunks (sized for 128KB SRAM)
      static constexpr FwSizeType RX_BUFFER_SIZE = 128;

      //! Construct UartDriver instance
      UartDriver(const char* const compName);

      //! Destructor
      ~UartDriver() override = default;

      //! Initialize component
      void init(FwEnumStoreType instance = 0);

      //! Configure target hardware peripheral instance and baud rate
      void configure(uint8_t portId, uint32_t baudRate);

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

      //! Hardware UART handle pointer
      UART_HandleTypeDef* m_huart;

      //! Hardware Port ID (1 for USART1 LoRa, 2 for USART2 GPS)
      uint8_t m_portId;

      //! Configured baud rate
      uint32_t m_baudRate;

      //! Initialization flag
      bool m_configured;

      //! Static receive buffer for zero-allocation incoming stream
      uint8_t m_rxBuffer[RX_BUFFER_SIZE];

      //! F Prime buffer wrapper over static storage
      Fw::Buffer m_fprimeRxBuffer;

      //! Telemetry counters
      U32 m_bytesSent;
      U32 m_bytesRecv;
      U32 m_txErrors;
      U32 m_rxErrors;

  };

}

#endif /* OBC_DRIVERS_UARTDRIVER_HPP_ */
