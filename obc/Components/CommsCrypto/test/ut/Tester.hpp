/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * CommsCrypto Component Unit Test Harness Header
 * ============================================================================
 */

#ifndef OBC_COMMSCRYPTO_TESTER_HPP_
#define OBC_COMMSCRYPTO_TESTER_HPP_

#include "obc/Components/CommsCrypto/CommsCrypto.hpp"
#include "CommsCryptoGTestBase.hpp"
#include "gtest/gtest.h"

namespace Obc {

  class CommsCryptoTester : public CommsCryptoGTestBase, public ::testing::Test {

    public:

      // Maximum history storage for port calls, events, and telemetry
      static constexpr U32 MAX_HISTORY_SIZE = 100;
      // Test instance ID supplied to the component
      static constexpr FwEnumStoreType TEST_INSTANCE_ID = 0;
      // Queue depth supplied to active component queue
      static constexpr FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 10;
      // Maximum frame buffer size
      static constexpr FwSizeType MAX_FRAME_LEN = CommsCrypto::MAX_FRAME_LEN;
      // Default NIST 128-bit key matching CommsCrypto.cpp
      static const uint8_t DEFAULT_KEY[16];

      //! Construct test harness
      CommsCryptoTester();

      //! Destructor
      ~CommsCryptoTester() override;

      // ----------------------------------------------------------------------
      // Test helper routines
      // ----------------------------------------------------------------------

      //! Feed raw byte stream directly to the comDataIn input port
      void sendRawBytes(const uint8_t* data, FwSizeType size);

      //! Synthesize, frame, optionally encrypt, and inject an uplink packet
      void injectFrame(
          U8 msgType,
          U16 seqId,
          const uint8_t* payload,
          U8 payloadLen,
          bool validCrc = true,
          bool encrypt = true,
          const uint8_t* customIv = nullptr
      );

      //! Trigger downlink transmission via component helper
      bool triggerDownlink(
          U8 msgType,
          const uint8_t* payload,
          U8 payloadLen
      );

      //! Dispatch COMMS_SET_KEY command
      void sendCmdSetKey(
          U32 k0,
          U32 k1,
          U32 k2,
          U32 k3,
          Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK
      );

      //! Dispatch COMMS_ENABLE_ENCRYPTION command
      void sendCmdEnableEncryption(
          Fw::Enabled enable,
          Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK
      );

      //! Dispatch COMMS_SEND_PING command
      void sendCmdSendPing(
          Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK
      );

      // ----------------------------------------------------------------------
      // Inspection accessors for verification
      // ----------------------------------------------------------------------

      FwSizeType getLastSentSize() const { return m_lastSentSize; }
      const uint8_t* getLastSentData() const { return m_lastSentData; }
      U32 getSendCallCount() const { return m_sendCallCount; }

    private:

      // ----------------------------------------------------------------------
      // Handlers for outbound ports from component under test
      // ----------------------------------------------------------------------

      //! Intercept transmission to USART1 LoRa driver
      Drv::ByteStreamStatus from_comSendOut_handler(
          FwIndexType portNum,
          Fw::Buffer& sendBuffer
      ) override;

      // ----------------------------------------------------------------------
      // Harness initialization
      // ----------------------------------------------------------------------

      void connectPorts();
      void initComponents();

    private:

      // Component under test
      CommsCrypto component;

      // Mock receiver storage for transmitted frames
      uint8_t m_lastSentData[MAX_FRAME_LEN];
      FwSizeType m_lastSentSize;
      U32 m_sendCallCount;

      // Inbound synthetic packet buffer
      uint8_t m_rawInBuf[MAX_FRAME_LEN];
      Fw::Buffer m_fprimeInBuf;

  };

  typedef CommsCryptoTester Tester;

}

#endif /* OBC_COMMSCRYPTO_TESTER_HPP_ */
