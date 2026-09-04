/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * CommsCrypto Component Header
 * ============================================================================
 */

#ifndef OBC_COMMSCRYPTO_HPP_
#define OBC_COMMSCRYPTO_HPP_

#include "obc/Components/CommsCrypto/CommsCryptoComponentAc.hpp"
#include "obc/Components/CommsCrypto/tiny_aes.h"

namespace Obc {

  class CommsCrypto : public CommsCryptoComponentBase {

    public:

      friend class CommsCryptoTester;

      //! Maximum serial frame buffer length (sized strictly for 128KB SRAM)
      static constexpr FwSizeType MAX_FRAME_LEN = 256;

      //! Synchronization 4-byte preamble ("SYNC" in ASCII: 0x53 0x59 0x4E 0x43)
      static constexpr uint8_t SYNC_BYTE_0 = 0x53;
      static constexpr uint8_t SYNC_BYTE_1 = 0x59;
      static constexpr uint8_t SYNC_BYTE_2 = 0x4E;
      static constexpr uint8_t SYNC_BYTE_3 = 0x43;

      //! Message type identifiers
      enum MsgType : U8 {
        MSG_TYPE_TLM  = 0x01,
        MSG_TYPE_CMD  = 0x02,
        MSG_TYPE_PING = 0x03
      };

      //! Construct CommsCrypto instance
      CommsCrypto(const char* const compName);

      //! Destructor
      ~CommsCrypto() override = default;

      //! Initialize component
      void init(
          FwSizeType queueDepth,
          FwEnumStoreType instance = 0
      );

      //! Transmit telemetry or application payload wrapped in encrypted frame
      bool sendDownlinkFrame(U8 msgType, const uint8_t* payload, U8 payloadLen);

    private:

      // ----------------------------------------------------------------------
      // Handlers for input ports
      // ----------------------------------------------------------------------

      void comDataIn_handler(
          FwIndexType portNum,
          Fw::Buffer& buffer,
          const Drv::ByteStreamStatus& status
      ) override;

      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      // ----------------------------------------------------------------------
      // Command handlers
      // ----------------------------------------------------------------------

      void COMMS_SET_KEY_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          U32 k0,
          U32 k1,
          U32 k2,
          U32 k3
      ) override;

      void COMMS_ENABLE_ENCRYPTION_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          Fw::Enabled enable
      ) override;

      void COMMS_SEND_PING_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      // ----------------------------------------------------------------------
      // Framing & Cryptography helpers
      // ----------------------------------------------------------------------

      void processIncomingByte(uint8_t byteVal);
      void parseReceivedFrame();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      TinyAesContext m_aesCtx;
      uint8_t m_aesKey[AES_KEYLEN];
      uint8_t m_currentIv[AES_BLOCKLEN];

      bool m_encryptionEnabled;
      U16 m_txSeqId;

      // Static frame buffers
      uint8_t m_txBuffer[MAX_FRAME_LEN];
      uint8_t m_rxBuffer[MAX_FRAME_LEN];
      FwSizeType m_rxIndex;
      U8 m_syncState;
      U8 m_expectedPayloadLen;

      Fw::Buffer m_fprimeTxBuf;

      // Telemetry counters
      U32 m_downlinks;
      U32 m_uplinks;
      U32 m_decryptFailures;
      U32 m_crcErrors;

  };

}

#endif /* OBC_COMMSCRYPTO_HPP_ */
