/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * CommsCrypto Component Implementation
 * ============================================================================
 */

#include "obc/Components/CommsCrypto/CommsCrypto.hpp"
#include <Fw/Types/Assert.hpp>
#include <cstring>

namespace Obc {

  // Default NIST 128-bit flight key: 2b7e151628aed2a6abf7158809cf4f3c
  static const uint8_t DEFAULT_FLIGHT_KEY[16] = {
      0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
      0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
  };

  constexpr uint8_t CommsCrypto::SYNC_BYTE_0;
  constexpr uint8_t CommsCrypto::SYNC_BYTE_1;
  constexpr uint8_t CommsCrypto::SYNC_BYTE_2;
  constexpr uint8_t CommsCrypto::SYNC_BYTE_3;

  CommsCrypto::CommsCrypto(const char* const compName) :
    CommsCryptoComponentBase(compName),
    m_encryptionEnabled(true),
    m_txSeqId(0),
    m_rxIndex(0),
    m_syncState(0),
    m_expectedPayloadLen(0),
    m_fprimeTxBuf(m_txBuffer, sizeof(m_txBuffer)),
    m_downlinks(0),
    m_uplinks(0),
    m_decryptFailures(0),
    m_crcErrors(0)
  {
    memcpy(m_aesKey, DEFAULT_FLIGHT_KEY, AES_KEYLEN);

    // Initial deterministic IV
    for (uint8_t i = 0; i < AES_BLOCKLEN; ++i) {
      m_currentIv[i] = i;
    }

    TinyAes_Init(&m_aesCtx, m_aesKey, m_currentIv);

    memset(m_txBuffer, 0, sizeof(m_txBuffer));
    memset(m_rxBuffer, 0, sizeof(m_rxBuffer));
  }

  void CommsCrypto::init(
      FwSizeType queueDepth,
      FwEnumStoreType instance
  ) {
    CommsCryptoComponentBase::init(queueDepth, instance);
  }

  void CommsCrypto::comDataIn_handler(
      FwIndexType portNum,
      Fw::Buffer& buffer,
      const Drv::ByteStreamStatus& status
  ) {
    FW_ASSERT(portNum == 0);

    if (status != Drv::ByteStreamStatus::OP_OK) {
      return;
    }

    const uint8_t* data = buffer.getData();
    const FwSizeType size = buffer.getSize();

    if (data == nullptr || size == 0) {
      return;
    }

    for (FwSizeType i = 0; i < size; ++i) {
      this->processIncomingByte(data[i]);
    }
  }

  void CommsCrypto::COMMS_SET_KEY_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      U32 k0,
      U32 k1,
      U32 k2,
      U32 k3
  ) {
    m_aesKey[0]  = static_cast<uint8_t>(k0 >> 24);
    m_aesKey[1]  = static_cast<uint8_t>(k0 >> 16);
    m_aesKey[2]  = static_cast<uint8_t>(k0 >> 8);
    m_aesKey[3]  = static_cast<uint8_t>(k0);

    m_aesKey[4]  = static_cast<uint8_t>(k1 >> 24);
    m_aesKey[5]  = static_cast<uint8_t>(k1 >> 16);
    m_aesKey[6]  = static_cast<uint8_t>(k1 >> 8);
    m_aesKey[7]  = static_cast<uint8_t>(k1);

    m_aesKey[8]  = static_cast<uint8_t>(k2 >> 24);
    m_aesKey[9]  = static_cast<uint8_t>(k2 >> 16);
    m_aesKey[10] = static_cast<uint8_t>(k2 >> 8);
    m_aesKey[11] = static_cast<uint8_t>(k2);

    m_aesKey[12] = static_cast<uint8_t>(k3 >> 24);
    m_aesKey[13] = static_cast<uint8_t>(k3 >> 16);
    m_aesKey[14] = static_cast<uint8_t>(k3 >> 8);
    m_aesKey[15] = static_cast<uint8_t>(k3);

    TinyAes_Init(&m_aesCtx, m_aesKey, m_currentIv);

    this->log_ACTIVITY_HI_CryptoKeyUpdated();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void CommsCrypto::COMMS_ENABLE_ENCRYPTION_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      Fw::Enabled enable
  ) {
    m_encryptionEnabled = (enable == Fw::Enabled::ENABLED);
    this->tlmWrite_EncryptionEnabled(m_encryptionEnabled);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void CommsCrypto::COMMS_SEND_PING_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    const uint8_t pingPayload[4] = {'P', 'I', 'N', 'G'};
    this->sendDownlinkFrame(MSG_TYPE_PING, pingPayload, 4);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  bool CommsCrypto::sendDownlinkFrame(
      U8 msgType,
      const uint8_t* payload,
      U8 payloadLen
  ) {
    if (payload == nullptr || payloadLen == 0 || payloadLen > 180) {
      return false;
    }

    // 1. Frame Preamble ("SYNC")
    m_txBuffer[0] = SYNC_BYTE_0;
    m_txBuffer[1] = SYNC_BYTE_1;
    m_txBuffer[2] = SYNC_BYTE_2;
    m_txBuffer[3] = SYNC_BYTE_3;

    // 2. Header
    m_txBuffer[4] = msgType;
    m_txBuffer[5] = static_cast<uint8_t>((m_txSeqId >> 8) & 0xFF);
    m_txBuffer[6] = static_cast<uint8_t>(m_txSeqId & 0xFF);

    // 3. Staging payload buffer with PKCS#7 padding
    uint8_t workBuf[208];
    memcpy(workBuf, payload, payloadLen);

    size_t encLen = payloadLen;
    if (m_encryptionEnabled) {
      encLen = TinyAes_ApplyPkcs7Padding(workBuf, payloadLen, sizeof(workBuf));
      if (encLen == 0) {
        return false;
      }
    }

    m_txBuffer[7] = static_cast<uint8_t>(encLen);

    // 4. Generate next Initialization Vector (incremental nonce counter)
    for (uint8_t i = 0; i < AES_BLOCKLEN; ++i) {
      m_currentIv[i] = static_cast<uint8_t>(m_currentIv[i] + 1);
      m_txBuffer[8 + i] = m_currentIv[i];
    }

    // 5. Encrypt if enabled
    if (m_encryptionEnabled) {
      TinyAes_SetIv(&m_aesCtx, m_currentIv);
      TinyAes_EncryptCbc(&m_aesCtx, workBuf, encLen);
    }
    memcpy(&m_txBuffer[24], workBuf, encLen);

    // 6. Compute CRC-16 over Header + IV + Encrypted Payload
    size_t crcCoverageLen = 4 + AES_BLOCKLEN + encLen; // bytes 4 through (23 + encLen)
    uint16_t crc = TinyAes_ComputeCrc16(&m_txBuffer[4], crcCoverageLen);

    size_t crcPos = 24 + encLen;
    m_txBuffer[crcPos]     = static_cast<uint8_t>((crc >> 8) & 0xFF);
    m_txBuffer[crcPos + 1] = static_cast<uint8_t>(crc & 0xFF);

    size_t totalFrameLen = crcPos + 2;
    m_fprimeTxBuf.setSize(totalFrameLen);

    // 7. Transmit frame over LoRa USART1
    Drv::ByteStreamStatus status = Drv::ByteStreamStatus::OTHER_ERROR;
    if (this->isConnected_comSendOut_OutputPort(0)) {
      status = this->comSendOut_out(0, m_fprimeTxBuf);
    }

    if (status == Drv::ByteStreamStatus::OP_OK) {
      m_downlinks++;
      this->log_ACTIVITY_HI_FrameTransmitted(m_txSeqId, payloadLen);
      this->tlmWrite_FramesDownlinked(m_downlinks);
      m_txSeqId++;
      return true;
    }

    return false;
  }

  void CommsCrypto::processIncomingByte(uint8_t byteVal) {
    // State machine matching 4-byte "SYNC" preamble: 0x53, 0x59, 0x4E, 0x43
    switch (m_syncState) {
      case 0:
        if (byteVal == SYNC_BYTE_0) m_syncState = 1;
        break;
      case 1:
        m_syncState = (byteVal == SYNC_BYTE_1) ? 2 : 0;
        break;
      case 2:
        m_syncState = (byteVal == SYNC_BYTE_2) ? 3 : 0;
        break;
      case 3:
        if (byteVal == SYNC_BYTE_3) {
          m_syncState = 4;
          m_rxIndex = 4; // Preamble matched
          m_rxBuffer[0] = SYNC_BYTE_0;
          m_rxBuffer[1] = SYNC_BYTE_1;
          m_rxBuffer[2] = SYNC_BYTE_2;
          m_rxBuffer[3] = SYNC_BYTE_3;
        } else {
          m_syncState = 0;
        }
        break;
      case 4:
        // Accumulating frame body
        if (m_rxIndex < MAX_FRAME_LEN) {
          m_rxBuffer[m_rxIndex++] = byteVal;

          if (m_rxIndex == 8) {
            m_expectedPayloadLen = m_rxBuffer[7];
          }

          // Complete frame: 4 (Sync) + 4 (Hdr) + 16 (IV) + PayloadLen + 2 (CRC) = 26 + PayloadLen
          size_t expectedTotal = 26 + m_expectedPayloadLen;
          if (m_rxIndex >= expectedTotal) {
            this->parseReceivedFrame();
            m_syncState = 0;
            m_rxIndex = 0;
          }
        } else {
          // Frame buffer overflow
          m_syncState = 0;
          m_rxIndex = 0;
        }
        break;
      default:
        m_syncState = 0;
        break;
    }
  }

  void CommsCrypto::parseReceivedFrame() {
    U16 seqId = (static_cast<U16>(m_rxBuffer[5]) << 8) | m_rxBuffer[6];
    U8 encLen = m_rxBuffer[7];

    size_t crcCoverageLen = 4 + AES_BLOCKLEN + encLen;
    uint16_t computedCrc = TinyAes_ComputeCrc16(&m_rxBuffer[4], crcCoverageLen);

    size_t crcPos = 24 + encLen;
    uint16_t expectedCrc = (static_cast<uint16_t>(m_rxBuffer[crcPos]) << 8) | m_rxBuffer[crcPos + 1];

    if (computedCrc != expectedCrc) {
      m_crcErrors++;
      this->log_WARNING_HI_FrameCrcError(expectedCrc, computedCrc);
      this->tlmWrite_CrcErrors(m_crcErrors);
      return;
    }

    // Extract IV
    uint8_t rxIv[AES_BLOCKLEN];
    memcpy(rxIv, &m_rxBuffer[8], AES_BLOCKLEN);

    // Decrypt payload in work buffer
    uint8_t decPayload[208];
    memcpy(decPayload, &m_rxBuffer[24], encLen);

    size_t plainLen = encLen;
    if (m_encryptionEnabled) {
      TinyAes_SetIv(&m_aesCtx, rxIv);
      TinyAes_DecryptCbc(&m_aesCtx, decPayload, encLen);
      plainLen = TinyAes_RemovePkcs7Padding(decPayload, encLen);
      if (plainLen == 0) {
        m_decryptFailures++;
        this->log_WARNING_HI_DecryptionError(seqId);
        this->tlmWrite_DecryptionFailures(m_decryptFailures);
        return;
      }
    }

    m_uplinks++;
    this->log_ACTIVITY_HI_FrameReceived(seqId, static_cast<U8>(plainLen));
    this->tlmWrite_FramesUplinked(m_uplinks);
  }

  void CommsCrypto::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    (void)portNum;
    (void)context;
  }

}
