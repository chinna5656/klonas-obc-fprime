/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * CommsCrypto Component Unit Test Harness Implementation
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>
#include <cstring>

namespace Obc {

  // Default NIST flight key matching CommsCrypto.cpp
  const uint8_t CommsCryptoTester::DEFAULT_KEY[16] = {
      0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
      0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
  };

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  CommsCryptoTester::CommsCryptoTester() :
    CommsCryptoGTestBase("Tester", CommsCryptoTester::MAX_HISTORY_SIZE),
    component("CommsCrypto"),
    m_lastSentSize(0),
    m_sendCallCount(0),
    m_fprimeInBuf(m_rawInBuf, sizeof(m_rawInBuf))
  {
    memset(m_lastSentData, 0, sizeof(m_lastSentData));
    memset(m_rawInBuf, 0, sizeof(m_rawInBuf));
    this->initComponents();
    this->connectPorts();
  }

  CommsCryptoTester::~CommsCryptoTester() {
    this->component.deinit();
  }

  // ----------------------------------------------------------------------
  // Test helper routines
  // ----------------------------------------------------------------------

  void CommsCryptoTester::sendRawBytes(const uint8_t* data, FwSizeType size) {
    ASSERT_NE(data, nullptr);
    ASSERT_LE(size, sizeof(m_rawInBuf));

    memcpy(m_rawInBuf, data, size);
    m_fprimeInBuf.setSize(size);

    this->invoke_to_comDataIn(0, m_fprimeInBuf, Drv::ByteStreamStatus::OP_OK);
    this->component.doDispatch();
  }

  void CommsCryptoTester::injectFrame(
      U8 msgType,
      U16 seqId,
      const uint8_t* payload,
      U8 payloadLen,
      bool validCrc,
      bool encrypt,
      const uint8_t* customIv
  ) {
    ASSERT_NE(payload, nullptr);
    ASSERT_LE(payloadLen, 180);

    uint8_t frame[MAX_FRAME_LEN];
    memset(frame, 0, sizeof(frame));

    // 1. Preamble ("SYNC")
    frame[0] = CommsCrypto::SYNC_BYTE_0;
    frame[1] = CommsCrypto::SYNC_BYTE_1;
    frame[2] = CommsCrypto::SYNC_BYTE_2;
    frame[3] = CommsCrypto::SYNC_BYTE_3;

    // 2. Header
    frame[4] = msgType;
    frame[5] = static_cast<uint8_t>((seqId >> 8) & 0xFF);
    frame[6] = static_cast<uint8_t>(seqId & 0xFF);

    // 3. Staging and padding
    uint8_t workBuf[208];
    memcpy(workBuf, payload, payloadLen);
    size_t encLen = TinyAes_ApplyPkcs7Padding(workBuf, payloadLen, sizeof(workBuf));
    ASSERT_GT(encLen, 0U);

    frame[7] = static_cast<uint8_t>(encLen);

    // 4. IV
    uint8_t iv[AES_BLOCKLEN];
    if (customIv != nullptr) {
      memcpy(iv, customIv, AES_BLOCKLEN);
    } else {
      for (uint8_t i = 0; i < AES_BLOCKLEN; ++i) {
        iv[i] = static_cast<uint8_t>(0xA0 + i);
      }
    }
    memcpy(&frame[8], iv, AES_BLOCKLEN);

    // 5. Encrypt payload
    if (encrypt) {
      TinyAesContext ctx;
      TinyAes_Init(&ctx, DEFAULT_KEY, iv);
      TinyAes_EncryptCbc(&ctx, workBuf, encLen);
    }
    memcpy(&frame[24], workBuf, encLen);

    // 6. CRC-16
    size_t crcCoverageLen = 4 + AES_BLOCKLEN + encLen;
    uint16_t crc = TinyAes_ComputeCrc16(&frame[4], crcCoverageLen);
    if (!validCrc) {
      crc ^= 0xFFFF; // Corrupt CRC
    }

    size_t crcPos = 24 + encLen;
    frame[crcPos]     = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame[crcPos + 1] = static_cast<uint8_t>(crc & 0xFF);

    size_t totalLen = crcPos + 2;
    this->sendRawBytes(frame, totalLen);
  }

  bool CommsCryptoTester::triggerDownlink(
      U8 msgType,
      const uint8_t* payload,
      U8 payloadLen
  ) {
    return this->component.sendDownlinkFrame(msgType, payload, payloadLen);
  }

  void CommsCryptoTester::sendCmdSetKey(
      U32 k0,
      U32 k1,
      U32 k2,
      U32 k3,
      Fw::CmdResponse expectedResponse
  ) {
    this->cmdResponseHistory->clear();
    this->sendCmd_COMMS_SET_KEY(TEST_INSTANCE_ID, 10, k0, k1, k2, k3);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, CommsCryptoComponentBase::OPCODE_COMMS_SET_KEY, 10, expectedResponse);
  }

  void CommsCryptoTester::sendCmdEnableEncryption(
      Fw::Enabled enable,
      Fw::CmdResponse expectedResponse
  ) {
    this->cmdResponseHistory->clear();
    this->sendCmd_COMMS_ENABLE_ENCRYPTION(TEST_INSTANCE_ID, 20, enable);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, CommsCryptoComponentBase::OPCODE_COMMS_ENABLE_ENCRYPTION, 20, expectedResponse);
  }

  void CommsCryptoTester::sendCmdSendPing(
      Fw::CmdResponse expectedResponse
  ) {
    this->cmdResponseHistory->clear();
    this->sendCmd_COMMS_SEND_PING(TEST_INSTANCE_ID, 30);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, CommsCryptoComponentBase::OPCODE_COMMS_SEND_PING, 30, expectedResponse);
  }

  // ----------------------------------------------------------------------
  // Handlers for outbound ports
  // ----------------------------------------------------------------------

  Drv::ByteStreamStatus CommsCryptoTester::from_comSendOut_handler(
      FwIndexType portNum,
      Fw::Buffer& sendBuffer
  ) {
    (void)portNum;
    m_lastSentSize = sendBuffer.getSize();
    if (m_lastSentSize <= sizeof(m_lastSentData)) {
      memcpy(m_lastSentData, sendBuffer.getData(), m_lastSentSize);
    }
    m_sendCallCount++;
    return Drv::ByteStreamStatus::OP_OK;
  }

}
