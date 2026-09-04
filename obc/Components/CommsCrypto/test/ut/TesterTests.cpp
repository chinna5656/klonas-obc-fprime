/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * CommsCrypto Component Unit Tests (GoogleTest Suite)
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>

namespace Obc {

  // ==========================================================================
  // 1. INITIAL BOOT & RESET STATE
  // ==========================================================================

  TEST_F(Tester, InitialBootState) {
    // Verify initial telemetry channels are unwritten prior to activity
    ASSERT_TLM_FramesDownlinked_SIZE(0);
    ASSERT_TLM_FramesUplinked_SIZE(0);
    ASSERT_TLM_DecryptionFailures_SIZE(0);
    ASSERT_TLM_CrcErrors_SIZE(0);
    ASSERT_TLM_EncryptionEnabled_SIZE(0);

    // Verify initial event logs are empty
    ASSERT_EVENTS_FrameTransmitted_SIZE(0);
    ASSERT_EVENTS_FrameReceived_SIZE(0);
    ASSERT_EVENTS_DecryptionError_SIZE(0);
    ASSERT_EVENTS_FrameCrcError_SIZE(0);
    ASSERT_EVENTS_CryptoKeyUpdated_SIZE(0);

    // Verify no spontaneous downlink transmissions occurred
    EXPECT_EQ(this->getSendCallCount(), 0U);
  }

  // ==========================================================================
  // 2. NOMINAL TELECOMMAND UPLINK & TELEMETRY DOWNLINK
  // ==========================================================================

  TEST_F(Tester, NominalUplinkFrameDecryption) {
    const uint8_t telecommandPayload[] = "RESET_WDT_TIMER";
    const U8 payloadLen = static_cast<U8>(sizeof(telecommandPayload) - 1);
    const U16 expectedSeqId = 0x0105;

    // Inject valid AES-128-CBC encrypted uplink frame
    this->injectFrame(
        CommsCrypto::MSG_TYPE_CMD,
        expectedSeqId,
        telecommandPayload,
        payloadLen,
        /*validCrc=*/true,
        /*encrypt=*/true
    );

    // Verify FrameReceived event emitted with exact sequence ID and decrypted payload length
    ASSERT_EVENTS_FrameReceived_SIZE(1);
    ASSERT_EVENTS_FrameReceived(0, expectedSeqId, payloadLen);

    // Verify telemetry counter incremented
    ASSERT_TLM_FramesUplinked_SIZE(1);
    ASSERT_TLM_FramesUplinked(0, 1);

    // Ensure zero error events were logged
    ASSERT_EVENTS_FrameCrcError_SIZE(0);
    ASSERT_EVENTS_DecryptionError_SIZE(0);
  }

  TEST_F(Tester, NominalDownlinkTransmission) {
    const uint8_t telemetryPayload[16] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
    };

    // Trigger downlink frame generation
    bool result = this->triggerDownlink(
        CommsCrypto::MSG_TYPE_TLM,
        telemetryPayload,
        sizeof(telemetryPayload)
    );
    EXPECT_TRUE(result);

    // Verify byte stream send invocation
    EXPECT_EQ(this->getSendCallCount(), 1U);
    EXPECT_GT(this->getLastSentSize(), 26U); // Sync (4) + Hdr (4) + IV (16) + Encrypted(16) + CRC(2) = 42

    // Verify FrameTransmitted event emitted for seq 0
    ASSERT_EVENTS_FrameTransmitted_SIZE(1);
    ASSERT_EVENTS_FrameTransmitted(0, 0, sizeof(telemetryPayload));

    // Verify FramesDownlinked telemetry updated
    ASSERT_TLM_FramesDownlinked_SIZE(1);
    ASSERT_TLM_FramesDownlinked(0, 1);

    // White-box verification of serialized wire frame
    const uint8_t* sentWire = this->getLastSentData();
    // 1. Check preamble: "SYNC" (0x53, 0x59, 0x4E, 0x43)
    EXPECT_EQ(sentWire[0], static_cast<uint8_t>(CommsCrypto::SYNC_BYTE_0));
    EXPECT_EQ(sentWire[1], static_cast<uint8_t>(CommsCrypto::SYNC_BYTE_1));
    EXPECT_EQ(sentWire[2], static_cast<uint8_t>(CommsCrypto::SYNC_BYTE_2));
    EXPECT_EQ(sentWire[3], static_cast<uint8_t>(CommsCrypto::SYNC_BYTE_3));

    // 2. Check message type and sequence ID
    EXPECT_EQ(sentWire[4], CommsCrypto::MSG_TYPE_TLM);
    EXPECT_EQ(sentWire[5], 0x00);
    EXPECT_EQ(sentWire[6], 0x00);

    // 3. Verify wire CRC-16 integrity over frame body
    FwSizeType wireSize = this->getLastSentSize();
    U8 encLen = sentWire[7];
    size_t crcCoverageLen = 4 + AES_BLOCKLEN + encLen;
    uint16_t computedCrc = TinyAes_ComputeCrc16(&sentWire[4], crcCoverageLen);
    uint16_t wireCrc = (static_cast<uint16_t>(sentWire[wireSize - 2]) << 8) | sentWire[wireSize - 1];
    EXPECT_EQ(computedCrc, wireCrc);
  }

  // ==========================================================================
  // 3. BOUNDARY & OFF-NOMINAL HANDLING
  // ==========================================================================

  TEST_F(Tester, CorruptedCrcRejection) {
    const uint8_t payload[] = "CRITICAL_PAYLOAD";
    const U8 payloadLen = static_cast<U8>(sizeof(payload) - 1);

    // Inject packet with intentionally inverted CRC-16
    this->injectFrame(
        CommsCrypto::MSG_TYPE_CMD,
        0x0200,
        payload,
        payloadLen,
        /*validCrc=*/false,
        /*encrypt=*/true
    );

    // FrameCrcError event must be emitted
    ASSERT_EVENTS_FrameCrcError_SIZE(1);

    // CrcErrors telemetry must increment
    ASSERT_TLM_CrcErrors_SIZE(1);
    ASSERT_TLM_CrcErrors(0, 1);

    // Telecommand must be rejected: no FrameReceived event or increment
    ASSERT_EVENTS_FrameReceived_SIZE(0);
    ASSERT_TLM_FramesUplinked_SIZE(0);
  }

  TEST_F(Tester, CorruptedPaddingDecryptionError) {
    // Construct valid CRC frame where ciphertext does not decrypt to valid PKCS#7 padding
    // We achieve this by sending unencrypted random data with encryption flag on
    const uint8_t rawGarbage[16] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67,
        0x89, 0xAB, 0xCD, 0xEF, 0xFE, 0xED, 0xBA, 0xBE
    };

    this->injectFrame(
        CommsCrypto::MSG_TYPE_CMD,
        0x0300,
        rawGarbage,
        sizeof(rawGarbage),
        /*validCrc=*/true,
        /*encrypt=*/false // Do not encrypt, so CBC decryption produces pseudo-random invalid padding
    );

    // DecryptionError event must be emitted with sequence ID
    ASSERT_EVENTS_DecryptionError_SIZE(1);
    ASSERT_EVENTS_DecryptionError(0, 0x0300);

    // DecryptionFailures telemetry must increment
    ASSERT_TLM_DecryptionFailures_SIZE(1);
    ASSERT_TLM_DecryptionFailures(0, 1);

    // Uplink must not succeed
    ASSERT_EVENTS_FrameReceived_SIZE(0);
  }

  TEST_F(Tester, NoisePreambleRejection) {
    // Feed arbitrary noise, false starts, and fragmented synchronizers
    const uint8_t noiseBytes[] = {
        0xFF, 0x00, 0x53, 0x59, 0xAA, // partial sync mismatch
        0x53, 0x59, 0x4E, 0x00,       // partial sync mismatch
        0x12, 0x34, 0x56, 0x78
    };

    this->sendRawBytes(noiseBytes, sizeof(noiseBytes));

    // Internal state machine must discard noise safely
    ASSERT_EVENTS_FrameReceived_SIZE(0);
    ASSERT_EVENTS_FrameCrcError_SIZE(0);
    ASSERT_EVENTS_DecryptionError_SIZE(0);
  }

  TEST_F(Tester, DownlinkPayloadSizeBounds) {
    uint8_t largeBuf[200];
    memset(largeBuf, 0xAA, sizeof(largeBuf));

    // Null pointer payload rejection
    EXPECT_FALSE(this->triggerDownlink(CommsCrypto::MSG_TYPE_TLM, nullptr, 10));

    // Zero-length payload rejection
    EXPECT_FALSE(this->triggerDownlink(CommsCrypto::MSG_TYPE_TLM, largeBuf, 0));

    // Exceeds max payload threshold (> 180 bytes)
    EXPECT_FALSE(this->triggerDownlink(CommsCrypto::MSG_TYPE_TLM, largeBuf, 181));

    // No frames transmitted
    EXPECT_EQ(this->getSendCallCount(), 0U);
    ASSERT_EVENTS_FrameTransmitted_SIZE(0);
  }

  // ==========================================================================
  // 4. GROUND COMMANDS & CRYPTOGRAPHIC RECONFIGURATION
  // ==========================================================================

  TEST_F(Tester, CommandSetKey) {
    // Reconfigure AES-128 key via 4x 32-bit words
    // 0x00010203 0x04050607 0x08090A0B 0x0C0D0E0F
    this->sendCmdSetKey(
        0x00010203,
        0x04050607,
        0x08090A0B,
        0x0C0D0E0F,
        Fw::CmdResponse::OK
    );

    // Verify CryptoKeyUpdated event emitted
    ASSERT_EVENTS_CryptoKeyUpdated_SIZE(1);
  }

  TEST_F(Tester, CommandEnableEncryptionToggle) {
    // 1. Disable encryption (bypass mode for bench test)
    this->sendCmdEnableEncryption(Fw::Enabled::DISABLED, Fw::CmdResponse::OK);
    ASSERT_TLM_EncryptionEnabled_SIZE(1);
    ASSERT_TLM_EncryptionEnabled(0, false);

    this->clearHistory();

    // 2. Re-enable encryption
    this->sendCmdEnableEncryption(Fw::Enabled::ENABLED, Fw::CmdResponse::OK);
    ASSERT_TLM_EncryptionEnabled_SIZE(1);
    ASSERT_TLM_EncryptionEnabled(0, true);
  }

  TEST_F(Tester, CommandSendPing) {
    // Dispatch ping command
    this->sendCmdSendPing(Fw::CmdResponse::OK);

    // Verify downlink transmission was triggered
    EXPECT_EQ(this->getSendCallCount(), 1U);
    ASSERT_EVENTS_FrameTransmitted_SIZE(1);
    ASSERT_EVENTS_FrameTransmitted(0, 0, 4); // "PING" payload is 4 bytes
    ASSERT_TLM_FramesDownlinked_SIZE(1);
    ASSERT_TLM_FramesDownlinked(0, 1);
  }

  // ==========================================================================
  // 5. STREAM FRAGMENTATION REASSEMBLY
  // ==========================================================================

  TEST_F(Tester, StreamFragmentedDelivery) {
    const uint8_t payload[] = "STREAM_TEST";
    const U8 payloadLen = static_cast<U8>(sizeof(payload) - 1);
    const U16 seqId = 0x0456;

    // Synthesize full valid wire frame into buffer
    uint8_t frame[CommsCryptoTester::MAX_FRAME_LEN];
    memset(frame, 0, sizeof(frame));

    frame[0] = CommsCrypto::SYNC_BYTE_0;
    frame[1] = CommsCrypto::SYNC_BYTE_1;
    frame[2] = CommsCrypto::SYNC_BYTE_2;
    frame[3] = CommsCrypto::SYNC_BYTE_3;
    frame[4] = CommsCrypto::MSG_TYPE_CMD;
    frame[5] = static_cast<uint8_t>((seqId >> 8) & 0xFF);
    frame[6] = static_cast<uint8_t>(seqId & 0xFF);

    uint8_t workBuf[208];
    memcpy(workBuf, payload, payloadLen);
    size_t encLen = TinyAes_ApplyPkcs7Padding(workBuf, payloadLen, sizeof(workBuf));
    frame[7] = static_cast<uint8_t>(encLen);

    uint8_t iv[AES_BLOCKLEN];
    for (uint8_t i = 0; i < AES_BLOCKLEN; ++i) iv[i] = i + 1;
    memcpy(&frame[8], iv, AES_BLOCKLEN);

    TinyAesContext ctx;
    TinyAes_Init(&ctx, CommsCryptoTester::DEFAULT_KEY, iv);
    TinyAes_EncryptCbc(&ctx, workBuf, encLen);
    memcpy(&frame[24], workBuf, encLen);

    size_t crcCoverageLen = 4 + AES_BLOCKLEN + encLen;
    uint16_t crc = TinyAes_ComputeCrc16(&frame[4], crcCoverageLen);
    size_t crcPos = 24 + encLen;
    frame[crcPos]     = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame[crcPos + 1] = static_cast<uint8_t>(crc & 0xFF);
    size_t totalLen = crcPos + 2;

    // Deliver frame byte-by-byte to simulate slow asynchronous UART FIFO feeding
    for (size_t i = 0; i < totalLen; ++i) {
      this->sendRawBytes(&frame[i], 1);
    }

    // Packet must be successfully reassembled upon receiving the final CRC byte
    ASSERT_EVENTS_FrameReceived_SIZE(1);
    ASSERT_EVENTS_FrameReceived(0, seqId, payloadLen);
    ASSERT_TLM_FramesUplinked_SIZE(1);
    ASSERT_TLM_FramesUplinked(0, 1);
  }

}

// ============================================================================
// Main entry point for GoogleTest runner
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
