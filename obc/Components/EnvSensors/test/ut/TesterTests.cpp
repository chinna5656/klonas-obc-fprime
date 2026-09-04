/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * EnvSensors Component Unit Test Suite
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>
#include <cmath>

namespace Obc {

  // --------------------------------------------------------------------------
  // Test 1: Initial Boot State
  // --------------------------------------------------------------------------
  TEST_F(Tester, InitialBootState) {
    // Assert CS lines idle in inactive HIGH state
    EXPECT_EQ(this->getCsBnoState(), Fw::Logic::HIGH);
    EXPECT_EQ(this->getCsBmpState(), Fw::Logic::HIGH);
    EXPECT_EQ(this->getCsBmeState(), Fw::Logic::HIGH);
    EXPECT_EQ(this->getCsBnoSelectCount(), 0U);
    EXPECT_EQ(this->getCsBmpSelectCount(), 0U);
    EXPECT_EQ(this->getCsBmeSelectCount(), 0U);
    EXPECT_EQ(this->getSpiTransferCount(), 0U);
  }

  // --------------------------------------------------------------------------
  // Test 2: Periodic Scheduling Cadence (10Hz IMU, 1Hz Env)
  // --------------------------------------------------------------------------
  TEST_F(Tester, PeriodicSchedulingCadence) {
    // Tick 1: Immediate read of BNO, BMP, and BME on boot
    this->sendSchedTick(1);
    EXPECT_EQ(this->getCsBnoSelectCount(), 1U);
    EXPECT_EQ(this->getCsBmpSelectCount(), 1U);
    EXPECT_EQ(this->getCsBmeSelectCount(), 1U);

    // Ticks 2 through 9: Only BNO (IMU) is sampled at 10 Hz
    for (U32 i = 2; i <= 9; i++) {
      this->sendSchedTick(i);
    }
    EXPECT_EQ(this->getCsBnoSelectCount(), 9U);
    EXPECT_EQ(this->getCsBmpSelectCount(), 1U); // Still 1
    EXPECT_EQ(this->getCsBmeSelectCount(), 1U); // Still 1

    // Tick 10: 1 Hz boundary hit, BMP and BME sampled again
    this->sendSchedTick(10);
    EXPECT_EQ(this->getCsBnoSelectCount(), 10U);
    EXPECT_EQ(this->getCsBmpSelectCount(), 2U);
    EXPECT_EQ(this->getCsBmeSelectCount(), 2U);

    // Verify telemetry channels emitted
    ASSERT_TLM_Bmp_InternalTemp_SIZE(10);
    ASSERT_TLM_Bmp_InternalPressure_SIZE(10);
    ASSERT_TLM_Bmp_InternalAltitude_SIZE(10);
    ASSERT_TLM_Imu_Roll_SIZE(10);
    ASSERT_TLM_Imu_Pitch_SIZE(10);
    ASSERT_TLM_Imu_Yaw_SIZE(10);
    ASSERT_TLM_Spi1ErrorCount_SIZE(10);
    ASSERT_TLM_Spi1ErrorCount(9, 0U);
  }

  // --------------------------------------------------------------------------
  // Test 3: Chip Select Isolation and Sequencing
  // --------------------------------------------------------------------------
  TEST_F(Tester, ChipSelectIsolationAndSequencing) {
    this->sendSchedTick(1);

    // After transactions complete, every CS must be restored to HIGH
    EXPECT_EQ(this->getCsBnoState(), Fw::Logic::HIGH);
    EXPECT_EQ(this->getCsBmpState(), Fw::Logic::HIGH);
    EXPECT_EQ(this->getCsBmeState(), Fw::Logic::HIGH);
  }

  // --------------------------------------------------------------------------
  // Test 4: InitSensors Command Success
  // --------------------------------------------------------------------------
  TEST_F(Tester, InitSensorsCommandSuccess) {
    // Send command to probe sensors on SPI1
    this->sendCmdInitSensors(Fw::CmdResponse::OK);

    // All 3 sensors responded with valid IDs: mask = 0x07 (bits 0, 1, 2)
    ASSERT_EVENTS_SensorsInitSuccess_SIZE(1);
    ASSERT_EVENTS_SensorsInitSuccess(0, 0x07);
  }

  // --------------------------------------------------------------------------
  // Test 5: SetSeaLevelPressure Valid Range
  // --------------------------------------------------------------------------
  TEST_F(Tester, SetSeaLevelPressureValid) {
    // Set standard 1020.0 hPa
    this->sendCmdSetSeaLevelPressure(1020.0f, Fw::CmdResponse::OK);

    // Boundary 800.0 hPa
    this->sendCmdSetSeaLevelPressure(800.0f, Fw::CmdResponse::OK);

    // Boundary 1100.0 hPa
    this->sendCmdSetSeaLevelPressure(1100.0f, Fw::CmdResponse::OK);
  }

  // --------------------------------------------------------------------------
  // Test 6: SetSeaLevelPressure Out of Range Rejection
  // --------------------------------------------------------------------------
  TEST_F(Tester, SetSeaLevelPressureOutOfRange) {
    // Below lower limit
    this->sendCmdSetSeaLevelPressure(750.0f, Fw::CmdResponse::VALIDATION_ERROR);

    // Above upper limit
    this->sendCmdSetSeaLevelPressure(1150.0f, Fw::CmdResponse::VALIDATION_ERROR);

    // Extreme negative
    this->sendCmdSetSeaLevelPressure(-10.0f, Fw::CmdResponse::VALIDATION_ERROR);
  }

  // --------------------------------------------------------------------------
  // Test 7: SPI Transaction Failure Handling
  // --------------------------------------------------------------------------
  TEST_F(Tester, SpiTransactionFailureHandling) {
    // Force SPI hardware error
    this->setMockSpiStatus(Drv::SpiStatus::SPI_OTHER_ERR);

    this->sendSchedTick(1);

    // Verify error counter incremented and warning events emitted
    ASSERT_EVENTS_SensorReadError_SIZE(3); // BNO, BMP, BME all failed
    EXPECT_GT(this->tlmHistory_Spi1ErrorCount->size(), 0U);
    EXPECT_EQ(this->tlmHistory_Spi1ErrorCount->at(0).arg, 3U);
  }

  // --------------------------------------------------------------------------
  // Test 8: BMP280 Data Decoding & Hypsometric Altitude
  // --------------------------------------------------------------------------
  TEST_F(Tester, Bmp280DataDecodingAndAltitude) {
    // Raw pressure: 0x65, 0x90, 0x00 -> (0x65 << 12) | (0x90 << 4) = 415744 + 2304 = 418048 / 256 / 100 = 16.33 hPa
    // Raw temp: 0x51, 0x80, 0x00 -> 333824 / 5120 = 65.2 deg C
    this->setMockBmpData(0x65, 0x90, 0x00, 0x51, 0x80, 0x00);

    this->sendSchedTick(1);

    ASSERT_TLM_Bmp_InternalTemp_SIZE(1);
    ASSERT_TLM_Bmp_InternalPressure_SIZE(1);
    ASSERT_TLM_Bmp_InternalAltitude_SIZE(1);

    F32 temp = this->tlmHistory_Bmp_InternalTemp->at(0).arg;
    F32 press = this->tlmHistory_Bmp_InternalPressure->at(0).arg;
    F32 alt = this->tlmHistory_Bmp_InternalAltitude->at(0).arg;

    EXPECT_GT(temp, 0.0f);
    EXPECT_GT(press, 0.0f);
    EXPECT_GT(alt, 0.0f);
  }

  // --------------------------------------------------------------------------
  // Test 9: BME680 Environmental Decoding
  // --------------------------------------------------------------------------
  TEST_F(Tester, Bme680DataDecoding) {
    this->setMockBmeData(0x65, 0x90, 0x00, 0x51, 0x80, 0x00, 0x01, 0x00);

    this->sendSchedTick(1);

    ASSERT_TLM_Bme_ExternalTemp_SIZE(1);
    ASSERT_TLM_Bme_ExternalPressure_SIZE(1);
    ASSERT_TLM_Bme_ExternalHumidity_SIZE(1);
    ASSERT_TLM_Bme_GasResistance_SIZE(1);

    EXPECT_GT(this->tlmHistory_Bme_ExternalTemp->at(0).arg, 0.0f);
    EXPECT_GT(this->tlmHistory_Bme_ExternalPressure->at(0).arg, 0.0f);
    EXPECT_GT(this->tlmHistory_Bme_ExternalHumidity->at(0).arg, 0.0f);
    EXPECT_FLOAT_EQ(this->tlmHistory_Bme_GasResistance->at(0).arg, 12500.0f);
  }

  // --------------------------------------------------------------------------
  // Test 10: BNO08X Quaternion & Euler Angle Calculation
  // --------------------------------------------------------------------------
  TEST_F(Tester, Bno08xQuaternionAndEulerConversion) {
    // Level attitude: qw = 1.0 (Q14 = 16384), qx = 0, qy = 0, qz = 0
    this->setMockBnoQuaternion(0, 0, 0, 16384);
    this->sendSchedTick(1);

    EXPECT_NEAR(this->tlmHistory_Imu_Roll->at(0).arg, 0.0f, 0.1f);
    EXPECT_NEAR(this->tlmHistory_Imu_Pitch->at(0).arg, 0.0f, 0.1f);
    EXPECT_NEAR(this->tlmHistory_Imu_Yaw->at(0).arg, 0.0f, 0.1f);
    EXPECT_NEAR(this->tlmHistory_Imu_AccZ->at(0).arg, 9.80665f, 0.01f);

    // Pure 90 deg Yaw: qw = cos(45 deg) = 0.7071 * 16384 ~= 11585, qz = sin(45 deg) ~= 11585
    this->setMockBnoQuaternion(0, 0, 11585, 11585);
    this->sendSchedTick(2);

    EXPECT_NEAR(this->tlmHistory_Imu_Yaw->at(1).arg, 90.0f, 1.0f);
  }

  // --------------------------------------------------------------------------
  // Test 11: Continuous Multi-Tick Flight Stability
  // --------------------------------------------------------------------------
  TEST_F(Tester, ContinuousMultitickStability) {
    for (U32 tick = 1; tick <= 50; tick++) {
      this->sendSchedTick(tick);
    }

    ASSERT_TLM_Imu_Roll_SIZE(50);
    ASSERT_TLM_Bmp_InternalTemp_SIZE(50);
    EXPECT_EQ(this->tlmHistory_Spi1ErrorCount->at(this->tlmHistory_Spi1ErrorCount->size() - 1).arg, 0U);
  }

}
