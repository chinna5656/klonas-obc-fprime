/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * EnvSensors Component Unit Test Harness Implementation
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>
#include <cstring>

namespace Obc {

  EnvSensorsTester::EnvSensorsTester() :
    EnvSensorsGTestBase("Tester", EnvSensorsTester::MAX_HISTORY_SIZE),
    component("EnvSensors"),
    m_mockSpiStatus(Drv::SpiStatus::SPI_OK),
    m_lastCsBno(Fw::Logic::HIGH),
    m_lastCsBmp(Fw::Logic::HIGH),
    m_lastCsBme(Fw::Logic::HIGH),
    m_csBnoSelectCount(0),
    m_csBmpSelectCount(0),
    m_csBmeSelectCount(0),
    m_spiTransferCount(0)
  {
    memset(m_mockBmpRaw, 0, sizeof(m_mockBmpRaw));
    memset(m_mockBmeRaw, 0, sizeof(m_mockBmeRaw));
    memset(m_mockBnoRaw, 0, sizeof(m_mockBnoRaw));

    this->initComponents();
    this->connectPorts();
  }

  EnvSensorsTester::~EnvSensorsTester() {
    this->component.deinit();
  }

  void EnvSensorsTester::sendSchedTick(U32 context) {
    this->invoke_to_schedIn(0, context);
  }

  void EnvSensorsTester::sendCmdInitSensors(Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_ENV_INIT_SENSORS(TEST_INSTANCE_ID, 10);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, EnvSensorsComponentBase::OPCODE_ENV_INIT_SENSORS, 10, expectedResponse);
  }

  void EnvSensorsTester::sendCmdSetSeaLevelPressure(F32 hpa, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_ENV_SET_SEA_LEVEL_PRESSURE(TEST_INSTANCE_ID, 20, hpa);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, EnvSensorsComponentBase::OPCODE_ENV_SET_SEA_LEVEL_PRESSURE, 20, expectedResponse);
  }

  void EnvSensorsTester::setMockBmpData(uint8_t pressMsb, uint8_t pressLsb, uint8_t pressXlsb,
                                        uint8_t tempMsb, uint8_t tempLsb, uint8_t tempXlsb) {
    m_mockBmpRaw[0] = 0x00;
    m_mockBmpRaw[1] = pressMsb;
    m_mockBmpRaw[2] = pressLsb;
    m_mockBmpRaw[3] = pressXlsb;
    m_mockBmpRaw[4] = tempMsb;
    m_mockBmpRaw[5] = tempLsb;
    m_mockBmpRaw[6] = tempXlsb;
  }

  void EnvSensorsTester::setMockBmeData(uint8_t pressMsb, uint8_t pressLsb, uint8_t pressXlsb,
                                        uint8_t tempMsb, uint8_t tempLsb, uint8_t tempXlsb,
                                        uint8_t humMsb, uint8_t humLsb) {
    m_mockBmeRaw[0] = 0x00;
    m_mockBmeRaw[1] = pressMsb;
    m_mockBmeRaw[2] = pressLsb;
    m_mockBmeRaw[3] = pressXlsb;
    m_mockBmeRaw[4] = tempMsb;
    m_mockBmeRaw[5] = tempLsb;
    m_mockBmeRaw[6] = tempXlsb;
    m_mockBmeRaw[7] = humMsb;
    m_mockBmeRaw[8] = humLsb;
  }

  void EnvSensorsTester::setMockBnoQuaternion(int16_t qx, int16_t qy, int16_t qz, int16_t qw) {
    memset(m_mockBnoRaw, 0, sizeof(m_mockBnoRaw));
    m_mockBnoRaw[8] = static_cast<uint8_t>(qx & 0xFF);
    m_mockBnoRaw[9] = static_cast<uint8_t>((qx >> 8) & 0xFF);
    m_mockBnoRaw[10] = static_cast<uint8_t>(qy & 0xFF);
    m_mockBnoRaw[11] = static_cast<uint8_t>((qy >> 8) & 0xFF);
    m_mockBnoRaw[12] = static_cast<uint8_t>(qz & 0xFF);
    m_mockBnoRaw[13] = static_cast<uint8_t>((qz >> 8) & 0xFF);
    m_mockBnoRaw[14] = static_cast<uint8_t>(qw & 0xFF);
    m_mockBnoRaw[15] = static_cast<uint8_t>((qw >> 8) & 0xFF);
  }

  Drv::SpiStatus EnvSensorsTester::from_spiOut_handler(
      FwIndexType portNum,
      Fw::Buffer& writeBuffer,
      Fw::Buffer& readBuffer
  ) {
    (void)portNum;
    m_spiTransferCount++;

    if (m_mockSpiStatus != Drv::SpiStatus::SPI_OK) {
      return m_mockSpiStatus;
    }

    const uint8_t* tx = writeBuffer.getData();
    uint8_t* rx = readBuffer.getData();
    FwSizeType size = readBuffer.getSize();

    if (rx == nullptr || tx == nullptr) {
      return Drv::SpiStatus::SPI_OK;
    }

    memset(rx, 0, size);

    // If Chip Select for BMP280 is active (LOW)
    if (m_lastCsBmp == Fw::Logic::LOW) {
      if (size >= 2 && tx[0] == 0xD0) {
        rx[0] = 0x58;
        rx[1] = 0x58;
      } else if (size >= 7 && tx[0] == 0xF7) {
        memcpy(rx, m_mockBmpRaw, (size < sizeof(m_mockBmpRaw)) ? size : sizeof(m_mockBmpRaw));
      }
    }
    // If Chip Select for BME680 is active (LOW)
    else if (m_lastCsBme == Fw::Logic::LOW) {
      if (size >= 2 && tx[0] == 0xD0) {
        rx[0] = 0x61;
        rx[1] = 0x61;
      } else if (size >= 9 && tx[0] == 0x1F) {
        memcpy(rx, m_mockBmeRaw, (size < sizeof(m_mockBmeRaw)) ? size : sizeof(m_mockBmeRaw));
      }
    }
    // If Chip Select for BNO08X is active (LOW)
    else if (m_lastCsBno == Fw::Logic::LOW) {
      if (size >= 16) {
        memcpy(rx, m_mockBnoRaw, (size < sizeof(m_mockBnoRaw)) ? size : sizeof(m_mockBnoRaw));
      }
    }

    return Drv::SpiStatus::SPI_OK;
  }

  Drv::GpioStatus EnvSensorsTester::from_csBnoOut_handler(
      FwIndexType portNum,
      const Fw::Logic& state
  ) {
    (void)portNum;
    m_lastCsBno = state;
    if (state == Fw::Logic::LOW) {
      m_csBnoSelectCount++;
    }
    return Drv::GpioStatus::OP_OK;
  }

  Drv::GpioStatus EnvSensorsTester::from_csBmpOut_handler(
      FwIndexType portNum,
      const Fw::Logic& state
  ) {
    (void)portNum;
    m_lastCsBmp = state;
    if (state == Fw::Logic::LOW) {
      m_csBmpSelectCount++;
    }
    return Drv::GpioStatus::OP_OK;
  }

  Drv::GpioStatus EnvSensorsTester::from_csBmeOut_handler(
      FwIndexType portNum,
      const Fw::Logic& state
  ) {
    (void)portNum;
    m_lastCsBme = state;
    if (state == Fw::Logic::LOW) {
      m_csBmeSelectCount++;
    }
    return Drv::GpioStatus::OP_OK;
  }

}
