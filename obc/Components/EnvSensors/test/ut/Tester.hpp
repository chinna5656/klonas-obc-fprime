/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * EnvSensors Component Unit Test Harness Header
 * ============================================================================
 */

#ifndef OBC_ENVSENSORS_TESTER_HPP_
#define OBC_ENVSENSORS_TESTER_HPP_

#include "obc/Components/EnvSensors/EnvSensors.hpp"
#include "EnvSensorsGTestBase.hpp"
#include "gtest/gtest.h"

namespace Obc {

  class EnvSensorsTester : public EnvSensorsGTestBase, public ::testing::Test {

    public:

      static constexpr U32 MAX_HISTORY_SIZE = 500;
      static constexpr FwEnumStoreType TEST_INSTANCE_ID = 0;
      static constexpr FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 10;

      EnvSensorsTester();
      ~EnvSensorsTester() override;

      // ----------------------------------------------------------------------
      // Test helper routines
      // ----------------------------------------------------------------------

      //! Advance periodic scheduler tick
      void sendSchedTick(U32 context = 0);

      //! Dispatch ENV_INIT_SENSORS command
      void sendCmdInitSensors(Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch ENV_SET_SEA_LEVEL_PRESSURE command
      void sendCmdSetSeaLevelPressure(F32 hpa, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Configure mock SPI status
      void setMockSpiStatus(Drv::SpiStatus status) { m_mockSpiStatus = status; }

      //! Configure mock raw sensor register data
      void setMockBmpData(uint8_t pressMsb, uint8_t pressLsb, uint8_t pressXlsb,
                          uint8_t tempMsb, uint8_t tempLsb, uint8_t tempXlsb);
      void setMockBmeData(uint8_t pressMsb, uint8_t pressLsb, uint8_t pressXlsb,
                          uint8_t tempMsb, uint8_t tempLsb, uint8_t tempXlsb,
                          uint8_t humMsb, uint8_t humLsb);
      void setMockBnoQuaternion(int16_t qx, int16_t qy, int16_t qz, int16_t qw);

      // State accessors
      Fw::Logic getCsBnoState() const { return m_lastCsBno; }
      Fw::Logic getCsBmpState() const { return m_lastCsBmp; }
      Fw::Logic getCsBmeState() const { return m_lastCsBme; }
      U32 getCsBnoSelectCount() const { return m_csBnoSelectCount; }
      U32 getCsBmpSelectCount() const { return m_csBmpSelectCount; }
      U32 getCsBmeSelectCount() const { return m_csBmeSelectCount; }
      U32 getSpiTransferCount() const { return m_spiTransferCount; }

    private:

      // ----------------------------------------------------------------------
      // Handlers for outbound ports
      // ----------------------------------------------------------------------

      Drv::SpiStatus from_spiOut_handler(
          FwIndexType portNum,
          Fw::Buffer& writeBuffer,
          Fw::Buffer& readBuffer
      ) override;

      Drv::GpioStatus from_csBnoOut_handler(
          FwIndexType portNum,
          const Fw::Logic& state
      ) override;

      Drv::GpioStatus from_csBmpOut_handler(
          FwIndexType portNum,
          const Fw::Logic& state
      ) override;

      Drv::GpioStatus from_csBmeOut_handler(
          FwIndexType portNum,
          const Fw::Logic& state
      ) override;

      void connectPorts();
      void initComponents();

    private:

      EnvSensors component;

      Drv::SpiStatus m_mockSpiStatus;
      Fw::Logic m_lastCsBno;
      Fw::Logic m_lastCsBmp;
      Fw::Logic m_lastCsBme;
      U32 m_csBnoSelectCount;
      U32 m_csBmpSelectCount;
      U32 m_csBmeSelectCount;
      U32 m_spiTransferCount;

      uint8_t m_mockBmpRaw[7];
      uint8_t m_mockBmeRaw[9];
      uint8_t m_mockBnoRaw[16];

  };

  typedef EnvSensorsTester Tester;

}

#endif /* OBC_ENVSENSORS_TESTER_HPP_ */
