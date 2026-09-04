/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * NavPredictor Component Unit Test Harness Implementation
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>
#include <cstring>

namespace Obc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  NavPredictorTester::NavPredictorTester() :
    NavPredictorGTestBase("Tester", NavPredictorTester::MAX_HISTORY_SIZE),
    component("NavPredictor"),
    m_crashPinState(Fw::Logic::LOW),
    m_fprimeBuffer(m_nmeaBuf, sizeof(m_nmeaBuf))
  {
    memset(m_nmeaBuf, 0, sizeof(m_nmeaBuf));
    this->initComponents();
    this->connectPorts();
  }

  NavPredictorTester::~NavPredictorTester() {
    this->component.deinit();
  }

  // ----------------------------------------------------------------------
  // Test helper routines
  // ----------------------------------------------------------------------

  void NavPredictorTester::sendNmeaSentence(const char* sentence) {
    ASSERT_NE(sentence, nullptr);
    FwSizeType len = static_cast<FwSizeType>(strlen(sentence));
    ASSERT_LE(len, sizeof(m_nmeaBuf));

    memcpy(m_nmeaBuf, sentence, len);
    m_fprimeBuffer.setSize(len);

    // Invoke sync input port
    this->invoke_to_gpsDataIn(0, m_fprimeBuffer, Drv::ByteStreamStatus::OP_OK);
  }

  void NavPredictorTester::sendSchedTick(U32 context) {
    this->invoke_to_schedIn(0, context);
  }

  void NavPredictorTester::setCrashPinState(Fw::Logic state) {
    m_crashPinState = state;
  }

  void NavPredictorTester::sendCmdSetDeployAlt(F32 deployAltM, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_NAV_SET_DEPLOY_ALT(TEST_INSTANCE_ID, 10, deployAltM);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, NavPredictorComponentBase::OPCODE_NAV_SET_DEPLOY_ALT, 10, expectedResponse);
  }

  void NavPredictorTester::sendCmdArmParachute(Fw::Enabled arm, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_NAV_ARM_PARACHUTE(TEST_INSTANCE_ID, 20, arm);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, NavPredictorComponentBase::OPCODE_NAV_ARM_PARACHUTE, 20, expectedResponse);
  }

  void NavPredictorTester::sendCmdForceDeploy(Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_NAV_FORCE_DEPLOY(TEST_INSTANCE_ID, 30);
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, NavPredictorComponentBase::OPCODE_NAV_FORCE_DEPLOY, 30, expectedResponse);
  }

  // ----------------------------------------------------------------------
  // Handlers for outbound ports
  // ----------------------------------------------------------------------

  void NavPredictorTester::from_deployOut_handler(
      FwIndexType portNum,
      U32 confidence,
      bool force
  ) {
    (void)portNum;
    this->pushFromPortEntry_deployOut(confidence, force);
  }

  Drv::GpioStatus NavPredictorTester::from_crashGpioIn_handler(
      FwIndexType portNum,
      Fw::Logic& state
  ) {
    (void)portNum;
    state = this->m_crashPinState;
    return Drv::GpioStatus::OP_OK;
  }

}
