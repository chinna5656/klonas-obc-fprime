/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * ParachuteDeployer Component Unit Test Harness Implementation
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>

namespace Obc {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  ParachuteDeployerTester::ParachuteDeployerTester() :
    ParachuteDeployerGTestBase("Tester", ParachuteDeployerTester::MAX_HISTORY_SIZE),
    component("ParachuteDeployer"),
    m_lastGpioState(Fw::Logic::LOW),
    m_gpioHighCount(0),
    m_gpioLowCount(0)
  {
    this->initComponents();
    this->connectPorts();
  }

  ParachuteDeployerTester::~ParachuteDeployerTester() {
    this->component.deinit();
  }

  // ----------------------------------------------------------------------
  // Test helper routines
  // ----------------------------------------------------------------------

  void ParachuteDeployerTester::sendDeployTrigger(U32 confidence, bool force) {
    this->invoke_to_deployIn(0, confidence, force);
    this->component.doDispatch();
  }

  void ParachuteDeployerTester::sendSchedTick(U32 context) {
    this->invoke_to_schedIn(0, context);
  }

  void ParachuteDeployerTester::sendCmdArm(U32 key, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_PARACHUTE_ARM(TEST_INSTANCE_ID, 10, key);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, ParachuteDeployerComponentBase::OPCODE_PARACHUTE_ARM, 10, expectedResponse);
  }

  void ParachuteDeployerTester::sendCmdDisarm(Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_PARACHUTE_DISARM(TEST_INSTANCE_ID, 20);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, ParachuteDeployerComponentBase::OPCODE_PARACHUTE_DISARM, 20, expectedResponse);
  }

  void ParachuteDeployerTester::sendCmdDeploy(U32 key, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_PARACHUTE_DEPLOY(TEST_INSTANCE_ID, 30, key);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, ParachuteDeployerComponentBase::OPCODE_PARACHUTE_DEPLOY, 30, expectedResponse);
  }

  void ParachuteDeployerTester::sendCmdSetBurnTime(U32 burnMs, Fw::CmdResponse expectedResponse) {
    this->cmdResponseHistory->clear();
    this->sendCmd_PARACHUTE_SET_BURN_TIME(TEST_INSTANCE_ID, 40, burnMs);
    this->component.doDispatch();
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(0, ParachuteDeployerComponentBase::OPCODE_PARACHUTE_SET_BURN_TIME, 40, expectedResponse);
  }

  // ----------------------------------------------------------------------
  // Handlers for outbound ports
  // ----------------------------------------------------------------------

  Drv::GpioStatus ParachuteDeployerTester::from_burnWireGpioOut_handler(
      FwIndexType portNum,
      const Fw::Logic& state
  ) {
    (void)portNum;
    m_lastGpioState = state;
    if (state == Fw::Logic::HIGH) {
      m_gpioHighCount++;
    } else {
      m_gpioLowCount++;
    }
    return Drv::GpioStatus::OP_OK;
  }

}
