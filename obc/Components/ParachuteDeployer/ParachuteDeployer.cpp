/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * ParachuteDeployer Component Implementation
 * ============================================================================
 */

#include "obc/Components/ParachuteDeployer/ParachuteDeployer.hpp"
#include <Fw/Types/Assert.hpp>

namespace Obc {

  ParachuteDeployer::ParachuteDeployer(const char* const compName) :
    ParachuteDeployerComponentBase(compName),
    m_state(STATE_DISARMED),
    m_burnDurationMs(DEFAULT_BURN_MS),
    m_burnRemainingMs(0),
    m_armTimeoutSec(0),
    m_deploymentCount(0)
  {
  }

  void ParachuteDeployer::init(FwEnumStoreType instance) {
    ParachuteDeployerComponentBase::init(instance);
  }

  void ParachuteDeployer::deployIn_handler(
      FwIndexType portNum,
      U32 confidence,
      bool force
  ) {
    FW_ASSERT(portNum == 0);
    (void)confidence;

    if (m_state == STATE_ARMED || force) {
      this->startBurn();
    }
  }

  void ParachuteDeployer::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    // Service active burn pulse (e.g., 100ms tick resolution)
    if (m_state == STATE_BURNING) {
      if (m_burnRemainingMs <= 100) {
        this->stopBurn();
      } else {
        m_burnRemainingMs -= 100;
      }
    }

    // Service arming window countdown
    if (m_state == STATE_ARMED) {
      if (m_armTimeoutSec > 0) {
        static U8 subTicks = 0;
        if (++subTicks >= 10) {
          subTicks = 0;
          m_armTimeoutSec--;
          if (m_armTimeoutSec == 0) {
            m_state = STATE_DISARMED;
            this->log_WARNING_HI_ArmWindowExpired();
            this->log_ACTIVITY_HI_DeployerDisarmed();
          }
        }
      }
    }

    // Update telemetry
    this->tlmWrite_ArmState(m_state == STATE_ARMED);
    this->tlmWrite_DeployerState(static_cast<U8>(m_state));
    this->tlmWrite_BurnTimeRemainingMs(m_burnRemainingMs);
    this->tlmWrite_ArmTimeoutRemainingSec(m_armTimeoutSec);
    this->tlmWrite_DeploymentCount(m_deploymentCount);
  }

  void ParachuteDeployer::PARACHUTE_ARM_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      U32 key
  ) {
    if (key != SAFETY_ARM_KEY) {
      this->log_WARNING_HI_UnauthorizedDeployAttempt(key);
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
      return;
    }

    m_state = STATE_ARMED;
    m_armTimeoutSec = DEFAULT_ARM_TIMEOUT_SEC;
    this->log_ACTIVITY_HI_DeployerArmed(m_armTimeoutSec);

    // Explicitly update telemetry channels immediately on command execution
    this->tlmWrite_ArmState(true);
    this->tlmWrite_DeployerState(static_cast<U8>(m_state));
    this->tlmWrite_ArmTimeoutRemainingSec(m_armTimeoutSec);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void ParachuteDeployer::PARACHUTE_DISARM_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    if (m_state == STATE_BURNING) {
      this->stopBurn();
    }
    m_state = STATE_DISARMED;
    m_armTimeoutSec = 0;
    this->log_ACTIVITY_HI_DeployerDisarmed();

    // Explicitly update telemetry channels immediately on command execution
    this->tlmWrite_ArmState(false);
    this->tlmWrite_DeployerState(static_cast<U8>(m_state));
    this->tlmWrite_ArmTimeoutRemainingSec(0);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void ParachuteDeployer::PARACHUTE_DEPLOY_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      U32 key
  ) {
    if (key != SAFETY_ARM_KEY) {
      this->log_WARNING_HI_UnauthorizedDeployAttempt(key);
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
      return;
    }

    this->startBurn();

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void ParachuteDeployer::PARACHUTE_SET_BURN_TIME_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      U32 burnDurationMs
  ) {
    if (burnDurationMs > MAX_PERMISSIBLE_BURN_MS) {
      burnDurationMs = MAX_PERMISSIBLE_BURN_MS;
    }
    m_burnDurationMs = burnDurationMs;

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void ParachuteDeployer::startBurn() {
    m_state = STATE_BURNING;
    m_burnRemainingMs = m_burnDurationMs;
    m_deploymentCount++;

    // Assert MOSFET gate HIGH to flow current through nichrome burn-wire
    if (this->isConnected_burnWireGpioOut_OutputPort(0)) {
      this->burnWireGpioOut_out(0, Fw::Logic::HIGH);
    }

    this->log_WARNING_HI_BurnWireActivated(m_burnDurationMs);

    this->tlmWrite_ArmState(false);
    this->tlmWrite_DeployerState(static_cast<U8>(m_state));
    this->tlmWrite_BurnTimeRemainingMs(m_burnRemainingMs);
    this->tlmWrite_DeploymentCount(m_deploymentCount);
  }

  void ParachuteDeployer::stopBurn() {
    // Assert MOSFET gate LOW to turn off nichrome heating
    if (this->isConnected_burnWireGpioOut_OutputPort(0)) {
      this->burnWireGpioOut_out(0, Fw::Logic::LOW);
    }

    m_burnRemainingMs = 0;
    m_state = STATE_DEPLOYED;

    this->log_ACTIVITY_HI_BurnWireDeactivated();
    this->log_ACTIVITY_HI_DeploymentCompleted();

    this->tlmWrite_ArmState(false);
    this->tlmWrite_DeployerState(static_cast<U8>(m_state));
    this->tlmWrite_BurnTimeRemainingMs(0);
  }

}
