/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * ParachuteDeployer Component Unit Tests (GoogleTest Suite)
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>

namespace Obc {

  // ==========================================================================
  // 1. ARMING & SECURITY INTERLOCK TESTS
  // ==========================================================================

  TEST_F(Tester, ArmWithValidKey) {
    ASSERT_EVENTS_DeployerArmed_SIZE(0);
    ASSERT_TLM_ArmState_SIZE(0);

    // Send valid 32-bit security key (0xDEADBEEF)
    this->sendCmdArm(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);

    // Verification
    ASSERT_EVENTS_DeployerArmed_SIZE(1);
    ASSERT_EVENTS_DeployerArmed(0, ParachuteDeployer::DEFAULT_ARM_TIMEOUT_SEC);

    ASSERT_TLM_ArmState_SIZE(1);
    ASSERT_TLM_ArmState(0, true);

    ASSERT_TLM_DeployerState_SIZE(1);
    ASSERT_TLM_DeployerState(0, ParachuteDeployer::STATE_ARMED);
  }

  TEST_F(Tester, ArmWithInvalidKeyRejected) {
    ASSERT_EVENTS_UnauthorizedDeployAttempt_SIZE(0);

    // Send incorrect passcode (0xBAD00000)
    this->sendCmdArm(0xBAD00000, Fw::CmdResponse::VALIDATION_ERROR);

    // Verify rejection event
    ASSERT_EVENTS_UnauthorizedDeployAttempt_SIZE(1);
    ASSERT_EVENTS_UnauthorizedDeployAttempt(0, 0xBAD00000);

    // Deployer must remain disarmed
    this->sendSchedTick();
    ASSERT_TLM_ArmState_SIZE(1);
    ASSERT_TLM_ArmState(0, false);
    ASSERT_TLM_DeployerState(0, ParachuteDeployer::STATE_DISARMED);
  }

  TEST_F(Tester, DisarmCommand) {
    // 1. Arm deployer first
    this->sendCmdArm(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);
    ASSERT_EVENTS_DeployerArmed_SIZE(1);

    this->clearHistory();

    // 2. Dispatch disarm
    this->sendCmdDisarm(Fw::CmdResponse::OK);

    // Verification
    ASSERT_EVENTS_DeployerDisarmed_SIZE(1);
    ASSERT_TLM_ArmState_SIZE(1);
    ASSERT_TLM_ArmState(0, false);
    ASSERT_TLM_DeployerState(0, ParachuteDeployer::STATE_DISARMED);
  }

  // ==========================================================================
  // 2. DIRECT TELECOMMAND ACTUATION & PASSCODE PROTECTION
  // ==========================================================================

  TEST_F(Tester, DirectDeployWithValidKey) {
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);
    ASSERT_EVENTS_BurnWireActivated_SIZE(0);

    // Send direct deploy with valid key
    this->sendCmdDeploy(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);

    // Verify MOSFET energized HIGH
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);
    ASSERT_EVENTS_BurnWireActivated_SIZE(1);
    ASSERT_EVENTS_BurnWireActivated(0, ParachuteDeployer::DEFAULT_BURN_MS);

    ASSERT_TLM_DeployerState_SIZE(1);
    ASSERT_TLM_DeployerState(0, ParachuteDeployer::STATE_BURNING);
  }

  TEST_F(Tester, DirectDeployWithInvalidKeyRejected) {
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);

    // Send direct deploy with invalid key
    this->sendCmdDeploy(0x11223344, Fw::CmdResponse::VALIDATION_ERROR);

    // Gate must remain un-energized (LOW)
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);
    ASSERT_EVENTS_UnauthorizedDeployAttempt_SIZE(1);
    ASSERT_EVENTS_BurnWireActivated_SIZE(0);
  }

  // ==========================================================================
  // 3. HARDWARE SAFETY TIMERS & AUTO-CUTOFF
  // ==========================================================================

  TEST_F(Tester, BurnPulseAutoCutoff) {
    // Initiate burn (default 3000 ms)
    this->sendCmdDeploy(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);

    // Tick 29 times (29 * 100ms = 2900ms) -> Gate must still be HIGH
    for (U32 i = 0; i < 29; ++i) {
      this->sendSchedTick();
      EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);
    }

    // 30th tick (3000ms elapsed) -> Burn must terminate automatically
    this->sendSchedTick();
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);

    // Verification of cutoff events
    ASSERT_EVENTS_BurnWireDeactivated_SIZE(1);
    ASSERT_EVENTS_DeploymentCompleted_SIZE(1);
  }

  TEST_F(Tester, ArmingWindowTimeoutAutoDisarm) {
    // Arm deployer (default 60s window = 600 ticks @ 100ms)
    this->sendCmdArm(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);
    ASSERT_EVENTS_DeployerArmed_SIZE(1);

    // Tick 599 times -> Must remain armed
    for (U32 i = 0; i < 599; ++i) {
      this->sendSchedTick();
    }
    ASSERT_EVENTS_ArmWindowExpired_SIZE(0);

    // 600th tick -> Window expires, auto-disarms
    this->sendSchedTick();
    ASSERT_EVENTS_ArmWindowExpired_SIZE(1);
    ASSERT_EVENTS_DeployerDisarmed_SIZE(1);
  }

  // ==========================================================================
  // 4. AUTONOMOUS TRIGGER LOGIC (deployIn PORT)
  // ==========================================================================

  TEST_F(Tester, DeployTriggerWhenArmed) {
    // 1. Arm deployer
    this->sendCmdArm(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);

    // 2. Incoming autonomous trigger from NavPredictor
    this->sendDeployTrigger(95, false);

    // Verify burn sequence started
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);
    ASSERT_EVENTS_BurnWireActivated_SIZE(1);
  }

  TEST_F(Tester, DeployTriggerWhenDisarmedIgnored) {
    // Deployer is disarmed by default
    this->sendDeployTrigger(95, false);

    // Verify trigger ignored, gate remains LOW
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);
    ASSERT_EVENTS_BurnWireActivated_SIZE(0);
  }

  TEST_F(Tester, ForceDeployTriggerWhenDisarmed) {
    // Deployer is disarmed, but emergency trigger arrives with force=true
    this->sendDeployTrigger(100, true);

    // Verify burn sequence forced on
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);
    ASSERT_EVENTS_BurnWireActivated_SIZE(1);
  }

  // ==========================================================================
  // 5. CONFIGURATION COMMANDS
  // ==========================================================================

  TEST_F(Tester, SetBurnDurationClampedToMax) {
    // Request 8000 ms burn (exceeds MAX_PERMISSIBLE_BURN_MS = 5000 ms)
    this->sendCmdSetBurnTime(8000, Fw::CmdResponse::OK);

    // Start burn and verify clamped to 5000 ms
    this->sendCmdDeploy(ParachuteDeployer::SAFETY_ARM_KEY, Fw::CmdResponse::OK);
    ASSERT_EVENTS_BurnWireActivated(0, ParachuteDeployer::MAX_PERMISSIBLE_BURN_MS);

    // Tick 49 times -> Gate HIGH
    for (U32 i = 0; i < 49; ++i) {
      this->sendSchedTick();
      EXPECT_EQ(this->getLastGpioState(), Fw::Logic::HIGH);
    }

    // 50th tick -> Terminated at 5000 ms
    this->sendSchedTick();
    EXPECT_EQ(this->getLastGpioState(), Fw::Logic::LOW);
    ASSERT_EVENTS_DeploymentCompleted_SIZE(1);
  }

}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
