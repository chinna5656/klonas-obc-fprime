/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * ParachuteDeployer Component Unit Test Harness Header
 * ============================================================================
 */

#ifndef OBC_PARACHUTEDEPLOYER_TESTER_HPP_
#define OBC_PARACHUTEDEPLOYER_TESTER_HPP_

#include "obc/Components/ParachuteDeployer/ParachuteDeployer.hpp"
#include "ParachuteDeployerGTestBase.hpp"
#include "gtest/gtest.h"

namespace Obc {

  class ParachuteDeployerTester : public ParachuteDeployerGTestBase, public ::testing::Test {

    public:

      static constexpr U32 MAX_HISTORY_SIZE = 2000;
      static constexpr FwEnumStoreType TEST_INSTANCE_ID = 0;
      static constexpr FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 10;

      //! Construct test harness
      ParachuteDeployerTester();

      //! Destructor
      ~ParachuteDeployerTester() override;

      // ----------------------------------------------------------------------
      // Test helper routines
      // ----------------------------------------------------------------------

      //! Invoke deployIn port (trigger from NavPredictor)
      void sendDeployTrigger(U32 confidence, bool force);

      //! Advance periodic scheduler tick (10 Hz = 100 ms)
      void sendSchedTick(U32 context = 0);

      //! Dispatch PARACHUTE_ARM command
      void sendCmdArm(U32 key, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch PARACHUTE_DISARM command
      void sendCmdDisarm(Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch PARACHUTE_DEPLOY command
      void sendCmdDeploy(U32 key, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch PARACHUTE_SET_BURN_TIME command
      void sendCmdSetBurnTime(U32 burnMs, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Get current state of mock MOSFET GPIO
      Fw::Logic getLastGpioState() const { return m_lastGpioState; }
      U32 getGpioHighCount() const { return m_gpioHighCount; }
      U32 getGpioLowCount() const { return m_gpioLowCount; }

    private:

      // ----------------------------------------------------------------------
      // Handlers for outbound ports from component under test
      // ----------------------------------------------------------------------

      Drv::GpioStatus from_burnWireGpioOut_handler(
          FwIndexType portNum,
          const Fw::Logic& state
      ) override;

      // ----------------------------------------------------------------------
      // Harness initialization
      // ----------------------------------------------------------------------

      void connectPorts();
      void initComponents();

      // ----------------------------------------------------------------------
      // Component under test
      // ----------------------------------------------------------------------

      ParachuteDeployer component;

      // Mock GPIO state tracking
      Fw::Logic m_lastGpioState;
      U32 m_gpioHighCount;
      U32 m_gpioLowCount;

  };

  using Tester = ParachuteDeployerTester;

}

#endif /* OBC_PARACHUTEDEPLOYER_TESTER_HPP_ */
