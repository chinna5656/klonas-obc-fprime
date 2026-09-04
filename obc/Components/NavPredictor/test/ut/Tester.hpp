/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * NavPredictor Component Unit Test Harness Header
 * ============================================================================
 */

#ifndef OBC_NAVPREDICTOR_TESTER_HPP_
#define OBC_NAVPREDICTOR_TESTER_HPP_

#include "obc/Components/NavPredictor/NavPredictor.hpp"
#include "NavPredictorGTestBase.hpp"
#include "gtest/gtest.h"

namespace Obc {

  class NavPredictorTester : public NavPredictorGTestBase, public ::testing::Test {

    public:

      // Maximum history storage for port calls, events, and telemetry
      static constexpr U32 MAX_HISTORY_SIZE = 100;
      // Test instance ID supplied to the component
      static constexpr FwEnumStoreType TEST_INSTANCE_ID = 0;
      // Queue depth supplied to active component queue
      static constexpr FwSizeType TEST_INSTANCE_QUEUE_DEPTH = 10;

      //! Construct test harness
      NavPredictorTester();

      //! Destructor
      ~NavPredictorTester() override;

      // ----------------------------------------------------------------------
      // Test helper routines
      // ----------------------------------------------------------------------

      //! Synthesize raw NMEA GPS byte stream and invoke input port
      void sendNmeaSentence(const char* sentence);

      //! Advance periodic scheduler tick (10 Hz)
      void sendSchedTick(U32 context = 0);

      //! Configure mock hardware crash switch logic level (PB8)
      void setCrashPinState(Fw::Logic state);

      //! Dispatch NAV_SET_DEPLOY_ALT command
      void sendCmdSetDeployAlt(F32 deployAltM, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch NAV_ARM_PARACHUTE command
      void sendCmdArmParachute(Fw::Enabled arm, Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

      //! Dispatch NAV_FORCE_DEPLOY command
      void sendCmdForceDeploy(Fw::CmdResponse expectedResponse = Fw::CmdResponse::OK);

    private:

      // ----------------------------------------------------------------------
      // Handlers for outbound ports from component under test
      // ----------------------------------------------------------------------

      //! Intercept parachute deployer trigger invocation
      void from_deployOut_handler(
          FwIndexType portNum,
          U32 confidence,
          bool force
      ) override;

      //! Intercept mechanical crash switch GPIO read invocation
      Drv::GpioStatus from_crashGpioIn_handler(
          FwIndexType portNum,
          Fw::Logic& state
      ) override;

      // ----------------------------------------------------------------------
      // Harness initialization
      // ----------------------------------------------------------------------

      void connectPorts();
      void initComponents();

    private:

      // Component under test
      NavPredictor component;

      // Mock hardware state
      Fw::Logic m_crashPinState;

      // Buffer storage for synthetic NMEA packets
      uint8_t m_nmeaBuf[256];
      Fw::Buffer m_fprimeBuffer;

  };

  typedef NavPredictorTester Tester;

}

#endif /* OBC_NAVPREDICTOR_TESTER_HPP_ */
