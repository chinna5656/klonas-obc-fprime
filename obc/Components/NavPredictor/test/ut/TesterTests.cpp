/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * NavPredictor Component Unit Tests (GoogleTest Suite)
 * ============================================================================
 */

#include "Tester.hpp"
#include <gtest/gtest.h>

namespace Obc {

  // ==========================================================================
  // 1. NOMINAL FLIGHT SCENARIOS
  // ==========================================================================

  TEST_F(Tester, GpsFixAndPositionTracking) {
    // Initial State: No events or telemetry emitted yet
    ASSERT_EVENTS_GpsLockAcquired_SIZE(0);
    ASSERT_TLM_Latitude_SIZE(0);
    ASSERT_TLM_Longitude_SIZE(0);
    ASSERT_TLM_Altitude_SIZE(0);

    // Feed valid $GPGGA NMEA sentence (Munich coordinates, 545.4m altitude, fix quality 1, 8 sats)
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    this->sendNmeaSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    // Event is emitted immediately upon parsing valid fix
    ASSERT_EVENTS_GpsLockAcquired_SIZE(1);
    ASSERT_EVENTS_GpsLockAcquired(0, 8);

    // Advance 10 Hz scheduler tick to publish telemetry
    this->sendSchedTick();

    // Verify Telemetry
    ASSERT_TLM_Latitude_SIZE(1);
    ASSERT_TLM_Longitude_SIZE(1);
    ASSERT_TLM_Altitude_SIZE(1);
    ASSERT_TLM_GpsFix_SIZE(1);
    ASSERT_TLM_SatsInUse_SIZE(1);

    // 48 deg 07.038 min N = 48 + 7.038/60 = 48.1173 deg
    EXPECT_NEAR(this->tlmHistory_Latitude->at(0).arg, 48.1173, 0.001);
    // 011 deg 31.000 min E = 11 + 31.000/60 = 11.51667 deg
    EXPECT_NEAR(this->tlmHistory_Longitude->at(0).arg, 11.5167, 0.001);
    ASSERT_TLM_Altitude(0, 545.4f);
    ASSERT_TLM_GpsFix(0, 1);
    ASSERT_TLM_SatsInUse(0, 8);

    // Feed valid $GPRMC NMEA sentence (Ground speed: 22.4 knots = 11.52 m/s, Track angle: 84.4 deg)
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    this->sendNmeaSentence("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    this->sendSchedTick();

    ASSERT_TLM_GroundSpeed_SIZE(2);
    EXPECT_NEAR(this->tlmHistory_GroundSpeed->at(1).arg, 11.523f, 0.05f);
    ASSERT_TLM_TrackAngle_SIZE(2);
    EXPECT_NEAR(this->tlmHistory_TrackAngle->at(1).arg, 84.4f, 0.01f);
  }

  TEST_F(Tester, DescentRateAndLandingFootprint) {
    // Initial high-altitude baseline sample at 3000.0 m
    this->sendNmeaSentence("$GPGGA,120000,3746.494,N,12225.200,W,1,09,1.0,3000.0,M,0.0,M,,*53\r\n");
    this->sendNmeaSentence("$GPRMC,120000,A,3746.494,N,12225.200,W,020.0,090.0,230394,003.1,W*6A\r\n");
    this->sendSchedTick();

    this->clearHistory();

    // Descend to 2985.0 m (delta h = -15m in 1s -> descent rate ~15 m/s)
    this->sendNmeaSentence("$GPGGA,120001,3746.494,N,12225.200,W,1,09,1.0,2985.0,M,0.0,M,,*57\r\n");
    this->sendSchedTick();

    // Verify descent rate telemetry updated and positive
    ASSERT_TLM_DescentRate_SIZE(1);
    EXPECT_GT(this->tlmHistory_DescentRate->at(0).arg, 0.0f);

    // Verify TimeToImpact is finite and positive
    ASSERT_TLM_TimeToImpact_SIZE(1);
    EXPECT_GT(this->tlmHistory_TimeToImpact->at(0).arg, 0.0f);

    // Verify predicted landing coordinates
    ASSERT_TLM_PredictedLandingLat_SIZE(1);
    ASSERT_TLM_PredictedLandingLon_SIZE(1);
  }

  TEST_F(Tester, ApogeeDetection) {
    // Simulate ascent phase approaching apogee (2900m -> 2980m -> 3000m)
    this->sendNmeaSentence("$GPGGA,120000,3746.494,N,12225.200,W,1,09,1.0,2900.0,M,0.0,M,,*5B\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120001,3746.494,N,12225.200,W,1,09,1.0,2980.0,M,0.0,M,,*52\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120002,3746.494,N,12225.200,W,1,09,1.0,3000.0,M,0.0,M,,*51\r\n");
    this->sendSchedTick();

    // No apogee event yet while climbing
    ASSERT_EVENTS_ApogeeDetected_SIZE(0);

    // Transition to descent: Over the crest and descending (3000m -> 2980m -> 2950m -> 2900m)
    this->sendNmeaSentence("$GPGGA,120003,3746.494,N,12225.200,W,1,09,1.0,2980.0,M,0.0,M,,*50\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120004,3746.494,N,12225.200,W,1,09,1.0,2950.0,M,0.0,M,,*5A\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120005,3746.494,N,12225.200,W,1,09,1.0,2900.0,M,0.0,M,,*5E\r\n");
    this->sendSchedTick();

    // Apogee detected event emitted with maximum reached altitude (3000.0m)
    ASSERT_EVENTS_ApogeeDetected_SIZE(1);
    ASSERT_EVENTS_ApogeeDetected(0, 3000.0f);
  }

  // ==========================================================================
  // 2. OFF-NOMINAL / FAULT CONDITIONS
  // ==========================================================================

  TEST_F(Tester, MalformedStreamHandling) {
    // Feed incomplete, truncated, and nonsensical byte streams
    this->sendNmeaSentence("NON_NMEA_RANDOM_DATA\r\n");
    this->sendNmeaSentence("$UNKNOWN,1,2,3*00\r\n");
    this->sendNmeaSentence("$$$$\r\n");
    this->sendSchedTick();

    // Must not crash, no fix acquired, no false events
    ASSERT_EVENTS_GpsLockAcquired_SIZE(0);
    ASSERT_TLM_GpsFix_SIZE(1);
    ASSERT_TLM_GpsFix(0, 0);
  }

  TEST_F(Tester, GpsFixLostTransition) {
    // 1. Establish valid fix first
    this->sendNmeaSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    this->sendSchedTick();
    ASSERT_EVENTS_GpsLockAcquired_SIZE(1);

    this->clearHistory();

    // 2. Transition to Fix Quality 0 (GPS signal lost, tunnel, or RF blackout)
    // $GPGGA,123520,4807.038,N,01131.000,E,0,00,99.9,0.0,M,0.0,M,,*4F
    this->sendNmeaSentence("$GPGGA,123520,4807.038,N,01131.000,E,0,00,99.9,0.0,M,0.0,M,,*4F\r\n");
    this->sendSchedTick();

    // Warning event emitted
    ASSERT_EVENTS_GpsLockLost_SIZE(1);
    ASSERT_TLM_GpsFix_SIZE(1);
    ASSERT_TLM_GpsFix(0, 0);
  }

  // ==========================================================================
  // 3. GROUND COMMAND HANDLING
  // ==========================================================================

  TEST_F(Tester, SetDeployAltitudeCommand) {
    // Dispatch deploy altitude update
    this->sendCmdSetDeployAlt(750.0f, Fw::CmdResponse::OK);
  }

  TEST_F(Tester, ArmParachuteCommand) {
    // Verify parachute can be armed via ground command
    this->sendCmdArmParachute(Fw::Enabled::ENABLED, Fw::CmdResponse::OK);
    this->sendSchedTick();
    ASSERT_TLM_ParachuteArmed_SIZE(1);
    ASSERT_TLM_ParachuteArmed(0, true);

    this->clearHistory();

    // Verify parachute can be disarmed
    this->sendCmdArmParachute(Fw::Enabled::DISABLED, Fw::CmdResponse::OK);
    this->sendSchedTick();
    ASSERT_TLM_ParachuteArmed_SIZE(1);
    ASSERT_TLM_ParachuteArmed(0, false);
  }

  TEST_F(Tester, ForceDeployCommand) {
    ASSERT_from_deployOut_SIZE(0);
    ASSERT_EVENTS_ParachuteTriggered_SIZE(0);

    // Ground station manual override command
    this->sendCmdForceDeploy(Fw::CmdResponse::OK);

    // Outbound trigger port must be fired immediately with force=true and confidence=100
    ASSERT_from_deployOut_SIZE(1);
    ASSERT_from_deployOut(0, 100U, true);

    // Event and telemetry must reflect deployment
    ASSERT_EVENTS_ParachuteTriggered_SIZE(1);
    this->sendSchedTick();
    ASSERT_TLM_ParachuteTriggered_SIZE(1);
    ASSERT_TLM_ParachuteTriggered(0, true);
  }

  // ==========================================================================
  // 4. SAFETY & ACTUATION LOGIC
  // ==========================================================================

  TEST_F(Tester, InhibitDeploymentAboveAltitude) {
    // Arm parachute and configure deploy altitude at 800m
    this->sendCmdArmParachute(Fw::Enabled::ENABLED, Fw::CmdResponse::OK);
    this->sendCmdSetDeployAlt(800.0f, Fw::CmdResponse::OK);
    this->clearHistory();

    // High altitude (3000m) with rapid descent (20 m/s)
    this->sendNmeaSentence("$GPGGA,120000,3746.494,N,12225.200,W,1,09,1.0,3000.0,M,0.0,M,,*53\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120001,3746.494,N,12225.200,W,1,09,1.0,2980.0,M,0.0,M,,*56\r\n");
    this->sendSchedTick();

    this->sendNmeaSentence("$GPGGA,120002,3746.494,N,12225.200,W,1,09,1.0,2960.0,M,0.0,M,,*58\r\n");
    this->sendSchedTick();

    // Parachute MUST remain strictly inhibited above 800m
    ASSERT_from_deployOut_SIZE(0);
    ASSERT_EVENTS_ParachuteTriggered_SIZE(0);
  }

  TEST_F(Tester, AutoDeploymentUnderCriteria) {
    // Arm parachute and configure deploy altitude 800m
    this->sendCmdArmParachute(Fw::Enabled::ENABLED, Fw::CmdResponse::OK);
    this->sendCmdSetDeployAlt(800.0f, Fw::CmdResponse::OK);
    this->clearHistory();

    // Baseline sample at 800m
    this->sendNmeaSentence("$GPGGA,120004,3746.494,N,12225.200,W,1,09,1.0,800.0,M,0.0,M,,*6A\r\n");
    this->sendSchedTick();

    // Cycle 1: 780m (descent rate = 20 m/s > 3.0 m/s threshold)
    this->sendNmeaSentence("$GPGGA,120005,3746.494,N,12225.200,W,1,09,1.0,780.0,M,0.0,M,,*69\r\n");
    this->sendSchedTick();

    // Cycle 2: 760m
    this->sendNmeaSentence("$GPGGA,120006,3746.494,N,12225.200,W,1,09,1.0,760.0,M,0.0,M,,*67\r\n");
    this->sendSchedTick();

    // Cycle 3: 740m -> Reaches 3 consecutive cycles under threshold
    this->sendNmeaSentence("$GPGGA,120007,3746.494,N,12225.200,W,1,09,1.0,740.0,M,0.0,M,,*65\r\n");
    this->sendSchedTick();

    // Criteria met event logged
    ASSERT_EVENTS_ParachuteCriteriaMet_SIZE(1);

    // Parachute deployer trigger invoked with 95% confidence, force=false
    ASSERT_from_deployOut_SIZE(1);
    ASSERT_from_deployOut(0, 95U, false);

    // Trigger event logged
    ASSERT_EVENTS_ParachuteTriggered_SIZE(1);

    // Verify telemetry
    ASSERT_TLM_ParachuteTriggered_SIZE(4);
    ASSERT_TLM_ParachuteTriggered(3, true);
  }

  TEST_F(Tester, CrashImpactSwitchMonitor) {
    // Normal flight: Switch open (LOW)
    this->setCrashPinState(Fw::Logic::LOW);
    this->sendSchedTick();
    ASSERT_EVENTS_CrashImpactDetected_SIZE(0);
    ASSERT_TLM_CrashDetected_SIZE(1);
    ASSERT_TLM_CrashDetected(0, false);

    // Impact event: Switch closes on impact (HIGH on PB8)
    this->setCrashPinState(Fw::Logic::HIGH);
    this->sendSchedTick();

    // Impact detected warning event must be emitted
    ASSERT_EVENTS_CrashImpactDetected_SIZE(1);
    ASSERT_TLM_CrashDetected_SIZE(2);
    ASSERT_TLM_CrashDetected(1, true);
  }

}

// ============================================================================
// Main entry point for GoogleTest runner
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
