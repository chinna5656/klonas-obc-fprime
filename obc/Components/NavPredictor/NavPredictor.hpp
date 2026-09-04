/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * NavPredictor Component Header
 * ============================================================================
 */

#ifndef OBC_NAVPREDICTOR_HPP_
#define OBC_NAVPREDICTOR_HPP_

#include "obc/Components/NavPredictor/NavPredictorComponentAc.hpp"

namespace Obc {

  class NavPredictor : public NavPredictorComponentBase {

    public:

      //! Earth mean radius in meters (WGS-84 spherical approximation)
      static constexpr F64 EARTH_RADIUS_M = 6371000.0;

      //! Mathematical Pi
      static constexpr F64 PI_VAL = 3.14159265358979323846;

      //! Degrees to Radians conversion factor
      static constexpr F64 DEG_TO_RAD = PI_VAL / 180.0;

      //! Radians to Degrees conversion factor
      static constexpr F64 RAD_TO_DEG = 180.0 / PI_VAL;

      //! NMEA line buffer size (sized strictly for STM32 128KB limit)
      static constexpr FwSizeType NMEA_BUF_LEN = 128;

      //! Construct NavPredictor instance
      NavPredictor(const char* const compName);

      //! Destructor
      ~NavPredictor() override = default;

      //! Initialize component
      void init(
          FwSizeType queueDepth,
          FwEnumStoreType instance = 0
      );

    private:

      // ----------------------------------------------------------------------
      // Handlers for input ports
      // ----------------------------------------------------------------------

      //! Handler for incoming GPS byte stream
      void gpsDataIn_handler(
          FwIndexType portNum,
          Fw::Buffer& buffer,
          const Drv::ByteStreamStatus& status
      ) override;

      //! Periodic schedule handler (typically 1 Hz to 10 Hz)
      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      // ----------------------------------------------------------------------
      // Command handlers
      // ----------------------------------------------------------------------

      void NAV_SET_DEPLOY_ALT_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          F32 altThresholdM
      ) override;

      void NAV_ARM_PARACHUTE_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          Fw::Enabled arm
      ) override;

      void NAV_FORCE_DEPLOY_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      // ----------------------------------------------------------------------
      // Mathematical & Navigation Processing Routines
      // ----------------------------------------------------------------------

      //! Feed single character into NMEA state machine
      void processNmeaChar(char c);

      //! Parse complete NMEA sentence
      void parseNmeaSentence(const char* sentence);

      //! Parse GGA sentence for position, fix quality, and altitude
      void parseGga(const char* sentence);

      //! Parse RMC sentence for ground speed and track angle
      void parseRmc(const char* sentence);

      //! Update filtered descent rate (Vz = -dh/dt)
      void updateDescentRate(F32 currentAltM, F32 dt);

      //! Compute real-time landing footprint coordinates
      void computeLandingFootprint();

      //! Evaluate safety criteria for parachute deployment
      void evaluateParachuteCriteria();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      char m_nmeaBuf[NMEA_BUF_LEN];
      U16 m_nmeaIdx;

      F64 m_lat;
      F64 m_lon;
      F32 m_alt;
      F32 m_prevAlt;
      F32 m_descentRate;
      F32 m_groundSpeed;
      F32 m_trackAngle;

      F64 m_predictedLandingLat;
      F64 m_predictedLandingLon;
      F32 m_timeToImpact;

      F32 m_maxAlt;
      bool m_apogeeDetected;
      U8 m_consecutiveDescentCount;

      U8 m_gpsFix;
      U8 m_satsInUse;
      bool m_prevLock;

      bool m_crashDetected;
      bool m_parachuteArmed;
      bool m_parachuteTriggered;

      F32 m_deployAltThreshold;

  };

}

#endif /* OBC_NAVPREDICTOR_HPP_ */
