/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * NavPredictor Component Implementation
 * ============================================================================
 */

#include "obc/Components/NavPredictor/NavPredictor.hpp"
#include <Fw/Types/Assert.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Obc {

  NavPredictor::NavPredictor(const char* const compName) :
    NavPredictorComponentBase(compName),
    m_nmeaIdx(0),
    m_lat(0.0),
    m_lon(0.0),
    m_alt(0.0f),
    m_prevAlt(0.0f),
    m_descentRate(0.0f),
    m_groundSpeed(0.0f),
    m_trackAngle(0.0f),
    m_predictedLandingLat(0.0),
    m_predictedLandingLon(0.0),
    m_timeToImpact(0.0f),
    m_maxAlt(0.0f),
    m_apogeeDetected(false),
    m_consecutiveDescentCount(0),
    m_gpsFix(0),
    m_satsInUse(0),
    m_prevLock(false),
    m_crashDetected(false),
    m_parachuteArmed(false),
    m_parachuteTriggered(false),
    m_deployAltThreshold(1000.0f) // 1000 m MSL default
  {
    memset(m_nmeaBuf, 0, sizeof(m_nmeaBuf));
  }

  void NavPredictor::init(
      FwSizeType queueDepth,
      FwEnumStoreType instance
  ) {
    NavPredictorComponentBase::init(queueDepth, instance);
  }

  void NavPredictor::gpsDataIn_handler(
      FwIndexType portNum,
      Fw::Buffer& buffer,
      const Drv::ByteStreamStatus& status
  ) {
    FW_ASSERT(portNum == 0);

    if (status != Drv::ByteStreamStatus::OP_OK) {
      return;
    }

    const uint8_t* bytes = buffer.getData();
    const FwSizeType size = buffer.getSize();

    if (bytes == nullptr || size == 0) {
      return;
    }

    for (FwSizeType i = 0; i < size; ++i) {
      this->processNmeaChar(static_cast<char>(bytes[i]));
    }
  }

  void NavPredictor::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    // 1. Monitor crash pin (PB8)
    if (this->isConnected_crashGpioIn_OutputPort(0)) {
      Fw::Logic crashPinState = Fw::Logic::LOW;
      Drv::GpioStatus status = this->crashGpioIn_out(0, crashPinState);
      if (status == Drv::GpioStatus::OP_OK && crashPinState == Fw::Logic::HIGH) {
        if (!m_crashDetected) {
          m_crashDetected = true;
          this->log_WARNING_HI_CrashImpactDetected();

          // Trigger emergency parachute deployment upon crash impact
          if (!m_parachuteTriggered && this->isConnected_deployOut_OutputPort(0)) {
            m_parachuteTriggered = true;
            this->log_ACTIVITY_HI_ParachuteTriggered(true);
            this->deployOut_out(0, 100, true);
          }
        }
      }
    }

    // 2. Evaluate parachute deployment logic
    this->evaluateParachuteCriteria();

    // 3. Update telemetry channels
    this->tlmWrite_Latitude(m_lat);
    this->tlmWrite_Longitude(m_lon);
    this->tlmWrite_Altitude(m_alt);
    this->tlmWrite_DescentRate(m_descentRate);
    this->tlmWrite_GroundSpeed(m_groundSpeed);
    this->tlmWrite_TrackAngle(m_trackAngle);
    this->tlmWrite_PredictedLandingLat(m_predictedLandingLat);
    this->tlmWrite_PredictedLandingLon(m_predictedLandingLon);
    this->tlmWrite_TimeToImpact(m_timeToImpact);
    this->tlmWrite_GpsFix(m_gpsFix);
    this->tlmWrite_SatsInUse(m_satsInUse);
    this->tlmWrite_CrashDetected(m_crashDetected);
    this->tlmWrite_ApogeeDetected(m_apogeeDetected);
    this->tlmWrite_ParachuteArmed(m_parachuteArmed);
    this->tlmWrite_ParachuteTriggered(m_parachuteTriggered);
  }

  void NavPredictor::NAV_SET_DEPLOY_ALT_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      F32 altThresholdM
  ) {
    m_deployAltThreshold = altThresholdM;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void NavPredictor::NAV_ARM_PARACHUTE_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      Fw::Enabled arm
  ) {
    m_parachuteArmed = (arm == Fw::Enabled::ENABLED);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void NavPredictor::NAV_FORCE_DEPLOY_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    m_parachuteTriggered = true;
    this->log_ACTIVITY_HI_ParachuteTriggered(true);

    if (this->isConnected_deployOut_OutputPort(0)) {
      this->deployOut_out(0, 100, true);
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void NavPredictor::processNmeaChar(char c) {
    if (c == '$') {
      m_nmeaIdx = 0;
      m_nmeaBuf[m_nmeaIdx++] = c;
    } else if (c == '\r' || c == '\n') {
      if (m_nmeaIdx > 0 && m_nmeaIdx < NMEA_BUF_LEN) {
        m_nmeaBuf[m_nmeaIdx] = '\0';
        this->parseNmeaSentence(m_nmeaBuf);
      }
      m_nmeaIdx = 0;
    } else {
      if (m_nmeaIdx < (NMEA_BUF_LEN - 1)) {
        m_nmeaBuf[m_nmeaIdx++] = c;
      } else {
        // Line buffer overflow reset
        m_nmeaIdx = 0;
      }
    }
  }

  void NavPredictor::parseNmeaSentence(const char* sentence) {
    if (sentence == nullptr || strlen(sentence) < 6) {
      return;
    }

    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
      this->parseGga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
      this->parseRmc(sentence);
    }
  }

  void NavPredictor::parseGga(const char* sentence) {
    // Copy sentence into static parse buffer for zero-alloc tokenization
    char buf[NMEA_BUF_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* tokens[15] = {nullptr};
    char* p = buf;
    U8 tokCount = 0;

    while (p != nullptr && tokCount < 15) {
      tokens[tokCount++] = p;
      char* comma = strchr(p, ',');
      if (comma != nullptr) {
        *comma = '\0';
        p = comma + 1;
      } else {
        break;
      }
    }

    // $GPGGA tokens:
    // 0: Header, 1: UTC, 2: Lat, 3: N/S, 4: Lon, 5: E/W, 6: Quality, 7: NumSV, 8: HDOP, 9: Alt, 10: M
    if (tokCount >= 10) {
      // Fix Quality (token 6)
      U8 fix = static_cast<U8>(atoi(tokens[6]));
      m_gpsFix = fix;

      if (fix > 0) {
        if (!m_prevLock) {
          m_prevLock = true;
          this->log_ACTIVITY_HI_GpsLockAcquired(static_cast<U8>(atoi(tokens[7])));
        }

        m_satsInUse = static_cast<U8>(atoi(tokens[7]));

        // Latitude (ddmm.mmmm)
        if (strlen(tokens[2]) >= 4) {
          double rawLat = atof(tokens[2]);
          int deg = static_cast<int>(rawLat / 100.0);
          double min = rawLat - (deg * 100.0);
          m_lat = deg + (min / 60.0);
          if (tokens[3][0] == 'S') {
            m_lat = -m_lat;
          }
        }

        // Longitude (dddmm.mmmm)
        if (strlen(tokens[4]) >= 5) {
          double rawLon = atof(tokens[4]);
          int deg = static_cast<int>(rawLon / 100.0);
          double min = rawLon - (deg * 100.0);
          m_lon = deg + (min / 60.0);
          if (tokens[5][0] == 'W') {
            m_lon = -m_lon;
          }
        }

        // Altitude
        F32 parsedAlt = static_cast<F32>(atof(tokens[9]));
        this->updateDescentRate(parsedAlt, 1.0f); // 1.0s typical GPS sample period
      } else {
        if (m_prevLock) {
          m_prevLock = false;
          this->log_WARNING_HI_GpsLockLost();
        }
      }
    }
  }

  void NavPredictor::parseRmc(const char* sentence) {
    char buf[NMEA_BUF_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* tokens[13] = {nullptr};
    char* p = buf;
    U8 tokCount = 0;

    while (p != nullptr && tokCount < 13) {
      tokens[tokCount++] = p;
      char* comma = strchr(p, ',');
      if (comma != nullptr) {
        *comma = '\0';
        p = comma + 1;
      } else {
        break;
      }
    }

    // $GPRMC tokens:
    // 0: Header, 1: UTC, 2: Status, 3: Lat, 4: N/S, 5: Lon, 6: E/W, 7: Speed(knots), 8: TrackAngle(deg)
    if (tokCount >= 9) {
      if (tokens[2][0] == 'A') { // Valid status
        // Speed in knots converted to m/s: 1 knot = 0.514444 m/s
        F32 speedKnots = static_cast<F32>(atof(tokens[7]));
        m_groundSpeed = speedKnots * 0.514444f;

        // Track angle (Course over ground)
        m_trackAngle = static_cast<F32>(atof(tokens[8]));

        this->computeLandingFootprint();
      }
    }
  }

  void NavPredictor::updateDescentRate(F32 currentAltM, F32 dt) {
    if (dt <= 0.0f) {
      dt = 1.0f;
    }

    if (m_prevAlt == 0.0f) {
      m_prevAlt = currentAltM;
      m_alt = currentAltM;
      m_maxAlt = currentAltM;
      m_descentRate = 0.0f;
      return;
    }

    // Vertical velocity (Vz = -dh/dt: positive downwards)
    F32 rawVz = -(currentAltM - m_prevAlt) / dt;
    m_prevAlt = currentAltM;
    m_alt = currentAltM;

    // First-order complementary low-pass filter (alpha = 0.25)
    // Vz_filtered = (1 - alpha) * Vz_prev + alpha * rawVz
    constexpr F32 ALPHA = 0.25f;
    m_descentRate = ((1.0f - ALPHA) * m_descentRate) + (ALPHA * rawVz);

    // Apogee detection tracking
    if (m_alt > m_maxAlt) {
      m_maxAlt = m_alt;
    }

    // Apogee detection condition:
    // Altitude reached > 200m MSL, dropped > 15m below peak, and positive descent rate > 2.0 m/s
    if (!m_apogeeDetected && m_maxAlt > 200.0f) {
      if ((m_maxAlt - m_alt) > 15.0f && m_descentRate > 2.0f) {
        m_apogeeDetected = true;
        this->log_ACTIVITY_HI_ApogeeDetected(m_maxAlt);
        this->tlmWrite_ApogeeDetected(true);
      }
    }

    this->computeLandingFootprint();
  }

  void NavPredictor::computeLandingFootprint() {
    constexpr F32 GROUND_ALT_MSL = 0.0f; // Reference ground altitude MSL

    if (m_descentRate > 0.5f && m_alt > GROUND_ALT_MSL) {
      // Time to touchdown (ttd = delta_h / Vz)
      m_timeToImpact = (m_alt - GROUND_ALT_MSL) / m_descentRate;

      // Horizontal velocity components (NED frame)
      F64 courseRad = static_cast<F64>(m_trackAngle) * DEG_TO_RAD;
      F64 vNorth = static_cast<F64>(m_groundSpeed) * cos(courseRad);
      F64 vEast  = static_cast<F64>(m_groundSpeed) * sin(courseRad);

      // Displacements
      F64 dNorth = vNorth * static_cast<F64>(m_timeToImpact);
      F64 dEast  = vEast  * static_cast<F64>(m_timeToImpact);

      // Spherical Geodetic Projection
      // Lat_proj = Lat + (dNorth / R_E) * (180 / pi)
      // Lon_proj = Lon + (dEast / (R_E * cos(Lat))) * (180 / pi)
      F64 dLatDeg = (dNorth / EARTH_RADIUS_M) * RAD_TO_DEG;
      F64 latRad = m_lat * DEG_TO_RAD;
      F64 cosLat = cos(latRad);
      if (fabs(cosLat) < 1e-6) {
        cosLat = 1e-6;
      }
      F64 dLonDeg = (dEast / (EARTH_RADIUS_M * cosLat)) * RAD_TO_DEG;

      m_predictedLandingLat = m_lat + dLatDeg;
      m_predictedLandingLon = m_lon + dLonDeg;
    } else {
      m_timeToImpact = 0.0f;
      m_predictedLandingLat = m_lat;
      m_predictedLandingLon = m_lon;
    }
  }

  void NavPredictor::evaluateParachuteCriteria() {
    if (!m_parachuteArmed || m_parachuteTriggered) {
      m_consecutiveDescentCount = 0;
      return;
    }

    // Safety Criteria:
    // 1. Apogee must be confirmed prior to enabling deployment
    // 2. Altitude <= threshold (e.g. 1000m MSL)
    // 3. Confirmed descent rate >= 3.0 m/s for at least 3 consecutive cycles
    if (m_apogeeDetected && m_alt <= m_deployAltThreshold && m_descentRate >= 3.0f) {
      m_consecutiveDescentCount++;
      if (m_consecutiveDescentCount >= 3) {
        m_parachuteTriggered = true;
        this->log_ACTIVITY_HI_ParachuteCriteriaMet(m_alt, m_descentRate);
        this->log_ACTIVITY_HI_ParachuteTriggered(false);

        if (this->isConnected_deployOut_OutputPort(0)) {
          this->deployOut_out(0, 95, false); // 95% confidence automatic deployment
        }
      }
    } else {
      if (m_consecutiveDescentCount > 0) {
        m_consecutiveDescentCount--;
      }
    }
  }

}
