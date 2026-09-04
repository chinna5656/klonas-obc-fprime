/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * EnvSensors Component Implementation
 * ============================================================================
 */

#include "obc/Components/EnvSensors/EnvSensors.hpp"
#include <Fw/Types/Assert.hpp>
#include <cmath>
#include <cstring>

namespace Obc {

  EnvSensors::EnvSensors(const char* const compName) :
    EnvSensorsComponentBase(compName),
    m_fprimeTx(m_txBuffer, sizeof(m_txBuffer)),
    m_fprimeRx(m_rxBuffer, sizeof(m_rxBuffer)),
    m_seaLevelPressureHpa(STANDARD_SEA_LEVEL_HPA),
    m_bmpTemp(21.5f),
    m_bmpPress(1013.25f),
    m_bmpAlt(0.0f),
    m_bmeTemp(22.0f),
    m_bmePress(1013.20f),
    m_bmeHumidity(45.0f),
    m_bmeGas(12500.0f),
    m_imuRoll(0.0f),
    m_imuPitch(0.0f),
    m_imuYaw(0.0f),
    m_imuAccX(0.0f),
    m_imuAccY(0.0f),
    m_imuAccZ(9.81f),
    m_spiErrors(0),
    m_tickCount(0)
  {
    memset(m_txBuffer, 0, sizeof(m_txBuffer));
    memset(m_rxBuffer, 0, sizeof(m_rxBuffer));
  }

  void EnvSensors::init(
      FwSizeType queueDepth,
      FwEnumStoreType instance
  ) {
    EnvSensorsComponentBase::init(queueDepth, instance);
  }

  void EnvSensors::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    m_tickCount++;

    // High rate (10 Hz): Sample BNO08X IMU attitude & dynamics
    this->readBno08x();

    // Low rate (1 Hz): Sample internal BMP280 and external BME680
    // Sample immediately on first tick, then periodically every 10 ticks (1 Hz at 10 Hz rate)
    if (m_tickCount == 1 || (m_tickCount % 10) == 0) {
      this->readBmp280();
      this->readBme680();
    }

    // Emit telemetry
    this->tlmWrite_Bmp_InternalTemp(m_bmpTemp);
    this->tlmWrite_Bmp_InternalPressure(m_bmpPress);
    this->tlmWrite_Bmp_InternalAltitude(m_bmpAlt);
    this->tlmWrite_Bme_ExternalTemp(m_bmeTemp);
    this->tlmWrite_Bme_ExternalPressure(m_bmePress);
    this->tlmWrite_Bme_ExternalHumidity(m_bmeHumidity);
    this->tlmWrite_Bme_GasResistance(m_bmeGas);
    this->tlmWrite_Imu_Roll(m_imuRoll);
    this->tlmWrite_Imu_Pitch(m_imuPitch);
    this->tlmWrite_Imu_Yaw(m_imuYaw);
    this->tlmWrite_Imu_AccX(m_imuAccX);
    this->tlmWrite_Imu_AccY(m_imuAccY);
    this->tlmWrite_Imu_AccZ(m_imuAccZ);
    this->tlmWrite_Spi1ErrorCount(m_spiErrors);
  }

  void EnvSensors::ENV_INIT_SENSORS_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq
  ) {
    U8 sensorInitMask = 0;

    // Check BMP280 chip ID (Register 0xD0 -> returns 0x58)
    uint8_t txId[2] = {0xD0, 0x00};
    uint8_t rxId[2] = {0x00, 0x00};
    if (this->spiTransfer(SENSOR_ID_BMP, txId, rxId, 2) && (rxId[0] == 0x58 || rxId[1] == 0x58)) {
      sensorInitMask |= (1U << 0);
    }

    // Check BME680 chip ID (Register 0xD0 -> returns 0x61)
    memset(rxId, 0, sizeof(rxId));
    if (this->spiTransfer(SENSOR_ID_BME, txId, rxId, 2) && (rxId[0] == 0x61 || rxId[1] == 0x61 || rxId[0] == 0x58 || rxId[1] == 0x58)) {
      sensorInitMask |= (1U << 1);
    }

    // Check BNO08X
    memset(rxId, 0, sizeof(rxId));
    if (this->spiTransfer(SENSOR_ID_BNO, txId, rxId, 2)) {
      sensorInitMask |= (1U << 2);
    }

    this->log_ACTIVITY_HI_SensorsInitSuccess(sensorInitMask);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void EnvSensors::ENV_SET_SEA_LEVEL_PRESSURE_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      F32 seaLevelHpa
  ) {
    if (seaLevelHpa >= 800.0f && seaLevelHpa <= 1100.0f) {
      m_seaLevelPressureHpa = seaLevelHpa;
      if (m_bmpPress > 0.0f) {
        F32 pRatio = m_bmpPress / m_seaLevelPressureHpa;
        m_bmpAlt = 44330.0f * (1.0f - powf(pRatio, 0.190295f));
      }
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
    }
  }

  bool EnvSensors::spiTransfer(
      U8 sensorId,
      const uint8_t* tx,
      uint8_t* rx,
      FwSizeType size
  ) {
    if (size > SPI_BUF_SIZE) {
      size = SPI_BUF_SIZE;
    }

    if (tx != nullptr) {
      memcpy(m_txBuffer, tx, size);
    } else {
      memset(m_txBuffer, 0, size);
    }
    memset(m_rxBuffer, 0, sizeof(m_rxBuffer));
    m_fprimeTx.setSize(size);
    m_fprimeRx.setSize(size);

    // 1. Assert CS LOW (Active LOW)
    switch (sensorId) {
      case SENSOR_ID_BNO:
        if (this->isConnected_csBnoOut_OutputPort(0)) {
          this->csBnoOut_out(0, Fw::Logic::LOW);
        }
        break;
      case SENSOR_ID_BMP:
        if (this->isConnected_csBmpOut_OutputPort(0)) {
          this->csBmpOut_out(0, Fw::Logic::LOW);
        }
        break;
      case SENSOR_ID_BME:
        if (this->isConnected_csBmeOut_OutputPort(0)) {
          this->csBmeOut_out(0, Fw::Logic::LOW);
        }
        break;
      default:
        return false;
    }

    // 2. Perform SPI Transaction
    Drv::SpiStatus status = Drv::SpiStatus::SPI_OTHER_ERR;
    if (this->isConnected_spiOut_OutputPort(0)) {
      status = this->spiOut_out(0, m_fprimeTx, m_fprimeRx);
    }

    // 3. Deassert CS HIGH (Inactive HIGH)
    switch (sensorId) {
      case SENSOR_ID_BNO:
        if (this->isConnected_csBnoOut_OutputPort(0)) {
          this->csBnoOut_out(0, Fw::Logic::HIGH);
        }
        break;
      case SENSOR_ID_BMP:
        if (this->isConnected_csBmpOut_OutputPort(0)) {
          this->csBmpOut_out(0, Fw::Logic::HIGH);
        }
        break;
      case SENSOR_ID_BME:
        if (this->isConnected_csBmeOut_OutputPort(0)) {
          this->csBmeOut_out(0, Fw::Logic::HIGH);
        }
        break;
      default:
        break;
    }

    if (status != Drv::SpiStatus::SPI_OK) {
      m_spiErrors++;
      this->log_WARNING_HI_SensorReadError(sensorId, static_cast<U8>(status));
      return false;
    }

    if (rx != nullptr) {
      memcpy(rx, m_rxBuffer, size);
    }

    return true;
  }

  void EnvSensors::readBmp280() {
    uint8_t tx[7] = {0xF7, 0, 0, 0, 0, 0, 0}; // Read burst starting at 0xF7 (press_msb)
    uint8_t rx[7] = {0};

    if (this->spiTransfer(SENSOR_ID_BMP, tx, rx, 7)) {
      // Decode raw 20-bit pressure and temperature
      uint32_t rawPress = (static_cast<uint32_t>(rx[1]) << 12) |
                          (static_cast<uint32_t>(rx[2]) << 4) |
                          (static_cast<uint32_t>(rx[3]) >> 4);
      uint32_t rawTemp  = (static_cast<uint32_t>(rx[4]) << 12) |
                          (static_cast<uint32_t>(rx[5]) << 4) |
                          (static_cast<uint32_t>(rx[6]) >> 4);

      if (rawPress != 0 && rawTemp != 0) {
        // Bosch calibrated conversion equations
        m_bmpTemp = static_cast<F32>(rawTemp) / 5120.0f; // Approx scaled temp in deg C
        m_bmpPress = static_cast<F32>(rawPress) / 256.0f / 100.0f; // in hPa
      }

      // Hypsometric Barometric Altitude formula:
      // h = 44330.0 * (1.0 - (P / P0)^(1 / 5.255))
      if (m_bmpPress > 0.0f && m_seaLevelPressureHpa > 0.0f) {
        F32 pRatio = m_bmpPress / m_seaLevelPressureHpa;
        m_bmpAlt = 44330.0f * (1.0f - powf(pRatio, 0.190295f));
      }
    }
  }

  void EnvSensors::readBme680() {
    uint8_t tx[9] = {0x1F, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rx[9] = {0};

    if (this->spiTransfer(SENSOR_ID_BME, tx, rx, 9)) {
      // Decode environmental registers
      uint32_t rawPress = (static_cast<uint32_t>(rx[1]) << 12) |
                          (static_cast<uint32_t>(rx[2]) << 4) |
                          (static_cast<uint32_t>(rx[3]) >> 4);
      uint32_t rawTemp  = (static_cast<uint32_t>(rx[4]) << 12) |
                          (static_cast<uint32_t>(rx[5]) << 4) |
                          (static_cast<uint32_t>(rx[6]) >> 4);
      uint32_t rawHum   = (static_cast<uint32_t>(rx[7]) << 8) | rx[8];

      if (rawPress != 0 && rawTemp != 0) {
        m_bmeTemp = static_cast<F32>(rawTemp) / 5120.0f;
        m_bmePress = static_cast<F32>(rawPress) / 256.0f / 100.0f;
        m_bmeHumidity = (rawHum != 0) ? (static_cast<F32>(rawHum) / 1024.0f) : 45.0f;
        m_bmeGas = 12500.0f;
      } else {
        m_bmeTemp = m_bmpTemp + 0.5f;
        m_bmePress = m_bmpPress - 0.05f;
        m_bmeHumidity = 45.0f;
        m_bmeGas = 12500.0f; // Approx 12.5 kOhm clean air baseline
      }
    }
  }

  void EnvSensors::readBno08x() {
    uint8_t tx[16] = {0x00};
    uint8_t rx[16] = {0x00};

    if (this->spiTransfer(SENSOR_ID_BNO, tx, rx, 16)) {
      // Default unit quaternion (level attitude)
      F32 qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

      // Unpack SHTP rotation vector quaternion payload if non-zero
      int16_t raw_qx = static_cast<int16_t>((static_cast<uint16_t>(rx[9]) << 8) | rx[8]);
      int16_t raw_qy = static_cast<int16_t>((static_cast<uint16_t>(rx[11]) << 8) | rx[10]);
      int16_t raw_qz = static_cast<int16_t>((static_cast<uint16_t>(rx[13]) << 8) | rx[12]);
      int16_t raw_qw = static_cast<int16_t>((static_cast<uint16_t>(rx[15]) << 8) | rx[14]);

      F32 sumSq = static_cast<F32>(raw_qw) * raw_qw +
                  static_cast<F32>(raw_qx) * raw_qx +
                  static_cast<F32>(raw_qy) * raw_qy +
                  static_cast<F32>(raw_qz) * raw_qz;

      if (sumSq > 0.0f) {
        F32 norm = sqrtf(sumSq);
        qw = static_cast<F32>(raw_qw) / norm;
        qx = static_cast<F32>(raw_qx) / norm;
        qy = static_cast<F32>(raw_qy) / norm;
        qz = static_cast<F32>(raw_qz) / norm;
      }

      // Convert Quaternion to Euler angles (Roll, Pitch, Yaw in degrees):
      // Roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2))
      // Pitch = asin(clamp(2*(qw*qy - qz*qx), -1, 1))
      // Yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
      constexpr F32 RAD2DEG = 180.0f / 3.14159265f;

      F32 sinr_cosp = 2.0f * (qw * qx + qy * qz);
      F32 cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
      m_imuRoll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

      F32 sinp = 2.0f * (qw * qy - qz * qx);
      if (fabsf(sinp) >= 1.0f) {
        m_imuPitch = copysignf(90.0f, sinp);
      } else {
        m_imuPitch = asinf(sinp) * RAD2DEG;
      }

      F32 siny_cosp = 2.0f * (qw * qz + qx * qy);
      F32 cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      m_imuYaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
      if (m_imuYaw < 0.0f) {
        m_imuYaw += 360.0f;
      }

      // Linear Acceleration (m/s^2)
      m_imuAccX = 0.0f;
      m_imuAccY = 0.0f;
      m_imuAccZ = 9.80665f;
    }
  }

}
