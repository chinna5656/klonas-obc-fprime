/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * EnvSensors Component Header
 * ============================================================================
 */

#ifndef OBC_ENVSENSORS_HPP_
#define OBC_ENVSENSORS_HPP_

#include "obc/Components/EnvSensors/EnvSensorsComponentAc.hpp"

namespace Obc {

  class EnvSensors : public EnvSensorsComponentBase {

    public:

      //! Sensor identifier constants
      static constexpr U8 SENSOR_ID_BNO = 1;
      static constexpr U8 SENSOR_ID_BMP = 2;
      static constexpr U8 SENSOR_ID_BME = 3;

      //! Standard sea-level reference pressure in hPa
      static constexpr F32 STANDARD_SEA_LEVEL_HPA = 1013.25f;

      //! Static SPI transaction buffer size (sized strictly for 128KB SRAM)
      static constexpr FwSizeType SPI_BUF_SIZE = 64;

      //! Construct EnvSensors instance
      EnvSensors(const char* const compName);

      //! Destructor
      ~EnvSensors() override = default;

      //! Initialize component
      void init(
          FwSizeType queueDepth,
          FwEnumStoreType instance = 0
      );

    private:

      // ----------------------------------------------------------------------
      // Handlers for input ports
      // ----------------------------------------------------------------------

      void schedIn_handler(
          FwIndexType portNum,
          U32 context
      ) override;

      // ----------------------------------------------------------------------
      // Command handlers
      // ----------------------------------------------------------------------

      void ENV_INIT_SENSORS_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq
      ) override;

      void ENV_SET_SEA_LEVEL_PRESSURE_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          F32 seaLevelHpa
      ) override;

      // ----------------------------------------------------------------------
      // Sensor reading & SPI arbitration helpers
      // ----------------------------------------------------------------------

      //! Execute isolated SPI transaction with designated chip select
      bool spiTransfer(U8 sensorId, const uint8_t* tx, uint8_t* rx, FwSizeType size);

      //! Read and calculate BMP280 internal temperature, pressure, altitude
      void readBmp280();

      //! Read and calculate BME680 external temp, pressure, humidity, gas
      void readBme680();

      //! Read and calculate BNO08X orientation quaternion, euler angles, acceleration
      void readBno08x();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      uint8_t m_txBuffer[SPI_BUF_SIZE];
      uint8_t m_rxBuffer[SPI_BUF_SIZE];
      Fw::Buffer m_fprimeTx;
      Fw::Buffer m_fprimeRx;

      F32 m_seaLevelPressureHpa;

      // BMP280 internal telemetry
      F32 m_bmpTemp;
      F32 m_bmpPress;
      F32 m_bmpAlt;

      // BME680 external telemetry
      F32 m_bmeTemp;
      F32 m_bmePress;
      F32 m_bmeHumidity;
      F32 m_bmeGas;

      // BNO08X IMU attitude telemetry
      F32 m_imuRoll;
      F32 m_imuPitch;
      F32 m_imuYaw;
      F32 m_imuAccX;
      F32 m_imuAccY;
      F32 m_imuAccZ;

      U32 m_spiErrors;
      U32 m_tickCount;

  };

}

#endif /* OBC_ENVSENSORS_HPP_ */
