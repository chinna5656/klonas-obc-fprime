module Obc {

  @ Environmental and Attitude Multi-Sensor orchestrator on shared SPI1 bus
  active component EnvSensors {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Periodic scheduler port (10Hz attitude sampling, 1Hz atmospheric sampling)
    sync input port schedIn: Svc.Sched

    @ SPI write/read transaction port on shared SPI1 bus
    output port spiOut: Drv.SpiWriteRead

    @ Chip Select line for BNO08X IMU (PA4)
    output port csBnoOut: Drv.GpioWrite

    @ Chip Select line for BMP280 Internal Barometer (PB2)
    output port csBmpOut: Drv.GpioWrite

    @ Chip Select line for BME680 External Gas/Env (PB6)
    output port csBmeOut: Drv.GpioWrite

    # ----------------------------------------------------------------------
    # Special ports
    # ----------------------------------------------------------------------

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    event port Log
    text event port LogText
    time get port Time
    telemetry port Tlm

    # ----------------------------------------------------------------------
    # Commands
    # ----------------------------------------------------------------------

    @ Re-initialize all sensors on the SPI1 bus
    async command ENV_INIT_SENSORS()

    @ Set baseline sea-level atmospheric pressure for altitude calculations
    async command ENV_SET_SEA_LEVEL_PRESSURE(
      seaLevelHpa: F32 @< Reference standard pressure in hPa (default 1013.25)
    )

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ BMP280 Internal Temperature in degrees Celsius
    telemetry Bmp_InternalTemp: F32

    @ BMP280 Internal Pressure in hPa
    telemetry Bmp_InternalPressure: F32

    @ BMP280 Estimated Internal Barometric Altitude in meters
    telemetry Bmp_InternalAltitude: F32

    @ BME680 External Temperature in degrees Celsius
    telemetry Bme_ExternalTemp: F32

    @ BME680 External Pressure in hPa
    telemetry Bme_ExternalPressure: F32

    @ BME680 External Relative Humidity in %
    telemetry Bme_ExternalHumidity: F32

    @ BME680 Gas Resistance in Ohms
    telemetry Bme_GasResistance: F32

    @ BNO08X Roll Euler angle in degrees (-180 to +180)
    telemetry Imu_Roll: F32

    @ BNO08X Pitch Euler angle in degrees (-90 to +90)
    telemetry Imu_Pitch: F32

    @ BNO08X Yaw Euler angle in degrees (0 to 360)
    telemetry Imu_Yaw: F32

    @ BNO08X Linear Acceleration X in m/s^2
    telemetry Imu_AccX: F32

    @ BNO08X Linear Acceleration Y in m/s^2
    telemetry Imu_AccY: F32

    @ BNO08X Linear Acceleration Z in m/s^2
    telemetry Imu_AccZ: F32

    @ Total SPI1 transaction errors on shared bus
    telemetry Spi1ErrorCount: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when SPI1 sensors initialize
    event SensorsInitSuccess(sensorMask: U8) \
      severity activity high \
      id 0 \
      format "SPI1 sensors initialized successfully with mask {}"

    @ Event emitted when a sensor transaction fails
    event SensorReadError(sensorId: U8, status: U8) \
      severity warning high \
      id 1 \
      format "SPI1 Sensor {} read failed with status {}"

  }

}
