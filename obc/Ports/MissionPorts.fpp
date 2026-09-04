module Obc {

  @ Port to trigger parachute deployment
  port DeployTrigger(
    confidence: U32 @< Confidence score (0-100) or check flags
    force: bool     @< Force deployment override flag
  )

  @ Port to read ADC raw value from specified channel
  port AdcSample(
    channel: U32     @< ADC Channel (e.g. 0 for PA0)
    ref rawVal: U16  @< Destination for raw 12-bit ADC reading
  ) -> Fw.Success

  @ Structured telemetry log record for flight logging to MicroSD
  struct FlightLogRecord {
    timestampMs: U32   @< Monotonic timestamp in milliseconds
    lat: F64           @< GPS Latitude in degrees
    lon: F64           @< GPS Longitude in degrees
    alt: F32           @< GPS Altitude in meters MSL
    descentRate: F32   @< Filtered descent rate in m/s (Vz)
    roll: F32          @< Roll angle in degrees
    pitch: F32         @< Pitch angle in degrees
    yaw: F32           @< Yaw angle in degrees
    tempInt: F32       @< Internal temperature (BMP280) in deg C
    pressInt: F32      @< Internal pressure (BMP280) in hPa
    tempExt: F32       @< External temperature (BME680) in deg C
    pressExt: F32      @< External pressure (BME680) in hPa
    humidity: F32      @< External relative humidity in %
    vBat: F32          @< Battery voltage in Volts
    soc: F32           @< Battery State-of-Charge in %
    $state: U8         @< Mission state byte
  }

  @ Port to pass flight log record to DataLogger
  port LogRecord(
    flightData: FlightLogRecord
  )

}
