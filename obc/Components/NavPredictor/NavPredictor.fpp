module Obc {

  @ Navigation predictor, descent estimator, and parachute trigger component
  passive component NavPredictor {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Input port for incoming GPS NMEA stream from UART driver
    sync input port gpsDataIn: Drv.ByteStreamData

    @ Periodic schedule port for state estimation and criteria checks
    sync input port schedIn: Svc.Sched

    @ Input port to sample crash impact GPIO (PB8)
    output port crashGpioIn: Drv.GpioRead

    @ Output port to trigger parachute deployment
    output port deployOut: Obc.DeployTrigger

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

    @ Command to set the automatic parachute deployment altitude threshold (meters MSL)
    sync command NAV_SET_DEPLOY_ALT(
      altThresholdM: F32 @< Parachute deploy threshold altitude in meters
    )

    @ Command to arm or disarm automatic parachute deployment
    sync command NAV_ARM_PARACHUTE(
      arm: Fw.Enabled @< Enable or disable parachute arming
    )

    @ Emergency command to force immediate parachute deployment trigger
    sync command NAV_FORCE_DEPLOY()

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ Current GPS Latitude in decimal degrees
    telemetry Latitude: F64

    @ Current GPS Longitude in decimal degrees
    telemetry Longitude: F64

    @ Current GPS Altitude in meters MSL
    telemetry Altitude: F32

    @ Filtered descent rate in m/s (positive downwards, Vz = -dh/dt)
    telemetry DescentRate: F32

    @ Current horizontal ground speed in m/s
    telemetry GroundSpeed: F32

    @ Current ground track angle in degrees (0 = North, 90 = East)
    telemetry TrackAngle: F32

    @ Real-time computed landing footprint latitude
    telemetry PredictedLandingLat: F64

    @ Real-time computed landing footprint longitude
    telemetry PredictedLandingLon: F64

    @ Estimated time to impact / touchdown in seconds
    telemetry TimeToImpact: F32

    @ GPS Fix status (0 = Invalid, 1 = GPS SPS, 2 = DGPS)
    telemetry GpsFix: U8

    @ Number of GPS satellites in use
    telemetry SatsInUse: U8

    @ Impact / crash detection trigger status
    telemetry CrashDetected: bool

    @ Apogee detected state
    telemetry ApogeeDetected: bool

    @ Parachute armed state
    telemetry ParachuteArmed: bool

    @ Parachute triggered state
    telemetry ParachuteTriggered: bool

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when GPS 3D fix is acquired
    event GpsLockAcquired(sats: U8) \
      severity activity high \
      id 0 \
      format "GPS 3D lock acquired with {} satellites"

    @ Event emitted when GPS lock is lost
    event GpsLockLost \
      severity warning high \
      id 1 \
      format "GPS lock lost - fix invalid"

    @ Event emitted when vehicle passes apogee and begins descent
    event ApogeeDetected(apogeeAltM: F32) \
      severity activity high \
      id 2 \
      format "Vehicle apogee detected at {} meters MSL"

    @ Event emitted when automatic parachute deployment criteria are satisfied
    event ParachuteCriteriaMet(altM: F32, descentRateMs: F32) \
      severity activity high \
      id 3 \
      format "Parachute deploy criteria satisfied at alt {} m, descent rate {} m/s"

    @ Event emitted when parachute deployment is triggered
    event ParachuteTriggered(force: bool) \
      severity activity high \
      id 4 \
      format "Parachute deployment triggered (force={})"

    @ Event emitted when impact crash sensor PB8 is triggered
    event CrashImpactDetected \
      severity warning high \
      id 5 \
      format "CRASH IMPACT DETECTED on PB8"

  }

}
