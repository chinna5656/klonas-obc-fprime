module Obc {

  @ Power monitor, battery SoC estimator, and NE555P WDT strobing component
  passive component PowerMonitor {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Periodic schedule tick for ADC sampling and WDT pulse strobing
    sync input port schedIn: Svc.Sched

    @ Output port to read ADC channel 0 (PA0 battery voltage divider)
    output port adcIn: Obc.AdcSample

    @ Output port to drive NE555P external hardware watchdog pulse (PB10)
    output port wdtGpioOut: Drv.GpioWrite

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

    @ Configure low power voltage threshold (Volts)
    sync command PWR_SET_LOW_POWER_THRESH(
      threshVolt: F32 @< Battery voltage threshold to enter low power mode
    )

    @ Force low power mode override
    sync command PWR_FORCE_LOW_POWER(
      enable: Fw.Enabled @< Enable or disable low power mode
    )

    @ Calibrate voltage divider ratio (default 2.0 for 100k/100k)
    sync command PWR_CALIBRATE_DIVIDER(
      dividerRatio: F32 @< Resistor divider attenuation factor
    )

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ Battery pack voltage in Volts
    telemetry BatteryVoltage: F32

    @ Raw 12-bit ADC reading (0 - 4095)
    telemetry AdcRaw: U16

    @ Battery State-of-Charge in percent (0.0% to 100.0%)
    telemetry StateOfCharge: F32

    @ Estimated remaining operational runtime in hours
    telemetry EstimatedRuntimeHours: F32

    @ Estimated remaining operational runtime in minutes
    telemetry EstimatedRuntimeMinutes: U32

    @ Low power mode status flag
    telemetry LowPowerModeActive: bool

    @ Total NE555P hardware watchdog pulse triggers
    telemetry WatchdogPulseCount: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Warning event when battery voltage drops below warning threshold
    event BatteryLowWarning(voltage: F32, soc: F32) \
      severity warning high \
      id 0 \
      format "Battery LOW: {} V ({}% SoC)"

    @ Critical event when battery reaches emergency cutoff
    event BatteryCriticalShutdown(voltage: F32) \
      severity fatal \
      id 1 \
      format "CRITICAL BATTERY: {} V! Initiating low-power freeze"

    @ Event emitted when entering Low Power Mode
    event LowPowerModeEntered(voltage: F32) \
      severity warning high \
      id 2 \
      format "Transitioning to LOW POWER MODE at {} V"

    @ Event emitted when restoring Normal Power Mode
    event NormalPowerModeRestored(voltage: F32) \
      severity activity high \
      id 3 \
      format "Restoring NORMAL POWER MODE at {} V"

    @ Diagnostic event when external watchdog is stroked
    event WatchdogKicked(count: U32) \
      severity diagnostic \
      id 4 \
      format "NE555P Watchdog pulsed on PB10 (count={})"

  }

}
