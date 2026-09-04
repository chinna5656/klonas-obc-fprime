module Obc {

  @ Parachute deployment safety controller and thermal burn-wire driver
  passive component ParachuteDeployer {

    # ----------------------------------------------------------------------
    # General ports
    # ----------------------------------------------------------------------

    @ Sync input trigger from NavPredictor
    sync input port deployIn: Obc.DeployTrigger

    @ Periodic tick to service burn countdown and arming timeout
    sync input port schedIn: Svc.Sched

    @ Output port to drive thermal burn-wire GPIO MOSFET gate
    output port burnWireGpioOut: Drv.GpioWrite

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

    @ Arm the parachute deployer using the 32-bit security key
    sync command PARACHUTE_ARM(
      key: U32 @< Security passcode (e.g. 0xDEADBEEF)
    )

    @ Disarm the parachute deployer and enforce safety interlock
    sync command PARACHUTE_DISARM()

    @ Command immediate manual deployment using security passcode
    sync command PARACHUTE_DEPLOY(
      key: U32 @< Security passcode
    )

    @ Configure thermal burn pulse duration in milliseconds
    sync command PARACHUTE_SET_BURN_TIME(
      burnDurationMs: U32 @< Burn duration in milliseconds (bounded <= 5000ms)
    )

    # ----------------------------------------------------------------------
    # Telemetry channels
    # ----------------------------------------------------------------------

    @ Current arming state (true = ARMED, false = DISARMED)
    telemetry ArmState: bool

    @ Current state (0=DISARMED, 1=ARMED, 2=BURNING, 3=DEPLOYED, 4=FAULT)
    telemetry DeployerState: U8

    @ Remaining burn-wire actuation time in milliseconds
    telemetry BurnTimeRemainingMs: U32

    @ Remaining armed window countdown in seconds
    telemetry ArmTimeoutRemainingSec: U32

    @ Total number of deployment actuations performed
    telemetry DeploymentCount: U32

    # ----------------------------------------------------------------------
    # Events
    # ----------------------------------------------------------------------

    @ Event emitted when deployer is armed
    event DeployerArmed(timeoutSec: U32) \
      severity activity high \
      id 0 \
      format "Parachute deployer ARMED with {}s safety window"

    @ Event emitted when deployer is disarmed
    event DeployerDisarmed \
      severity activity high \
      id 1 \
      format "Parachute deployer DISARMED"

    @ Event emitted when arming window expires without deployment
    event ArmWindowExpired \
      severity warning high \
      id 2 \
      format "Parachute arming window expired; auto-disarming"

    @ Event emitted when burn-wire gate is activated (GPIO HIGH)
    event BurnWireActivated(durationMs: U32) \
      severity warning high \
      id 3 \
      format "Burn-wire MOSFET gate ACTIVATED for {} ms"

    @ Event emitted when burn-wire gate is deactivated (GPIO LOW)
    event BurnWireDeactivated \
      severity activity high \
      id 4 \
      format "Burn-wire MOSFET gate DEACTIVATED"

    @ Event emitted when deployment sequence completes successfully
    event DeploymentCompleted \
      severity activity high \
      id 5 \
      format "Parachute deployment sequence COMPLETED"

    @ Event emitted when unauthorized or incorrect key is provided
    event UnauthorizedDeployAttempt(providedKey: U32) \
      severity warning high \
      id 6 \
      format "Unauthorized deploy command rejected with key {}"

  }

}
