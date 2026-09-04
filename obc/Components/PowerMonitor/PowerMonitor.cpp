/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * PowerMonitor Component Implementation
 * ============================================================================
 */

#include "obc/Components/PowerMonitor/PowerMonitor.hpp"
#include <Fw/Types/Assert.hpp>
#include <cmath>

namespace Obc {

  PowerMonitor::PowerMonitor(const char* const compName) :
    PowerMonitorComponentBase(compName),
    m_dividerRatio(2.0f), // 100k / 100k voltage divider on PA0
    m_lowPowerThreshV(DEFAULT_LOW_POWER_THRESH_V),
    m_recoveryThreshV(DEFAULT_RECOVERY_THRESH_V),
    m_vBat(4.18f),
    m_adcRaw(2600),
    m_soc(98.0f),
    m_estRuntimeHours(21.3f),
    m_estRuntimeMinutes(1278),
    m_lowPowerMode(false),
    m_lowPowerConfirmCount(0),
    m_recoveryConfirmCount(0),
    m_wdtPulseCount(0)
  {
  }

  void PowerMonitor::init(
      FwSizeType queueDepth,
      FwEnumStoreType instance
  ) {
    PowerMonitorComponentBase::init(queueDepth, instance);
  }

  void PowerMonitor::schedIn_handler(
      FwIndexType portNum,
      U32 context
  ) {
    FW_ASSERT(portNum == 0);
    (void)context;

    // 1. Kick external hardware watchdog (PB10)
    this->kickWatchdog();

    // 2. Sample Battery Voltage via ADC1 Channel 0 (PA0)
    if (this->isConnected_adcIn_OutputPort(0)) {
      U16 rawVal = 0;
      Fw::Success status = this->adcIn_out(0, 0, rawVal);
      if (status == Fw::Success::SUCCESS) {
        m_adcRaw = rawVal;

        // V_sense = (ADC_RAW / 4095.0) * 3.30 V
        F32 vSense = (static_cast<F32>(m_adcRaw) / ADC_MAX_COUNTS) * ADC_VREF;

        // V_bat = V_sense * DividerRatio
        m_vBat = vSense * m_dividerRatio;
      }
    }

    // 3. Compute State of Charge (%)
    m_soc = this->calculateSoc(m_vBat);

    // 4. Estimate Remaining Operational Runtime
    this->estimateRuntime(m_soc, m_lowPowerMode, m_estRuntimeHours, m_estRuntimeMinutes);

    // 5. Evaluate Low Power Mode State Transitions
    if (m_vBat <= BATTERY_CRITICAL_CUTOFF_V) {
      this->log_FATAL_BatteryCriticalShutdown(m_vBat);
    } else if (m_vBat <= m_lowPowerThreshV) {
      m_recoveryConfirmCount = 0;
      m_lowPowerConfirmCount++;
      if (m_lowPowerConfirmCount >= 5 && !m_lowPowerMode) {
        m_lowPowerMode = true;
        this->log_WARNING_HI_LowPowerModeEntered(m_vBat);
      }
      if (m_soc < 15.0f) {
        this->log_WARNING_HI_BatteryLowWarning(m_vBat, m_soc);
      }
    } else if (m_vBat >= m_recoveryThreshV) {
      m_lowPowerConfirmCount = 0;
      m_recoveryConfirmCount++;
      if (m_recoveryConfirmCount >= 5 && m_lowPowerMode) {
        m_lowPowerMode = false;
        this->log_ACTIVITY_HI_NormalPowerModeRestored(m_vBat);
      }
    } else {
      m_lowPowerConfirmCount = 0;
      m_recoveryConfirmCount = 0;
    }

    // 6. Write Telemetry Channels
    this->tlmWrite_BatteryVoltage(m_vBat);
    this->tlmWrite_AdcRaw(m_adcRaw);
    this->tlmWrite_StateOfCharge(m_soc);
    this->tlmWrite_EstimatedRuntimeHours(m_estRuntimeHours);
    this->tlmWrite_EstimatedRuntimeMinutes(m_estRuntimeMinutes);
    this->tlmWrite_LowPowerModeActive(m_lowPowerMode);
    this->tlmWrite_WatchdogPulseCount(m_wdtPulseCount);
  }

  void PowerMonitor::PWR_SET_LOW_POWER_THRESH_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      F32 threshVolt
  ) {
    if (threshVolt >= 3.10f && threshVolt <= 3.80f) {
      m_lowPowerThreshV = threshVolt;
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
    }
  }

  void PowerMonitor::PWR_FORCE_LOW_POWER_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      Fw::Enabled enable
  ) {
    m_lowPowerMode = (enable == Fw::Enabled::ENABLED);
    if (m_lowPowerMode) {
      this->log_WARNING_HI_LowPowerModeEntered(m_vBat);
    } else {
      this->log_ACTIVITY_HI_NormalPowerModeRestored(m_vBat);
    }
    this->tlmWrite_LowPowerModeActive(m_lowPowerMode);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void PowerMonitor::PWR_CALIBRATE_DIVIDER_cmdHandler(
      FwOpcodeType opCode,
      U32 cmdSeq,
      F32 dividerRatio
  ) {
    if (dividerRatio >= 1.0f && dividerRatio <= 5.0f) {
      m_dividerRatio = dividerRatio;
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
    } else {
      this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::VALIDATION_ERROR);
    }
  }

  F32 PowerMonitor::calculateSoc(F32 voltage) const {
    // Piecewise linear Li-Ion discharge curve model
    if (voltage >= 4.20f) {
      return 100.0f;
    } else if (voltage >= 4.00f) {
      return 80.0f + (20.0f * (voltage - 4.00f) / 0.20f);
    } else if (voltage >= 3.80f) {
      return 55.0f + (25.0f * (voltage - 3.80f) / 0.20f);
    } else if (voltage >= 3.65f) {
      return 25.0f + (30.0f * (voltage - 3.65f) / 0.15f);
    } else if (voltage >= 3.40f) {
      return 5.0f + (20.0f * (voltage - 3.40f) / 0.25f);
    } else if (voltage >= 3.00f) {
      return 5.0f * (voltage - 3.00f) / 0.40f;
    } else {
      return 0.0f;
    }
  }

  void PowerMonitor::estimateRuntime(
      F32 soc,
      bool isLowPower,
      F32& hours,
      U32& minutes
  ) const {
    // Available capacity = Nominal * (SoC / 100)
    F32 availCapMah = NOMINAL_CAPACITY_MAH * (soc / 100.0f);

    // Current draw depending on operating mode
    F32 loadCurrent = isLowPower ? LOAD_CURRENT_LOW_POWER_MA : LOAD_CURRENT_NORMAL_MA;

    // Remaining runtime
    hours = availCapMah / loadCurrent;
    minutes = static_cast<U32>(roundf(hours * 60.0f));
  }

  void PowerMonitor::kickWatchdog() {
    if (this->isConnected_wdtGpioOut_OutputPort(0)) {
      // Pulse PB10 HIGH to strobe NE555P trigger pin
      this->wdtGpioOut_out(0, Fw::Logic::HIGH);
      this->wdtGpioOut_out(0, Fw::Logic::LOW);
    }
    m_wdtPulseCount++;
    this->log_DIAGNOSTIC_WatchdogKicked(m_wdtPulseCount);
  }

}
