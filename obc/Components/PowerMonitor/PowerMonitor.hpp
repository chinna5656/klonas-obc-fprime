/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * PowerMonitor Component Header
 * ============================================================================
 */

#ifndef OBC_POWERMONITOR_HPP_
#define OBC_POWERMONITOR_HPP_

#include "obc/Components/PowerMonitor/PowerMonitorComponentAc.hpp"

namespace Obc {

  class PowerMonitor : public PowerMonitorComponentBase {

    public:

      //! ADC full scale reference voltage
      static constexpr F32 ADC_VREF = 3.30f;

      //! ADC 12-bit resolution count
      static constexpr F32 ADC_MAX_COUNTS = 4095.0f;

      //! Nominal battery cell capacity in milliampere-hours (1S Li-Ion)
      static constexpr F32 NOMINAL_CAPACITY_MAH = 2500.0f;

      //! Typical active flight software load current in mA
      static constexpr F32 LOAD_CURRENT_NORMAL_MA = 115.0f;

      //! Low-power standby load current in mA
      static constexpr F32 LOAD_CURRENT_LOW_POWER_MA = 28.0f;

      //! Critical voltage cutoff for battery safety (V)
      static constexpr F32 BATTERY_CRITICAL_CUTOFF_V = 3.00f;

      //! Default low-power entry threshold (V)
      static constexpr F32 DEFAULT_LOW_POWER_THRESH_V = 3.45f;

      //! Default normal-power recovery threshold (V)
      static constexpr F32 DEFAULT_RECOVERY_THRESH_V = 3.65f;

      //! Construct PowerMonitor instance
      PowerMonitor(const char* const compName);

      //! Destructor
      ~PowerMonitor() override = default;

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

      void PWR_SET_LOW_POWER_THRESH_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          F32 threshVolt
      ) override;

      void PWR_FORCE_LOW_POWER_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          Fw::Enabled enable
      ) override;

      void PWR_CALIBRATE_DIVIDER_cmdHandler(
          FwOpcodeType opCode,
          U32 cmdSeq,
          F32 dividerRatio
      ) override;

      // ----------------------------------------------------------------------
      // Power calculations & Watchdog routines
      // ----------------------------------------------------------------------

      //! Compute Li-Ion State of Charge (%) using piecewise linear OCV curve
      F32 calculateSoc(F32 voltage) const;

      //! Estimate remaining operational runtime in hours and minutes
      void estimateRuntime(F32 soc, bool isLowPower, F32& hours, U32& minutes) const;

      //! Pulse PB10 output to stroke external NE555P hardware watchdog
      void kickWatchdog();

      // ----------------------------------------------------------------------
      // Member variables (zero dynamic allocation)
      // ----------------------------------------------------------------------

      F32 m_dividerRatio;
      F32 m_lowPowerThreshV;
      F32 m_recoveryThreshV;

      F32 m_vBat;
      U16 m_adcRaw;
      F32 m_soc;
      F32 m_estRuntimeHours;
      U32 m_estRuntimeMinutes;

      bool m_lowPowerMode;
      U8 m_lowPowerConfirmCount;
      U8 m_recoveryConfirmCount;

      U32 m_wdtPulseCount;

  };

}

#endif /* OBC_POWERMONITOR_HPP_ */
