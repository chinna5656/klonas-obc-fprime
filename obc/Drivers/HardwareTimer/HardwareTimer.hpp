/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * Hardware Timer Component Header
 * ============================================================================
 */

#ifndef OBC_DRIVERS_HARDWARETIMER_HPP_
#define OBC_DRIVERS_HARDWARETIMER_HPP_

#include "Fw/Time/TimeInterval.hpp"
#include "Os/Mutex.hpp"
#include "Os/RawTime.hpp"
#include "obc/Drivers/HardwareTimer/HardwareTimerComponentAc.hpp"

namespace Obc {

  class HardwareTimer final : public HardwareTimerComponentBase {

    public:

      //! Construct object HardwareTimer
      HardwareTimer(const char* const compName);

      //! Destroy object HardwareTimer
      ~HardwareTimer() override;

      //! Start timer loop
      void startTimer(const Fw::TimeInterval& interval);

      //! Quit timer loop
      void quit();

      //! Direct tick invocation from hardware ISR / SysTick / TIM2
      void tick();

    private:

      Os::Mutex m_mutex;
      volatile bool m_quit;
      Os::RawTime m_rawTime;

  };

}

#endif /* OBC_DRIVERS_HARDWARETIMER_HPP_ */
