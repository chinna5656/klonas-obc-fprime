/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * Hardware Timer Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/HardwareTimer/HardwareTimer.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"
#include <Fw/Types/Assert.hpp>

#if defined(TGT_OS_TYPE_LINUX)
#include <unistd.h>
#endif

namespace Obc {

  HardwareTimer::HardwareTimer(const char* const compName) :
    HardwareTimerComponentBase(compName),
    m_quit(false)
  {
  }

  HardwareTimer::~HardwareTimer() {
  }

  void HardwareTimer::quit() {
    this->m_mutex.lock();
    this->m_quit = true;
    this->m_mutex.unLock();
  }

  void HardwareTimer::tick() {
    this->m_rawTime.now();
    if (this->isConnected_CycleOut_OutputPort(0)) {
      this->CycleOut_out(0, this->m_rawTime);
    }
  }

  void HardwareTimer::startTimer(const Fw::TimeInterval& interval) {
    this->m_mutex.lock();
    this->m_quit = false;
    this->m_mutex.unLock();

#if defined(TGT_OS_TYPE_LINUX)
    U32 usec = interval.getUSeconds() + interval.getSeconds() * 1000000;
    while (!this->m_quit) {
      this->tick();
      usleep(usec);
    }
#else
    #if defined(__arm__) || defined(STM32F411xE)
    // 1. Initialize PC13 onboard heartbeat LED (active low)
    BSP_LED_Init();

    // 2. Configure SysTick for 1ms hardware intervals at 96 MHz SYSCLK
    #define SYSTICK_CTRL_REG  (*(volatile uint32_t*)0xE000E010U)
    #define SYSTICK_LOAD_REG  (*(volatile uint32_t*)0xE000E014U)
    #define SYSTICK_VAL_REG   (*(volatile uint32_t*)0xE000E018U)

    SYSTICK_LOAD_REG = (96000000U / 1000U) - 1U; // 96000 cycles per ms
    SYSTICK_VAL_REG  = 0;
    SYSTICK_CTRL_REG = 0x05U; // Processor clock, enable counter, no interrupt

    U32 interval_ms = interval.getSeconds() * 1000 + interval.getUSeconds() / 1000;
    if (interval_ms == 0) {
      interval_ms = 100; // default 100ms (10 Hz base tick)
    }

    U32 tickCount = 0;
    // Toggle LED every 500ms (1 Hz full blink cycle: 500ms ON, 500ms OFF)
    U32 togglePeriodTicks = 500 / interval_ms;
    if (togglePeriodTicks == 0) {
      togglePeriodTicks = 1;
    }

    while (!this->m_quit) {
      this->tick();
      tickCount++;

      // Heartbeat blink on PC13
      if ((tickCount % togglePeriodTicks) == 0) {
        BSP_LED_Toggle();
      }

      // Hardware millisecond delay via SysTick COUNTFLAG (bit 16)
      for (uint32_t m = 0; m < interval_ms; m++) {
        while ((SYSTICK_CTRL_REG & (1U << 16)) == 0) {
          __asm__ volatile("nop");
        }
      }
    }
    #else
    while (!this->m_quit) {
      this->tick();
    }
    #endif
#endif
  }

}
