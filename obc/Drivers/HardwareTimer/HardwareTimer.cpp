/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * Hardware Timer Component Implementation
 * ============================================================================
 */

#include "obc/Drivers/HardwareTimer/HardwareTimer.hpp"
#include "obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h"
#include "obc/Drivers/UsbCdcDriver/usbd/usb_clock.h"
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
    BSP_LED_Init();

    volatile uint32_t* const DEMCR_REG      = (volatile uint32_t*)0xE000EDFCU;
    volatile uint32_t* const DWT_CTRL_REG   = (volatile uint32_t*)0xE0001000U;
    volatile uint32_t* const DWT_CYCCNT_REG = (volatile uint32_t*)0xE0001004U;

    // Enable DWT Cycle Counter
    *DEMCR_REG |= (1U << 24);    // CoreDebug_DEMCR_TRCENA_Msk
    *DWT_CTRL_REG |= (1U << 0);   // DWT_CTRL_CYCCNTENA_Msk

    const uint32_t cyclesPerTick = 1600000U; // 100 ms at 16 MHz HSI
    uint32_t lastCycle = *DWT_CYCCNT_REG;
    uint32_t tickCount = 0;

    while (!this->m_quit) {
      while ((*DWT_CYCCNT_REG - lastCycle) < cyclesPerTick) {
        __asm__ volatile("nop");
      }
      lastCycle += cyclesPerTick;

      this->tick();
      tickCount++;

      if ((tickCount % 5) == 0) {
        BSP_LED_Toggle();
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
