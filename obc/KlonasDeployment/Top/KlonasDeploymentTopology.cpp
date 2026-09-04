// ======================================================================
// \title  KlonasDeploymentTopology.cpp
// \brief cpp file containing the topology instantiation code
//
// ======================================================================
// Provides access to autocoded functions
#include <obc/KlonasDeployment/Top/KlonasDeploymentTopologyAc.hpp>
// Note: Uncomment when using Svc:TlmPacketizer
//#include <obc/KlonasDeployment/Top/KlonasDeploymentPacketsAc.hpp>

// Necessary project-specified types
#include <Fw/Types/MallocAllocator.hpp>

// KLONAS Phase-1 Custom Component Headers
#include <obc/Components/NavPredictor/NavPredictor.hpp>
#include <obc/Components/ParachuteDeployer/ParachuteDeployer.hpp>
#include <obc/Components/CommsCrypto/CommsCrypto.hpp>
#include <obc/Components/EnvSensors/EnvSensors.hpp>
#include <obc/Components/PowerMonitor/PowerMonitor.hpp>
#include <obc/Components/DataLogger/DataLogger.hpp>
#include <obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h>

// Public functions for use in main program are namespaced with deployment module obc
// This is also the namespace where the topology components are instantiated by FPP.
namespace obc {

// Instantiate a malloc allocator for cmdSeq buffer allocation
Fw::MallocAllocator mallocator;

// KLONAS Phase-1 Rate Group Divisors:
// Base tick: 10 Hz (100 ms interval)
// RateGroup 1: 10 Hz (divisor 1) -> EnvSensors, NavPredictor, TlmSend, ComQueue
// RateGroup 2: 1 Hz (divisor 10) -> CommsCrypto, PowerMonitor, ParachuteDeployer, CmdSeq
// RateGroup 3: 0.25 Hz (divisor 40) -> DataLogger, Health, BufferManager, DpWriter
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{{{1, 0}, {10, 0}, {40, 0}}};

// Rate groups may supply a context token to each of the attached children whose purpose is set by the project. The
// reference topology sets each token to zero as these contexts are unused in this project.
U32 rateGroup1Context[10] = {};
U32 rateGroup2Context[10] = {};
U32 rateGroup3Context[10] = {};

enum TopologyConstants {
    COMM_PRIORITY = 34,
};

/**
 * \brief configure/setup components in project-specific way
 *
 * This is a *helper* function which configures/sets up each component requiring project specific input. This includes
 * allocating resources, passing-in arguments, etc. This function may be inlined into the topology setup function if
 * desired, but is extracted here for clarity.
 */
void configureTopology() {
    // Rate group driver needs a divisor list
    rateGroupDriver.configure(rateGroupDivisorsSet);

    // Rate groups require context arrays.
    rateGroup1.configure(rateGroup1Context, FW_NUM_ARRAY_ELEMENTS(rateGroup1Context));
    rateGroup2.configure(rateGroup2Context, FW_NUM_ARRAY_ELEMENTS(rateGroup2Context));
    rateGroup3.configure(rateGroup3Context, FW_NUM_ARRAY_ELEMENTS(rateGroup3Context));

    // Command sequencer needs to allocate memory to hold contents of command sequences
    cmdSeq.allocateBuffer(0, mallocator, 1024);
}

void setupTopology(const TopologyState& state) {
    // Autocoded initialization. Function provided by autocoder.
    initComponents(state);
    // Autocoded id setup. Function provided by autocoder.
    setBaseIds();
    // Autocoded connection wiring. Function provided by autocoder.
    connectComponents();
    // Autocoded command registration. Function provided by autocoder.
    regCommands();
    // Autocoded configuration. Function provided by autocoder.
    configComponents(state);
    // Initialize USB CDC ACM Virtual COM Port (/dev/ttyACM0)
    comDriver.configure();
    // Initialize STM32 hardware peripherals (GPIO, SPI1 bus, BNO08X reset sequence)
    HalBridge_HardwareInit();

    // Project-specific component configuration. Function provided above. May be inlined, if desired.
    configureTopology();
    // Autocoded parameter loading. Function provided by autocoder.
    loadParameters();
#ifndef __arm__
    // Autocoded task kick-off (active components on host Linux).
    startTasks(state);
#endif
}

void startRateGroups(const Fw::TimeInterval& interval) {
    // The timer component drives the fundamental tick rate of the system.
    // Svc::RateGroupDriver will divide this down to the slower rate groups.
    timer.startTimer(interval);
}

void stopRateGroups() {
    timer.quit();
}

void teardownTopology(const TopologyState& state) {
#ifndef __arm__
    // Autocoded (active component) task clean-up. Functions provided by topology autocoder.
    stopTasks(state);
    freeThreads(state);
#endif

    // Resource deallocation
    cmdSeq.deallocateBuffer(mallocator);

    tearDownComponents(state);
    deinitComponents(state);
}
};  // namespace obc
