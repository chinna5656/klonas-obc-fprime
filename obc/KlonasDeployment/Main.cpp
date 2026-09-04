// ======================================================================
// \title  Main.cpp
// \brief main program for the F' application. Intended for CLI-based systems (Linux, macOS)
//
// ======================================================================
// Used to access topology functions
#include <obc/KlonasDeployment/Top/KlonasDeploymentTopology.hpp>
// OSAL initialization
#include <Os/Os.hpp>
// Low-level hardware bridge (Heartbeat LED & GPIO)
#include <obc/Drivers/HalBridge/stm32f4xx_hal_bridge.h>
// F Prime assert hook
#include <Fw/Types/Assert.hpp>

#if !defined(__arm__) && !defined(STM32F411xE)
// Used for signal handling shutdown (host Linux only)
#include <signal.h>
// Used for command line argument processing (host Linux only)
#include <getopt.h>
// Used for logging to the console
#include <Fw/Logger/Logger.hpp>
#endif

// Used for atoi
#include <cstdlib>

#if defined(__arm__) || defined(STM32F411xE)
// ============================================================
// Bare-Metal FW_ASSERT Hook
// Overrides the default assert handler that calls fputs(stderr)
// which dereferences newlib's _impure_ptr and triggers a BusFault
// on bare-metal where C library stdio is not initialized.
// Instead, we hang in a fast LED blink loop identical to HardFault_Handler.
// ============================================================
namespace {

class BareMetalAssertHook final : public Fw::AssertHook {
public:
    BareMetalAssertHook() = default;

    void reportAssert(
        FILE_NAME_ARG,
        FwSizeType,
        FwSizeType,
        FwAssertArgType,
        FwAssertArgType,
        FwAssertArgType,
        FwAssertArgType,
        FwAssertArgType,
        FwAssertArgType
    ) override {
        // No-op: avoid any stdio/fputs/malloc on bare-metal.
    }

    void doAssert() override {
        // Ensure GPIOC clock (bit 2 of RCC_AHB1ENR) and toggle PC13 LED rapidly.
        *(volatile uint32_t*)0x40023830U |= (1U << 2);
        *(volatile uint32_t*)0x40020800U &= ~(0x03U << 26);
        *(volatile uint32_t*)0x40020800U |=  (0x01U << 26);
        while (true) {
            *(volatile uint32_t*)0x40020814U ^= (1U << 13);
            for (volatile uint32_t i = 0; i < 50000U; i++) {
                __asm__ volatile("nop");
            }
        }
    }
};

} // anonymous namespace
#endif // __arm__

/**
 * \brief execute the program
 *
 * @param argc: argument count supplied to program
 * @param argv: argument values supplied to program
 * @return: 0 on success, something else on failure
 */
int main(int argc, char* argv[]) {

#if defined(__arm__) || defined(STM32F411xE)
    // 1. Immediate hardware indication as soon as static constructors finish.
    BSP_LED_Init();

    // 2. Register bare-metal assert hook BEFORE any F Prime initialization to
    //    prevent fputs(stderr) -> BusFault from any FW_ASSERT failures.
    static BareMetalAssertHook s_assertHook;
    s_assertHook.registerHook();

    // 3. Initialize OS abstractions (stub implementations on bare-metal).
    Os::init();

    // 4. Setup, cycle, and teardown topology.
    obc::TopologyState inputs;
    inputs.hostname = nullptr;
    inputs.port = 0;
    obc::setupTopology(inputs);
    obc::startRateGroups(Fw::TimeInterval(0, 100000)); // 10 Hz base tick (100 ms)
    obc::teardownTopology(inputs);

    // Should never reach here on bare-metal.
    while (true) {
        __asm__ volatile("nop");
    }
    return 0;

#else
    // Host Linux path
    auto signalHandler = [](int) { obc::stopRateGroups(); };

    I32 option = 0;
    CHAR* hostname = nullptr;
    U16 port_number = 0;

    Os::init();

    while ((option = getopt(argc, argv, "hp:a:")) != -1) {
        switch (option) {
            case 'a':
                hostname = optarg;
                break;
            case 'p':
                port_number = static_cast<U16>(atoi(optarg));
                break;
            case 'h':
            case '?':
            default:
                Fw::Logger::log("Usage: ./%s [options]\n-a\thostname/IP address\n-p\tport_number\n", argv[0]);
                return (option == 'h') ? 0 : 1;
        }
    }

    obc::TopologyState inputs;
    inputs.hostname = hostname;
    inputs.port = port_number;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    Fw::Logger::log("Hit Ctrl-C to quit\n");

    obc::setupTopology(inputs);
    obc::startRateGroups(Fw::TimeInterval(0, 100000)); // 10 Hz base tick (100 ms)
    obc::teardownTopology(inputs);
    Fw::Logger::log("Exiting...\n");
    return 0;
#endif
}
