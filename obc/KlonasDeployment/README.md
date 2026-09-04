# KlonasDeployment

F Prime deployment target for the **KLONAS Phase-1** CubeSat mission.

## Supported Targets

| Target | Platform | Description |
| :--- | :--- | :--- |
| **Native (Linux)** | `x86_64` | Software-in-the-loop simulation with TCP GDS |
| **STM32F411** | `arm-none-eabi` | Bare-metal cross-compilation for Black Pill |

## Build & Run — Native (Linux SIL)

```bash
cd ~/F-prime-obc
source fprime-venv/bin/activate

fprime-util generate
fprime-util build

# Launch GDS + application
fprime-gds

# Or run separately:
fprime-gds --no-app
./build-artifacts/Linux/obc_KlonasDeployment/bin/obc_KlonasDeployment -a 127.0.0.1 -p 50000
```

## Build & Run — STM32F411 (Bare-Metal)

```bash
cd ~/F-prime-obc
source fprime-venv/bin/activate

# Generate & build
fprime-util generate stm32f411
fprime-util build stm32f411

# Extract binary
arm-none-eabi-objcopy -O binary \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment.bin

# Flash via ST-Link V2
st-flash --reset write \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment.bin 0x08000000

# Connect GDS via USB CDC (/dev/ttyACM0)
fprime-gds -n --comm-adapter uart --uart-device /dev/ttyACM0 --uart-baud 115200 \
  --dictionary build-artifacts/stm32f411/obc_KlonasDeployment/dict/KlonasDeploymentTopologyDictionary.json
```

## Unit Tests

```bash
fprime-util generate --ut
fprime-util build --ut
ctest --test-dir build-fprime-automatic-native-ut --output-on-failure
```

## Topology Architecture

This deployment uses F Prime **core subtopologies** for a modular, reusable architecture:

- **CdhCore**: Command & Data Handling
  - Command dispatching and event management
  - Event logging and telemetry collection
  - Health monitoring system
  - Fatal error handling

- **ComCcsds**: CCSDS Communication Subsystem
  - CCSDS protocol implementation
  - Uplink/downlink data handling
  - Frame processing and routing

- **FileHandling**: File Transfer & Command Sequencing
  - File upload and download services
  - Parameter database management
  - File system operations

- **DataProducts**: Data Product Management
  - Data product cataloging
  - Storage and retrieval capabilities
  - Product metadata management

## Custom KLONAS Components

| Instance | Component | Priority | Rate |
| :--- | :--- | :--- | :--- |
| `navPredictor` | `Obc.NavPredictor` | 50 | 10 Hz |
| `parachuteDeployer` | `Obc.ParachuteDeployer` | 50 | 1 Hz |
| `commsCrypto` | `Obc.CommsCrypto` | 60 | 1 Hz |
| `envSensors` | `Obc.EnvSensors` | 37 | 10 Hz |
| `powerMonitor` | `Obc.PowerMonitor` | 36 | 1 Hz |
| `dataLogger` | `Obc.DataLogger` | 30 | 0.25 Hz |

## Bare-Metal Configuration

The STM32 target uses the following OSAL implementations:

```
Os_Task_Stub            # No OS threading (cooperative)
Os_Mutex_Stub           # No OS mutexes
Os_Generic_PriorityQueue # Static priority queue for message dispatch
Os_Console_Stub         # No console output
Os_File_Stub            # No filesystem
Os_RawTime_Stub         # SysTick-based timing
Os_Cpu_Stub             # No CPU stats
Os_Memory_Stub          # No memory stats
```

Compiler flags include `-fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-unwind-tables -fno-use-cxa-atexit` for minimal bare-metal overhead.