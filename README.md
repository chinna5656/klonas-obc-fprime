# KLONAS Phase-1 CubeSat Flight Software

> NASA F Prime (F') flight software for the KLONAS Phase-1 CubeSat mission, targeting the **STM32F411CEU6 (WeAct Black Pill)** ARM Cortex-M4F microcontroller.

---

## Mission Overview

KLONAS Phase-1 is a technology demonstration flight validating an autonomous high-altitude descent prediction and parachute recovery system. The avionics stack executes real-time trajectory estimation, encrypted telemetry, multi-sensor fusion, and thermal burn-wire parachute actuation within strict SWaP constraints.

## Target Hardware

| Parameter | Value |
| :--- | :--- |
| **MCU** | STM32F411CEU6 (ARM Cortex-M4F) |
| **Clock** | 96 MHz (HSE 25 MHz, PLL M=25 N=192 P=2 Q=4) |
| **Flash** | 512 KB (`0x08000000`) |
| **SRAM** | 128 KB (`0x20000000`) |
| **GDS Interface** | USB CDC ACM (`/dev/ttyACM0`) via onboard USB-C |
| **Debug/Flash** | SWD via ST-Link V2 |
| **Burn Wire** | PB9 (GPIO Out, N-MOSFET Gate) |
| **Crash Switch** | PB8 (GPIO In, Pull-down) |
| **Heartbeat LED** | PC13 (Active Low) |

## Flight Software Components

| Component | Role | Rate |
| :--- | :--- | :--- |
| **NavPredictor** | GPS NMEA parsing, descent rate filter, apogee detection, deploy trigger | 10 Hz |
| **ParachuteDeployer** | Dual-key safety interlock, thermal burn-wire actuation (PB9) | 1 Hz |
| **CommsCrypto** | AES-128-CBC encryption, CCSDS framing, CRC-16-CCITT | 1 Hz |
| **EnvSensors** | BNO08X IMU, BMP280 barometer, BME680 gas sensor (SPI1) | 10 Hz |
| **PowerMonitor** | Battery SoC estimation (ADC1), NE555P WDT strobe (PB10) | 1 Hz |
| **DataLogger** | Ping-pong 512B double-buffer, MicroSD writes (SPI2) | 0.25 Hz |

## Quick Start

### Prerequisites

```bash
# Python virtual environment with F Prime tools
source fprime-venv/bin/activate

# ARM cross-compiler (for STM32 target)
arm-none-eabi-gcc --version   # requires 12.2.rel1 or compatible
```

### Native Build (Linux SIL Simulation)

```bash
fprime-util generate
fprime-util build
fprime-gds
```

### STM32 Cross-Compilation

```bash
fprime-util generate stm32f411
fprime-util build stm32f411
```

### Extract Binary & Check Memory

```bash
arm-none-eabi-objcopy -O binary \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment.bin

arm-none-eabi-size build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment
```

### Flash to Hardware (ST-Link V2)

```bash
# One-step script:
./scripts/flash_stm32.sh

# Or manually:
st-flash --reset write \
  build-artifacts/stm32f411/obc_KlonasDeployment/bin/obc_KlonasDeployment.bin 0x08000000
```

### Connect F Prime GDS via USB CDC

```bash
fprime-gds -n --comm-adapter uart --uart-device /dev/ttyACM0 --uart-baud 115200 \
  --dictionary build-artifacts/stm32f411/obc_KlonasDeployment/dict/KlonasDeploymentTopologyDictionary.json
```

## Unit Tests

```bash
fprime-util generate --ut
fprime-util build --ut
ctest --test-dir build-fprime-automatic-native-ut --output-on-failure
```

**Current Status:** 4/4 test suites, 44/44 tests passing (100%).

| Test Suite | Tests | Status |
| :--- | :--- | :--- |
| NavPredictor | 11 | ✅ PASS |
| ParachuteDeployer | 11 | ✅ PASS |
| CommsCrypto | 11 | ✅ PASS |
| EnvSensors | 11 | ✅ PASS |

## Memory Budget (Latest Build)

| Region | Used | Limit | Usage | Margin |
| :--- | :--- | :--- | :--- | :--- |
| **Flash ROM** | **398,508 B** (389.2 KB) | 524,288 B (512 KB) | **76.0%** | **122.8 KB free** |
| **BSS + Data** | **90,020 B** (87.9 KB) | 131,072 B (128 KB) | **68.7%** | **41.1 KB free** |
| **Total SRAM (Allocated)** | **102,312 B** (99.9 KB) | 131,072 B (128 KB) | **78.1%** | **28.1 KB free** |

## Project Structure

```
F-prime-obc/
├── cmake/
│   ├── toolchain/
│   │   ├── stm32f411.cmake          # Cross-compilation toolchain
│   │   └── stm32f411ceu6.ld         # Linker script (Flash/RAM layout)
│   └── platform/
│       └── stm32f411/Platform/      # OSAL stubs, startup, vector table
├── docs/
│   ├── klonas_phase1_sdd.md         # Software Design Document (SDD)
│   └── walkthrough.md               # Implementation walkthrough
├── obc/
│   ├── Components/
│   │   ├── NavPredictor/            # GPS, descent rate, deploy logic
│   │   ├── ParachuteDeployer/       # Safety interlock, burn wire
│   │   ├── CommsCrypto/             # AES-128-CBC, packet framing
│   │   ├── EnvSensors/              # BNO08X, BMP280, BME680
│   │   ├── PowerMonitor/            # Battery SoC, WDT strobe
│   │   └── DataLogger/              # Ring-buffer, MicroSD logging
│   ├── Drivers/
│   │   ├── HalBridge/               # STM32 HAL abstraction + mock
│   │   ├── UsbCdcDriver/            # USB CDC ACM for GDS comms
│   │   ├── HardwareTimer/           # SysTick-based rate group driver
│   │   ├── SpiDriver/               # SPI1/SPI2 transaction driver
│   │   ├── UartDriver/              # USART1/USART2 driver
│   │   ├── AdcDriver/               # ADC1 battery voltage driver
│   │   └── GpioDriver/              # GPIO read/write driver
│   ├── KlonasDeployment/
│   │   ├── Main.cpp                 # Entry point (bare-metal + native)
│   │   └── Top/                     # Topology (instances.fpp, topology.fpp)
│   └── Ports/                       # Custom F Prime port definitions
├── scripts/
│   └── flash_stm32.sh               # One-step flash utility
├── lib/fprime/                       # F Prime framework (submodule)
└── settings.ini                      # F Prime project settings
```

## Documentation

| Document | Description |
| :--- | :--- |
| [Software Design Document (SDD)](docs/klonas_phase1_sdd.md) | Full system architecture, component breakdown, safety interlocks, recovery state machine |
| [Implementation Walkthrough](docs/walkthrough.md) | Detailed engineering log of all implementations, test results, and memory audits |

## Framework

Built on [NASA F Prime (F')](https://fprime.jpl.nasa.gov) — a component-driven framework for rapid development of spaceflight and embedded software applications.
