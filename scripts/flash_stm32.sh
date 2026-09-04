#!/usr/bin/env bash
# ============================================================================
# KLONAS Phase-1 CubeSat Flight Software
# STM32F411CEU6 Flashing & Verification Script (ST-Link V2)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET_DIR="${WORKSPACE_ROOT}/build-artifacts/stm32f411/obc_KlonasDeployment/bin"
ELF_IMAGE="${TARGET_DIR}/obc_KlonasDeployment"
BIN_IMAGE="${TARGET_DIR}/obc_KlonasDeployment.bin"
HEX_IMAGE="${TARGET_DIR}/obc_KlonasDeployment.hex"
FLASH_BASE_ADDR="0x08000000"

echo "===================================================================="
echo " KLONAS Phase-1: STM32F411 Flight Software Flashing Utility"
echo "===================================================================="

# Check if build artifact exists
if [ ! -f "${ELF_IMAGE}" ]; then
    echo "[ERROR] Target ELF image not found at: ${ELF_IMAGE}"
    echo "[HINT] Run 'fprime-util build stm32f411' first."
    exit 1
fi

# Ensure raw binary and hex files are up-to-date
echo "[1/4] Generating raw .bin and .hex images from ELF..."
arm-none-eabi-objcopy -O binary "${ELF_IMAGE}" "${BIN_IMAGE}"
arm-none-eabi-objcopy -O ihex "${ELF_IMAGE}" "${HEX_IMAGE}"

# Print binary size summary
echo "[2/4] Inspecting memory footprint:"
arm-none-eabi-size --format=berkeley "${ELF_IMAGE}"

echo "[3/4] Checking programmer connection..."
PROGRAMMER_TOOL=""

if command -v st-flash &> /dev/null; then
    PROGRAMMER_TOOL="st-flash"
elif command -v openocd &> /dev/null; then
    PROGRAMMER_TOOL="openocd"
else
    echo "[ERROR] Neither 'st-flash' nor 'openocd' found on host."
    echo "[INSTALL] On Ubuntu/Debian, install with: sudo apt install stlink-tools openocd"
    exit 1
fi

echo "[4/4] Flashing STM32F411CEU6 using ${PROGRAMMER_TOOL}..."

if [ "${PROGRAMMER_TOOL}" = "st-flash" ]; then
    echo "[EXEC] st-flash --reset write ${BIN_IMAGE} ${FLASH_BASE_ADDR}"
    st-flash --reset write "${BIN_IMAGE}" "${FLASH_BASE_ADDR}"
else
    echo "[EXEC] openocd (stlink.cfg -> target/stm32f4x.cfg)"
    openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "init" \
        -c "reset init" \
        -c "flash write_image erase ${HEX_IMAGE}" \
        -c "reset run" \
        -c "shutdown"
fi

echo "===================================================================="
echo " FLASH SUCCESSFUL: Target MCU is running KLONAS Phase-1 Flight SW"
echo " Heartbeat: PC13 LED ON (solid) -> 1 Hz blink when rate groups start"
echo " GDS Interface: USB CDC ACM (/dev/ttyACM0) via onboard USB-C"
echo " Connect GDS: fprime-gds -n --comm-adapter uart --uart-device /dev/ttyACM0 --uart-baud 115200"
echo "===================================================================="
