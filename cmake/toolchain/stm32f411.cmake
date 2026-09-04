####
# ============================================================================
# KLONAS Phase-1 CubeSat Flight Software
# CMake Cross-Compilation Toolchain for STM32F411CEU6 (Black Pill)
# Target Core: ARM Cortex-M4F with Hardware FPU (Single Precision)
# ============================================================================
####

set(CMAKE_SYSTEM_NAME Generic CACHE STRING "Target System" FORCE)
set(CMAKE_SYSTEM_PROCESSOR arm CACHE STRING "Target Processor" FORCE)
set(FPRIME_PLATFORM "stm32f411" CACHE STRING "F Prime Platform" FORCE)

# ----------------------------------------------------------------------------
# Cross-Compiler Toolchain Binaries
# ----------------------------------------------------------------------------
set(TOOLCHAIN_PREFIX arm-none-eabi-)

find_program(CMAKE_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}g++)
find_program(CMAKE_ASM_COMPILER NAMES ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy)
find_program(CMAKE_OBJDUMP NAMES ${TOOLCHAIN_PREFIX}objdump)
find_program(CMAKE_SIZE NAMES ${TOOLCHAIN_PREFIX}size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ----------------------------------------------------------------------------
# Architecture & Hardware FPU Flags (Cortex-M4F @ 96/100 MHz)
# ----------------------------------------------------------------------------
set(ARCH_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# ----------------------------------------------------------------------------
# Target Preprocessor Definitions
# ----------------------------------------------------------------------------
add_definitions(
    -DSTM32F411xE
    -DUSE_HAL_DRIVER
    -DHSE_VALUE=25000000
    -DARM_MATH_CM4
    -D__STDC_FORMAT_MACROS
    -DFW_OBJECT_NAMES=0
)

# ----------------------------------------------------------------------------
# Compiler Flags (-Os size optimization, zero exceptions, zero RTTI)
# ----------------------------------------------------------------------------
set(COMMON_FLAGS "${ARCH_FLAGS} -Os -g3 -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_C_FLAGS   "${COMMON_FLAGS} -std=c11" CACHE STRING "C compiler flags" FORCE)
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-unwind-tables -fno-use-cxa-atexit" CACHE STRING "C++ compiler flags" FORCE)
set(CMAKE_ASM_FLAGS "${ARCH_FLAGS} -x assembler-with-cpp" CACHE STRING "ASM compiler flags" FORCE)

# ----------------------------------------------------------------------------
# Linker Configuration & Memory Map Script
# ----------------------------------------------------------------------------
set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/stm32f411ceu6.ld")

set(CMAKE_EXE_LINKER_FLAGS
    "${ARCH_FLAGS} -T\"${LINKER_SCRIPT}\" -Wl,--gc-sections -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref --specs=nano.specs --specs=nosys.specs -u _printf_float"
    CACHE STRING "Linker flags" FORCE
)

# ----------------------------------------------------------------------------
# Search Path Rules (Avoid searching host Linux directories)
# ----------------------------------------------------------------------------
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
