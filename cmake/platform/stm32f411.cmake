####
# stm32f411.cmake:
#
# F Prime Platform configuration for STM32F411CEU6 microcontroller.
####
include_guard()

# Add platform module for 32-bit ARM Cortex-M
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/stm32f411/Platform/")
