####
# cmake/platform/stm32f411/Platform/CMakeLists.txt:
#
# STM32 Platform Configuration Module choosing bare-metal / embedded OSAL implementations
####

register_fprime_config(
    STM32PlatformTypes
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/PlatformTypes.fpp"
    HEADERS
        "${CMAKE_CURRENT_LIST_DIR}/PlatformTypes.h"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/startup_stm32f411.c"
    CHOOSES_IMPLEMENTATIONS
        Os_File_Stub
        Os_Console_Stub
        Os_Task_Stub
        Os_Mutex_Stub
        Os_Queue_Stub
        Os_RawTime_Stub
        Os_Cpu_Stub
        Os_Memory_Stub
        Fw_StringFormat_snprintf
        Fw_StringScan_sscanf
    BASE_CONFIG
)
