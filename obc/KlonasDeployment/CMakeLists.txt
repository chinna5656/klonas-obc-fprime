#####
# 'KlonasDeployment' Deployment:
#
# This registers the 'KlonasDeployment' deployment to the build system. 
# Custom components that have not been added at the project-level should be added to 
# the list below.
#
#####

###
# Topology and Components
###

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Top/")

# Custom flight components for KLONAS Phase-1 CubeSat
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/NavPredictor" "${CMAKE_BINARY_DIR}/obc/Components/NavPredictor")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/ParachuteDeployer" "${CMAKE_BINARY_DIR}/obc/Components/ParachuteDeployer")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/CommsCrypto" "${CMAKE_BINARY_DIR}/obc/Components/CommsCrypto")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/EnvSensors" "${CMAKE_BINARY_DIR}/obc/Components/EnvSensors")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/PowerMonitor" "${CMAKE_BINARY_DIR}/obc/Components/PowerMonitor")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../Components/DataLogger" "${CMAKE_BINARY_DIR}/obc/Components/DataLogger")

if (CMAKE_SYSTEM_NAME STREQUAL "Generic" OR FPRIME_PLATFORM STREQUAL "stm32f411")
    set(DEPLOYMENT_CHOSEN_IMPLEMENTATIONS
        CHOOSES_IMPLEMENTATIONS
            Os_Task_Stub
            Os_Mutex_Stub
            Os_Generic_PriorityQueue
            Os_RawTime_Stub
            Os_Console_Stub
            Os_File_Stub
            Os_Cpu_Stub
            Os_Memory_Stub
    )
endif()

register_fprime_deployment(
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/Main.cpp"
    DEPENDS
        ${FPRIME_CURRENT_MODULE}_Top
    ${DEPLOYMENT_CHOSEN_IMPLEMENTATIONS}
)
