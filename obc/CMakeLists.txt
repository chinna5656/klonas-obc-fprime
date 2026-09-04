# This CMake file is intended to register project-wide objects.
# This allows for reuse between deployments, or other projects.

add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Config")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Ports")
# Components are registered by deployment (KlonasDeployment/CMakeLists.txt)
# add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Components")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Drivers")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/KlonasDeployment/")
