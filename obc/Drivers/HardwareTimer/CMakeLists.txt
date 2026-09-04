set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/HardwareTimer.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/HardwareTimer.cpp"
)

set(MOD_DEPS
  obc_Drivers_HalBridge
)

register_fprime_module()
