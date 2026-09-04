set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/AdcDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/AdcDriver.cpp"
)

set(MOD_DEPS
  obc_Drivers_HalBridge
)

register_fprime_module()
