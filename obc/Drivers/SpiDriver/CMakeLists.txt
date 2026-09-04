set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/SpiDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/SpiDriver.cpp"
)

set(MOD_DEPS
  obc_Drivers_HalBridge
)

register_fprime_module()
