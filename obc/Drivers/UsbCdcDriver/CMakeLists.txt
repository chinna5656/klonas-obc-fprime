set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/UsbCdcDriver.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsbCdcDriver.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/usbd/usb_clock.c"
  "${CMAKE_CURRENT_LIST_DIR}/usbd/usbd_desc.c"
  "${CMAKE_CURRENT_LIST_DIR}/usbd/usbd_conf.c"
  "${CMAKE_CURRENT_LIST_DIR}/usbd/usbd_cdc_if.c"
)

set(MOD_DEPS
  obc_Drivers_HalBridge
)

register_fprime_module()
