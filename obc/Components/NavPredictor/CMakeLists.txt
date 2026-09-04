set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/NavPredictor.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/NavPredictor.cpp"
)

register_fprime_module()

### Unit Tests ###
set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/NavPredictor.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/Tester.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/TesterTests.cpp"
)

set(UT_HEADER_FILES
  "${CMAKE_CURRENT_LIST_DIR}/test/ut/Tester.hpp"
)

set(UT_MOD_DEPS
  obc_Ports
)

set(UT_AUTO_HELPERS ON)

register_fprime_ut()
