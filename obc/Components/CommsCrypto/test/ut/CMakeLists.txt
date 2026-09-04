####
# CommsCrypto Unit Test CMakeLists.txt
####

set(UT_SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/../../CommsCrypto.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Tester.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TesterTests.cpp"
)

set(UT_HEADER_FILES
  "${CMAKE_CURRENT_LIST_DIR}/Tester.hpp"
)

set(UT_AUTO_HELPERS ON)

register_fprime_ut()
