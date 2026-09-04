####
# obc/Config/CMakeLists.txt:
#
# Configuration overrides for KLONAS Phase-1 Flight Software
####

register_fprime_config(
    KlonasConfig
    CONFIGURATION_OVERRIDES
        "${CMAKE_CURRENT_LIST_DIR}/TlmChanImplCfg.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/DpCatalogCfg.hpp"
        "${CMAKE_CURRENT_LIST_DIR}/FpConstants.fpp"
    INTERFACE
)
