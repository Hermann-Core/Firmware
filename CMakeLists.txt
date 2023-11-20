########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0             | #
# -------------------------------------------------------------------- #
# | Date : 07.02.2023                |     File : CmakeLists.txt     | #
# ---------------------------------------------------------------------#
# | Brief : Top level CmakeLists.txt file                            | #
# -------------------------------------------------------------------- #
########################################################################

cmake_minimum_required(VERSION 3.24)

project("EBIKE" C CXX ASM)

include(cmake/stm32_project.cmake)

set(COMMON_SOURCES
    # ${CMAKE_SOURCE_DIR}/device/Src/startup_stm32f303vctx.s
    "${CMAKE_SOURCE_DIR}/device/core/src/hardware_core.cpp"
    "${CMAKE_SOURCE_DIR}/device/core/src/peripherals_irqs.cpp"
    "${CMAKE_SOURCE_DIR}/RTT/src/SEGGER_RTT_ASM_ARMv7M.S"
    "${CMAKE_SOURCE_DIR}/RTT/src/SEGGER_RTT.c"
    "${CMAKE_SOURCE_DIR}/RTT/src/SEGGER_RTT_printf.c"
    "${CMAKE_SOURCE_DIR}/device/drivers/flash/flash.cpp"
    "${CMAKE_SOURCE_DIR}/service/protocol.cpp"
)

include_directories(
    "service"
    "RTT/inc"
    "device/CMSIS"
    "device/core/inc"
    "common/inc"
    "common/arm_dsp/inc"
    "device/drivers/crc"
    "device/drivers/flash"
)

link_directories(common/arm_dsp/lib)

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")
    link_libraries(arm_cortexM4lf_math.lib)
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")
    link_libraries(libarm_cortexM4lf_math.a)
endif()

add_subdirectory(bootloader)
add_subdirectory(application)
