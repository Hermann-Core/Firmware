########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0.1           | #
# -------------------------------------------------------------------- #
# | Date : 11.26.2023                |     File : CmakeLists.txt     | #
# ---------------------------------------------------------------------#
# | Brief : Top level CmakeLists.txt file                            | #
# -------------------------------------------------------------------- #
########################################################################

cmake_minimum_required(VERSION 3.24)

project("EBIKE" C CXX ASM)

include(cmake/stm32_project.cmake)

set(COMMON_SOURCES
    "${CMAKE_SOURCE_DIR}/device/core/src/hardware_core.cpp"
    "${CMAKE_SOURCE_DIR}/device/core/src/peripherals_irqs.cpp"
    "${CMAKE_SOURCE_DIR}/common/src/assert.c"
    "${CMAKE_SOURCE_DIR}/logging/rtt/SEGGER_RTT_ASM_ARMv7M.S"
    "${CMAKE_SOURCE_DIR}/logging/rtt/SEGGER_RTT.c"
    "${CMAKE_SOURCE_DIR}/logging/rtt/SEGGER_RTT_printf.c"
    "${CMAKE_SOURCE_DIR}/device/drivers/rcc/rcc.cpp"
    "${CMAKE_SOURCE_DIR}/device/drivers/flash/flash.cpp"
    "${CMAKE_SOURCE_DIR}/service/protocol.cpp"
)

# get the list of the include directories
header_directories(header_dir_list)
list(LENGTH header_dir_list header_dir_list_count)

if(header_dir_list_count LESS 1)
    message(FATAL_ERROR "Could not find include directories. Please check your project.")
else()
    message(STATUS "[INFO] Found ${header_dir_list_count} header directories.")
endif()

include_directories(
    ${header_dir_list}
)

link_directories(common/arm_dsp/lib)

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")
    link_libraries(arm_cortexM4lf_math.lib)
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")
    link_libraries(libarm_cortexM4lf_math.a)
endif()

add_subdirectory(bootloader)
add_subdirectory(application)
