########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.1             | #
# -------------------------------------------------------------------- #
# | Date : 06.12.2024                |     File : CmakeLists.txt     | #
# ---------------------------------------------------------------------#
# | Brief : Bootloader CmakeLists.txt file                           | #
# -------------------------------------------------------------------- #
########################################################################

set(BOOT_EXECUTABLE bootloader)

add_executable(${BOOT_EXECUTABLE})
add_target_properties(${BOOT_EXECUTABLE})

get_target_property(DEFINES ${BOOT_EXECUTABLE} COMPILE_DEFINITIONS)

# Get the first macro defined
list(GET DEFINES 0 DEVICE_DEFINE)

if(DEVICE_DEFINE STREQUAL STM32F303)
    set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_cmd/STM32F303_boot.cmd)
elseif(DEVICE_DEFINE STREQUAL STM32G474)
    set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_cmd/STM32G474_boot.cmd)
endif()

target_link_options(
    ${BOOT_EXECUTABLE} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
    ${LINKER_FILE}
    -Wl,--map_file=${BOOT_EXECUTABLE}.map, -v
)

target_sources(
    ${BOOT_EXECUTABLE} PRIVATE
    "${COMMON_SOURCES}"
    "${PROJECT_SOURCE_DIR}/device/core/setup_boot.c"
    "${PROJECT_SOURCE_DIR}/device/core/hw_init.c"
    "${PROJECT_SOURCE_DIR}/device/drivers/crc/crc.cpp"
    "${PROJECT_SOURCE_DIR}/${BOOT_EXECUTABLE}/boot/boot_jump.cpp"
    "${PROJECT_SOURCE_DIR}/${BOOT_EXECUTABLE}/boot/flash_config.cpp"
    "${PROJECT_SOURCE_DIR}/${BOOT_EXECUTABLE}/boot/crc_compute.cpp"
    "${PROJECT_SOURCE_DIR}/${BOOT_EXECUTABLE}/core/main.cpp"
)