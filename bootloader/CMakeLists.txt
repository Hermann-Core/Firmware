########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0             | #
# -------------------------------------------------------------------- #
# | Date : 07.02.2023                |     File : CmakeLists.txt     | #
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

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_commands/STM32F303_boot.cmd)
    elseif(DEVICE_DEFINE STREQUAL STM32G473)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_commands/STM32G473_boot.cmd)
    endif()

    target_link_options(
        ${BOOT_EXECUTABLE} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        ${LINKER_FILE}
        -Wl,--map_file=${BOOT_EXECUTABLE}.map, -v
    )
    
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_scripts/STM32F303_boot.ld)
    elseif(DEVICE_DEFINE STREQUAL STM32G473)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_scripts/STM32G473_boot.ld)
    endif()

    target_link_options(
        ${BOOT_EXECUTABLE} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        -specs=nano.specs -specs=nosys.specs
        -T${LINKER_FILE}
        -Wl,--gc-sections
        -Wl,--Map=${BOOT_EXECUTABLE}.map,--cref
    )

endif()

target_include_directories(
    ${BOOT_EXECUTABLE} PRIVATE
    ${CMAKE_SOURCE_DIR}/${BOOT_EXECUTABLE}
    ${CMAKE_SOURCE_DIR}/${BOOT_EXECUTABLE}/Inc
)

target_sources(
    ${BOOT_EXECUTABLE} PRIVATE
    ${COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/device/Src/setup_boot.c
    ${CMAKE_SOURCE_DIR}/device/Src/hardware_init.cpp
    ${CMAKE_SOURCE_DIR}/${BOOT_EXECUTABLE}/Src/boot_jump.cpp
    ${CMAKE_SOURCE_DIR}/${BOOT_EXECUTABLE}/bootloader_main.cpp
    ${CMAKE_SOURCE_DIR}/device/hardware_drivers/Src/crc.cpp
    ${CMAKE_SOURCE_DIR}/${BOOT_EXECUTABLE}/Src/flash_config.cpp
)