########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0.1           | #
# -------------------------------------------------------------------- #
# | Date : 11.26.2023                |     File : CmakeLists.txt    s| #
# ---------------------------------------------------------------------#
# | Brief : Main app CmakeLists.txt file                             | #
# -------------------------------------------------------------------- #
########################################################################

set(MAIN_EXECUTABLE application)

add_executable(${MAIN_EXECUTABLE})
add_target_properties(${MAIN_EXECUTABLE})

get_target_property(DEFINES ${MAIN_EXECUTABLE} COMPILE_DEFINITIONS)

# Get the first macro defined
list(GET DEFINES 0 DEVICE_DEFINE)

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_cmd/STM32F303_app.cmd)
    elseif(DEVICE_DEFINE STREQUAL STM32G474)
        set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_cmd/STM32G474_app.cmd)
    endif()

    target_link_options(
        ${MAIN_EXECUTABLE} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        ${LINKER_FILE}
        -Wl,--map_file=${MAIN_EXECUTABLE}.map, -v
    )

elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_scripts/STM32F303_app.ld)
    elseif(DEVICE_DEFINE STREQUAL STM32G474)
        set(LINKER_FILE   ${PROJECT_SOURCE_DIR}/device/config/linker_scripts/STM32G474_app.ld)
    endif()

    target_link_options(
        ${MAIN_EXECUTABLE} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        -specs=nano.specs -specs=nosys.specs
        -T${LINKER_FILE}
        -Wl,--gc-sections
        -Wl,--Map=${MAIN_EXECUTABLE}.map,--cref
    )

endif()

target_sources(
    ${MAIN_EXECUTABLE} PRIVATE
    "${COMMON_SOURCES}"
    "${PROJECT_SOURCE_DIR}/device/core/setup_app.c"
    "${PROJECT_SOURCE_DIR}/${MAIN_EXECUTABLE}/core/main.cpp"
)