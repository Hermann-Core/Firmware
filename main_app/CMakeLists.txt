########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0             | #
# -------------------------------------------------------------------- #
# | Date : 07.02.2023                |     File : CmakeLists.txt    s | #
# ---------------------------------------------------------------------#
# | Brief : Main app CmakeLists.txt file                             | #
# -------------------------------------------------------------------- #
########################################################################

set(MAIN_EXECUTABLE main_app)

add_executable(${MAIN_EXECUTABLE})
add_target_properties(${MAIN_EXECUTABLE})

get_target_property(DEFINES ${MAIN_EXECUTABLE} COMPILE_DEFINITIONS)

# Get the first macro defined
list(GET DEFINES 0 DEVICE_DEFINE)

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_commands/STM32F303_app.cmd)
    elseif(DEVICE_DEFINE STREQUAL STM32G473)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_commands/STM32G473_app.cmd)
    endif()

    target_link_options(
        ${MAIN_EXECUTABLE} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
        ${LINKER_FILE}
        -Wl,--map_file=${MAIN_EXECUTABLE}.map, -v
    )
    
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")

    if(DEVICE_DEFINE STREQUAL STM32F303)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_scripts/STM32F303_app.ld)
    elseif(DEVICE_DEFINE STREQUAL STM32G473)
        set(LINKER_FILE   ${CMAKE_SOURCE_DIR}/device/linker_scripts/STM32G473_app.ld)
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

target_include_directories(
    ${MAIN_EXECUTABLE} PRIVATE
    ${CMAKE_SOURCE_DIR}/main_app
    ${CMAKE_SOURCE_DIR}/main_app/Inc
)

target_sources(
    ${MAIN_EXECUTABLE} PRIVATE
    ${COMMON_SOURCES}
    ${CMAKE_SOURCE_DIR}/device/Src/setup_app.c
    ${CMAKE_SOURCE_DIR}/main_app/app_main.cpp
)