########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0             | #
# -------------------------------------------------------------------- #
# | Date : 07.02.2023                |     File: stm32_project.cmake | #
# ---------------------------------------------------------------------#
# | This file is use to configure the common stm32 target options    | #
# -------------------------------------------------------------------- #
########################################################################

function(add_target_properties TARGET_NAME)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    STM32F303
    _DSP_LIB
    _USE_RTT
    _ENABLE_IRQ
)

if(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{TI_TOOLCHAIN_PATH}")

    target_compile_options(
        ${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -flto -fno-common -ffunction-sections -fdata-sections -fstack-usage
        -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-strict-aliasing
    )

elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")

    target_compile_options(
        ${TARGET_NAME} PRIVATE
        -fdiagnostics-color=always
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -specs=nano.specs -specs=nosys.specs
        -fno-common -ffunction-sections -fdata-sections -fstack-usage
        -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-strict-aliasing
    )

endif()

target_compile_features(
    ${TARGET_NAME} PUBLIC 
    c_std_17
    cxx_std_14
)

target_link_libraries(
    ${TARGET_NAME} PRIVATE
)

target_link_directories(
    ${TARGET_NAME} PRIVATE
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

endfunction()