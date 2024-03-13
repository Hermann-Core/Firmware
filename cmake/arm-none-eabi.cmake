########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.0             | #
# -------------------------------------------------------------------- #
# | Date : 07.02.2023                |     File: arm-none-eabi.cmake | #
# ---------------------------------------------------------------------#
# | Brief : This file is use to configure the ARM  toolchain         | #
# -------------------------------------------------------------------- #
########################################################################

# Find a toolchain installation to use
file(GLOB TOOLCHAIN_DIRECTORIES
    "$ENV{TI_TOOLCHAIN_PATH}"
    # "$ENV{GNU_TOOLCHAIN_PATH}"
    "/opt/GNU Arm Embedded Toolchain/12.2-rel1/bin"
)

# Count the available toolchains
list(LENGTH TOOLCHAIN_DIRECTORIES TOOLCHAIN_DIRECTORIES_COUNT)

if(TOOLCHAIN_DIRECTORIES_COUNT LESS 1)
    message(FATAL_ERROR "Could not find an ARM toolchain installation. Please install a toolchain.")
else()
    list(GET TOOLCHAIN_DIRECTORIES 0 TOOLCHAIN_DIRECTORY)
    if (TOOLCHAIN_DIRECTORIES_COUNT GREATER 1)
        message(STATUS "Found multiple ARM toolchain installations. Using the default \"${TOOLCHAIN_DIRECTORY}\".")
    endif()
endif()

# Set the suffix of executables if on Windows
if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
endif()

set(CMAKE_SYSTEM_NAME                  Generic)
set(CMAKE_SYSTEM_PROCESSOR             ARM)

# If the toolchain is the Texas Instruments ARM Clang toolchain
if(TOOLCHAIN_DIRECTORY PATH_EQUAL      "$ENV{TI_TOOLCHAIN_PATH}")
    set(TOOLCHAIN_PREFIX               "tiarm")
# If the toolchain is the GCC embedded toolchain
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL  "$ENV{GNU_TOOLCHAIN_PATH}")
    set(TOOLCHAIN_PREFIX               "arm-none-eabi-")
endif()

if(DEFINED TOOLCHAIN_DIRECTORY)
    set(TOOLCHAIN_PREFIX               "${TOOLCHAIN_DIRECTORY}/${TOOLCHAIN_PREFIX}")
endif()

set(ASM_OPTIONS                        "-x assembler-with-cpp")
set(CPP_OPTIONS                        "-fno-rtti -fno-exceptions -fno-threadsafe-statics -fno-unwind-tables")

if(TOOLCHAIN_DIRECTORY PATH_EQUAL     "$ENV{TI_TOOLCHAIN_PATH}")
    set(CMAKE_C_COMPILER               ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX})
    set(CMAKE_ASM_COMPILER             ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX} ${ASM_OPTIONS})
    set(CMAKE_CXX_COMPILER             ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX} ${CPP_OPTIONS})
elseif(TOOLCHAIN_DIRECTORY PATH_EQUAL "$ENV{GNU_TOOLCHAIN_PATH}")
    set(CMAKE_C_COMPILER               ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX})
    set(CMAKE_ASM_COMPILER             ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} ${ASM_OPTIONS})
    set(CMAKE_CXX_COMPILER             ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX} ${CPP_OPTIONS})
endif()

set(CMAKE_OBJCOPY                      ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE                         ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})
set(CMAKE_READELF                      ${TOOLCHAIN_PREFIX}readelf${TOOLCHAIN_SUFFIX})

set(CMAKE_EXECUTABLE_SUFFIX_ASM        ".out")
set(CMAKE_EXECUTABLE_SUFFIX_C          ".out")
set(CMAKE_EXECUTABLE_SUFFIX_CXX        ".out")

set(CMAKE_TRY_COMPILE_TARGET_TYPE      STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH               ${TOOLCHAIN_DIRECTORY})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM  NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY  ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE  ONLY)