########################################################################
# -------------------------------------------------------------------- #
# | Author : Awatsa Hermann          |     Version : 1.1             | #
# -------------------------------------------------------------------- #
# | Date : 06.12.2024                |     File: arm-none-eabi.cmake | #
# ---------------------------------------------------------------------#
# | Brief : This file is use to configure the ARM  toolchain         | #
# -------------------------------------------------------------------- #
########################################################################

# Find a toolchain installation to use
file(GLOB TOOLCHAIN_DIRECTORIES
    "C:/TI_LLVM_Arm_Embedded_Toolchain/bin"
    "/home/hermann-core/TI_LLVM_ARM_TOOLCHAIN/bin"
)

# Count the available toolchains
list(LENGTH TOOLCHAIN_DIRECTORIES TOOLCHAIN_DIRECTORIES_COUNT)

if(TOOLCHAIN_DIRECTORIES_COUNT LESS 1)
    message(FATAL_ERROR "Could not find an ARM toolchain installation. Please install a toolchain.")
else()
    list(GET TOOLCHAIN_DIRECTORIES 0 TOOLCHAIN_DIRECTORY)
endif()

# Set the suffix of executables if on Windows
if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
endif()

set(CMAKE_SYSTEM_NAME                  Generic)
set(CMAKE_SYSTEM_PROCESSOR             ARM)

set(TOOLCHAIN_PREFIX                   "tiarm")

if(DEFINED TOOLCHAIN_DIRECTORY)
    set(TOOLCHAIN_PREFIX               "${TOOLCHAIN_DIRECTORY}/${TOOLCHAIN_PREFIX}")
endif()

set(ASM_OPTIONS                        "-x assembler-with-cpp")
set(CPP_OPTIONS                        "-fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_COMPILER                   ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX})
set(CMAKE_ASM_COMPILER                 ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX} ${ASM_OPTIONS})
set(CMAKE_CXX_COMPILER                 ${TOOLCHAIN_PREFIX}clang${TOOLCHAIN_SUFFIX} ${CPP_OPTIONS})

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