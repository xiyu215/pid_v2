# =============================================================================
#  gcc-arm-none-eabi.cmake —— STM32 (Cortex-M) 交叉编译工具链
# -----------------------------------------------------------------------------
#  用法：
#     cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
#
#  工具链查找顺序：
#     1) -DARM_TOOLCHAIN_DIR=<arm-none-eabi-gcc 所在目录>
#     2) 环境变量 ARM_TOOLCHAIN_DIR
#     3) 系统 PATH
#     4) STM32CubeCLT 常见安装路径
# =============================================================================

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 交叉编译阶段不做真实链接（此时还没有链接脚本），用静态库方式做编译器探测
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(WIN32)
    set(_EXE ".exe")
else()
    set(_EXE "")
endif()

# -----------------------------------------------------------------------------
# 定位 arm-none-eabi-gcc
# -----------------------------------------------------------------------------
set(ARM_TOOLCHAIN_DIR "" CACHE PATH "arm-none-eabi 工具链 bin 目录（留空则自动查找）")

if(ARM_TOOLCHAIN_DIR)
    set(_ARM_HINTS "${ARM_TOOLCHAIN_DIR}")
else()
    set(_ARM_HINTS
        "D:/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin"
        "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin"
        "$ENV{LOCALAPPDATA}/Programs/STM32CubeCLT/GNU-tools-for-STM32/bin"
        "/opt/STM32CubeCLT/GNU-tools-for-STM32/bin"
        "/Applications/STM32CubeCLT/GNU-tools-for-STM32/bin"
    )
endif()

find_program(_ARM_GCC arm-none-eabi-gcc${_EXE}
    HINTS ${_ARM_HINTS}
    ENV   ARM_TOOLCHAIN_DIR
)

if(NOT _ARM_GCC)
    message(FATAL_ERROR
        "找不到 arm-none-eabi-gcc。\n"
        "  · 请先安装 STM32CubeCLT（ST 官网免费下载），或\n"
        "  · 用 -DARM_TOOLCHAIN_DIR=<工具链 bin 目录> 显式指定。")
endif()

get_filename_component(_ARM_BIN "${_ARM_GCC}" DIRECTORY)
message(STATUS "ARM 工具链目录: ${_ARM_BIN}")

# -----------------------------------------------------------------------------
# 声明编译器 / 工具
# -----------------------------------------------------------------------------
set(CMAKE_C_COMPILER   "${_ARM_BIN}/arm-none-eabi-gcc${_EXE}"     CACHE FILEPATH "" FORCE)
set(CMAKE_ASM_COMPILER "${_ARM_BIN}/arm-none-eabi-gcc${_EXE}"     CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_ARM_BIN}/arm-none-eabi-g++${_EXE}"     CACHE FILEPATH "" FORCE)

set(CMAKE_OBJCOPY      "${_ARM_BIN}/arm-none-eabi-objcopy${_EXE}" CACHE FILEPATH "" FORCE)
set(CMAKE_OBJDUMP      "${_ARM_BIN}/arm-none-eabi-objdump${_EXE}" CACHE FILEPATH "" FORCE)
set(CMAKE_SIZE         "${_ARM_BIN}/arm-none-eabi-size${_EXE}"    CACHE FILEPATH "" FORCE)
set(CMAKE_AR           "${_ARM_BIN}/arm-none-eabi-ar${_EXE}"      CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB       "${_ARM_BIN}/arm-none-eabi-ranlib${_EXE}"  CACHE FILEPATH "" FORCE)
set(CMAKE_STRIP        "${_ARM_BIN}/arm-none-eabi-strip${_EXE}"   CACHE FILEPATH "" FORCE)

# -----------------------------------------------------------------------------
# 只在工具链目录里查找头文件 / 库，避免误用主机的库
# -----------------------------------------------------------------------------
set(CMAKE_FIND_ROOT_PATH "${_ARM_BIN}/..")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
