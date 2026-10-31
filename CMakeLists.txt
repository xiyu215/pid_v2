# =============================================================================
#  STM32 固件工程 —— 通用 CMake 模板（GCC / arm-none-eabi）
# -----------------------------------------------------------------------------
#  这个文件可以直接复制到任何 STM32CubeMX 生成的工程根目录下使用。
#
#  它会自动识别：
#    · 工程名        ← 当前文件夹名
#    · 芯片宏        ← .mxproject 里的 CDefines（如 STM32F103xB）
#    · CPU 内核      ← 由芯片宏推导（如 STM32F1 → cortex-m3）
#    · 启动文件      ← Core/Startup/startup_*.s
#    · 链接脚本      ← 根目录下的 *.ld
#
#  所以换芯片、换工程都不用改这个文件。若自动识别不对，
#  在下面的「手动覆盖区」填上即可。
#
#  用法：
#    cmake -S . -B build -G Ninja
#    cmake --build build
#  产物：build/<工程名>.elf / .hex / .bin
#
#  CubeMX 重新生成代码只会覆盖 Core/ 与 Drivers/，不会动这个文件。
# =============================================================================

cmake_minimum_required(VERSION 3.22)

# =============================================================================
#  手动覆盖区（留空 = 自动识别）
# =============================================================================
set(APP_NAME           "")   # 工程名，留空则用当前文件夹名
set(MCU_DEFINE_OVERRIDE "")  # 芯片宏，如 STM32F103xB
set(MCU_CPU_OVERRIDE    "")  # 内核，如 cortex-m3 / cortex-m4 / cortex-m7

# -----------------------------------------------------------------------------
# 自动加载自带的 ARM 工具链（无需在命令行指定 CMAKE_TOOLCHAIN_FILE）
# -----------------------------------------------------------------------------
if(NOT CMAKE_TOOLCHAIN_FILE)
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")
endif()

# =============================================================================
#  1. 工程名
# =============================================================================
if(NOT APP_NAME)
    get_filename_component(APP_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    string(REPLACE " " "_" APP_NAME "${APP_NAME}")
endif()

project(${APP_NAME} LANGUAGES C ASM)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi")
    message(FATAL_ERROR
        "当前使用的 C 编译器是: ${CMAKE_C_COMPILER}\n"
        "这不是 ARM 交叉编译器，无法生成 STM32 固件。\n"
        "请删除构建目录（cmake-build-debug / build）后重新加载 CMake 项目。")
endif()

# =============================================================================
#  2. 芯片宏（从 .mxproject 的 CDefines 自动提取）
# =============================================================================
set(MCU_DEFINE "${MCU_DEFINE_OVERRIDE}")

if(NOT MCU_DEFINE)
    set(_MXP "${CMAKE_CURRENT_SOURCE_DIR}/.mxproject")
    if(EXISTS "${_MXP}")
        file(STRINGS "${_MXP}" _CDEF_LINE REGEX "^CDefines=")
        if(_CDEF_LINE)
            # 注意：CMake 正则不支持 {3} 这种重复写法，必须把 [0-9] 写三次
            string(REGEX MATCH "STM32[FGLHUWCN][0-9][0-9][0-9][A-Za-z0-9]*" MCU_DEFINE "${_CDEF_LINE}")
        endif()
    endif()
endif()

if(NOT MCU_DEFINE)
    message(FATAL_ERROR
        "无法自动识别芯片宏。请在 CMakeLists.txt 的「手动覆盖区」里设置：\n"
        "    set(MCU_DEFINE_OVERRIDE \"STM32F103xB\")")
endif()
message(STATUS "芯片宏        : ${MCU_DEFINE}")

# =============================================================================
#  3. CPU 内核（由芯片宏推导）
# =============================================================================
set(MCU_CPU "${MCU_CPU_OVERRIDE}")

if(NOT MCU_CPU)
    string(REGEX REPLACE "^((STM32[A-Z][0-9]).*)$" "\\2" _FAMILY "${MCU_DEFINE}")

    set(_CPU_TABLE
        "STM32C0:cortex-m0plus"
        "STM32F0:cortex-m0"
        "STM32F1:cortex-m3"
        "STM32F2:cortex-m3"
        "STM32F3:cortex-m4"
        "STM32F4:cortex-m4"
        "STM32F7:cortex-m7"
        "STM32G0:cortex-m0plus"
        "STM32G4:cortex-m4"
        "STM32H5:cortex-m33"
        "STM32H7:cortex-m7"
        "STM32L0:cortex-m0plus"
        "STM32L1:cortex-m3"
        "STM32L4:cortex-m4"
        "STM32L5:cortex-m33"
        "STM32U0:cortex-m0plus"
        "STM32U5:cortex-m33"
        "STM32WB:cortex-m4"
        "STM32WL:cortex-m4"
    )

    foreach(_entry IN LISTS _CPU_TABLE)
        string(REPLACE ":" ";" _kv "${_entry}")
        list(GET _kv 0 _key)
        list(GET _kv 1 _val)
        if(_key STREQUAL _FAMILY)
            set(MCU_CPU "${_val}")
        endif()
    endforeach()
endif()

if(NOT MCU_CPU)
    message(WARNING
        "无法从 ${MCU_DEFINE} 推导 CPU 内核，已回退到 cortex-m3。\n"
        "若不是 STM32F1/F2/L1，请在「手动覆盖区」设置 MCU_CPU_OVERRIDE。")
    set(MCU_CPU "cortex-m3")
endif()
message(STATUS "CPU 内核      : ${MCU_CPU}")

# 浮点 ABI：Cortex-M4F/M7/M33 自带 FPU，必须开硬件浮点。
# 若用 -mfloat-abi=soft，port.c 里的 VFP 指令汇编不过，
# 且 __FPU_USED 为 0 会让 SystemInit 不打开 CPACR。
set(MCU_FPU "")
if(MCU_CPU MATCHES "^cortex-m7")
    set(MCU_FPU "fpv5-d16")
elseif(MCU_CPU MATCHES "^cortex-m4")
    set(MCU_FPU "fpv4-sp-d16")
elseif(MCU_CPU MATCHES "^cortex-m33")
    set(MCU_FPU "fpv5-sp-d16")
endif()

set(MCU_FLAGS -mcpu=${MCU_CPU} -mthumb)
if(MCU_FPU)
    list(APPEND MCU_FLAGS -mfpu=${MCU_FPU} -mfloat-abi=hard)
    message(STATUS "浮点 ABI      : hard (${MCU_FPU})")
else()
    # 无 FPU 的内核（M0/M3 等）走软浮点
    list(APPEND MCU_FLAGS -mfloat-abi=soft)
    message(STATUS "浮点 ABI      : soft（${MCU_CPU} 无 FPU）")
endif()

# =============================================================================
#  4. 启动文件与链接脚本（自动查找）
# =============================================================================
file(GLOB _STARTUP_LIST "${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_*.s")
list(LENGTH _STARTUP_LIST _STARTUP_COUNT)
if(_STARTUP_COUNT EQUAL 0)
    message(FATAL_ERROR "找不到启动文件：Core/Startup/startup_*.s")
elseif(_STARTUP_COUNT GREATER 1)
    message(FATAL_ERROR "Core/Startup 下有多个启动文件，请只保留一个：\n  ${_STARTUP_LIST}")
endif()
set(STARTUP_SOURCE "${_STARTUP_LIST}")
message(STATUS "启动文件      : ${STARTUP_SOURCE}")

file(GLOB _LD_LIST "${CMAKE_CURRENT_SOURCE_DIR}/*.ld")
list(LENGTH _LD_LIST _LD_COUNT)

if(_LD_COUNT EQUAL 0)
    message(FATAL_ERROR "找不到链接脚本：根目录下的 *.ld")
endif()

if(_LD_COUNT GREATER 1)
    # CubeMX 有时会同时生成 FLASH 版和 RAM 版两份链接脚本，
    # 此时优先选用 *_FLASH.ld（正常烧到 Flash 里跑的那份）。
    set(_LD_FLASH_LIST ${_LD_LIST})
    list(FILTER _LD_FLASH_LIST INCLUDE REGEX "_FLASH\\.ld$")
    list(LENGTH _LD_FLASH_LIST _LD_FLASH_COUNT)

    if(_LD_FLASH_COUNT EQUAL 1)
        message(STATUS "发现 ${_LD_COUNT} 个链接脚本，自动选用 FLASH 版：${_LD_FLASH_LIST}")
        set(_LD_LIST ${_LD_FLASH_LIST})
    else()
        message(FATAL_ERROR
            "根目录下有多个链接脚本，且无法自动确定用哪个，请只保留一个：\n  ${_LD_LIST}")
    endif()
endif()

set(LINKER_SCRIPT "${_LD_LIST}")
message(STATUS "链接脚本      : ${LINKER_SCRIPT}")

# =============================================================================
#  5. 头文件搜索路径
# =============================================================================
set(INCLUDE_DIRS
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

# 若工程用了别的系列（如 F4），把 HAL/CMSIS 目录按实际补进来
file(GLOB _HAL_INC "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32*_HAL_Driver/Inc")
file(GLOB _CMSIS_DEV_INC "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32*/Include")
list(APPEND INCLUDE_DIRS ${_HAL_INC} ${_CMSIS_DEV_INC})

# FreeRTOS 中间件（CubeMX 勾选 FreeRTOS 后才会出现该目录）
# cmsis_os.h 位于 Source/CMSIS_RTOS，portmacro.h 位于 Source/portable/GCC/ARM_CMx[x]
set(_FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source")
if(EXISTS "${_FREERTOS_DIR}/include/FreeRTOS.h")
    # 端口层目录随内核不同（ARM_CM3 / ARM_CM4F / ARM_CM7 ...），取实际存在的那一个
    file(GLOB _FREERTOS_PORT_INC "${_FREERTOS_DIR}/portable/GCC/ARM_CM*")
    list(APPEND INCLUDE_DIRS
        "${_FREERTOS_DIR}/include"
        "${_FREERTOS_DIR}/CMSIS_RTOS"
        ${_FREERTOS_PORT_INC}
    )
endif()

# =============================================================================
#  6. 源文件
# =============================================================================
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c"
)

file(GLOB_RECURSE HAL_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32*_HAL_Driver/Src/*.c"
)

# FreeRTOS 内核 + CMSIS-RTOS v1 封装 + 堆管理 + 端口层
set(FREERTOS_SOURCES "")
if(EXISTS "${_FREERTOS_DIR}/include/FreeRTOS.h")
    file(GLOB FREERTOS_SOURCES CONFIGURE_DEPENDS
        "${_FREERTOS_DIR}/*.c"
        "${_FREERTOS_DIR}/CMSIS_RTOS/*.c"
        "${_FREERTOS_DIR}/portable/GCC/ARM_CM*/*.c"
        "${_FREERTOS_DIR}/portable/MemMang/heap_4.c"
    )
endif()

set(PROJECT_SOURCES
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${STARTUP_SOURCE}
)

# =============================================================================
#  7. 编译选项
# =============================================================================
add_compile_definitions(
    ${MCU_DEFINE}
    USE_HAL_DRIVER
)

add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -fdata-sections
    -ffunction-sections
)

# 注意：每个选项必须写成独立的生成器表达式，不能写在一个里用分号分隔，
#       否则 CMake 会在分号处把参数拆断，导致 gcc 报 "-gdwarf- 参数错误"。
add_compile_options(
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Debug>:-gdwarf-2>
    $<$<CONFIG:Release>:-Os>
    $<$<CONFIG:Release>:-g0>
    $<$<CONFIG:RelWithDebInfo>:-O2>
    $<$<CONFIG:RelWithDebInfo>:-g3>
    $<$<CONFIG:MinSizeRel>:-Os>
)

# =============================================================================
#  8. 可执行文件（固件）
# =============================================================================
add_executable(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INCLUDE_DIRS})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${LINKER_SCRIPT}"
    OUTPUT_NAME "${CMAKE_PROJECT_NAME}"
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -specs=nano.specs
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -lc
    -lm
    # 不链接 -lnosys：工程自带 syscalls.c / sysmem.c 已实现 _write/_sbrk 等，
    # 再链接 libnosys.a 会造成 _exit/_close/_lseek/_sbrk 重复定义。
)

# =============================================================================
#  9. 生成 .hex / .bin 并打印固件体积
# =============================================================================
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "生成 ${CMAKE_PROJECT_NAME}.hex / .bin"
)

# =============================================================================
# 10. 烧录目标（需要 STM32CubeProgrammer）
#     用法：cmake --build build --target flash
# =============================================================================
find_program(OPENOCD_EXE
    NAMES openocd${_EXE}
    HINTS
        "D:/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin"
        "C:/Program Files/OpenOCD/bin"
    ENV OPENOCD_PATH
)

find_program(STM32_PROGRAMMER_CLI
    NAMES STM32_Programmer_CLI${_EXE}
    HINTS
        "D:/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin"
        "C:/ST/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin"
        "$ENV{LOCALAPPDATA}/Programs/STM32CubeCLT/STM32CubeProgrammer/bin"
    ENV STM32CUBECLT_PATH
)

# 按芯片型号挑选 OpenOCD 板级配置（CMSIS-DAP 探针）
if(MCU_DEFINE MATCHES "^STM32F1")
    set(OPENOCD_BOARD "board/stm32f1_cmsis-dap.cfg")
elseif(MCU_DEFINE MATCHES "^STM32F4")
    set(OPENOCD_BOARD "board/stm32f4_cmsis-dap.cfg")
else()
    set(OPENOCD_BOARD "")
endif()

if(OPENOCD_EXE AND OPENOCD_BOARD)
    get_filename_component(_OCD_BIN_DIR "${OPENOCD_EXE}" DIRECTORY)
    get_filename_component(_OCD_ROOT "${_OCD_BIN_DIR}" DIRECTORY)
    message(STATUS "OpenOCD 板级配置: ${OPENOCD_BOARD}")
    add_custom_target(flash
        COMMAND ${OPENOCD_EXE}
                -s "${_OCD_ROOT}/share/openocd/scripts"
                -f ${OPENOCD_BOARD}
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "通过 CMSIS-DAP 烧录 ${CMAKE_PROJECT_NAME}.elf"
        VERBATIM
    )
elseif(STM32_PROGRAMMER_CLI)
    add_custom_target(flash
        COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD -w
                $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -v -rst
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "通过 ST-LINK 烧录 ${CMAKE_PROJECT_NAME}.elf"
        VERBATIM
    )
else()
    message(STATUS "未找到 OpenOCD / STM32_Programmer_CLI，已跳过 flash 目标")
endif()
