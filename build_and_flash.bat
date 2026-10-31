@echo off
chcp 936 >nul
setlocal enabledelayedexpansion
rem ============================================================
rem  一键编译 + 烧录（STM32 / CMSIS-DAP 调试器）
rem  在本工程根目录双击运行：先编译，再用 OpenOCD 烧录
rem ============================================================

set "CLT=D:\STM32CubeCLT_1.22.0"
set "CMAKE=%CLT%\CMake\bin\cmake.exe"
set "NINJA=%CLT%\Ninja\bin\ninja.exe"
set "OCD=D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32"
set "OCD_SCRIPTS=%OCD%\share\openocd\scripts"
set "BOARD=board/stm32f4_cmsis-dap.cfg"

for %%I in ("%~dp0.") do set "PROJ=%%~nxI"
set "BUILD=%~dp0cmake-build-debug"
rem OpenOCD 的 -c 参数里是 Tcl 语法，路径必须用正斜杠，否则反斜杠会被当成转义
set "BUILD_F=%BUILD:\=/%"

echo.
echo ============================================================
echo   工程    : %PROJ%
echo   构建目录: %BUILD%
echo   烧录配置: %BOARD%
echo ============================================================
echo.

if not exist "%CMAKE%" (
    echo [错误] 找不到 CMake: %CMAKE%
    pause & exit /b 1
)

echo [1/3] 配置 CMake 工程...
if not exist "%BUILD%\CMakeCache.txt" (
    "%CMAKE%" -S "%~dp0." -B "%BUILD%" -G Ninja -DCMAKE_MAKE_PROGRAM="%NINJA%"
    if errorlevel 1 goto :fail
)

echo.
echo [2/3] 编译固件...
"%CMAKE%" --build "%BUILD%"
if errorlevel 1 goto :fail

echo.
echo [3/3] 通过 CMSIS-DAP 烧录...
if not exist "%OCD%\bin\openocd.exe" (
    echo [错误] 找不到 OpenOCD: %OCD%\bin\openocd.exe
    goto :fail
)
"%OCD%\bin\openocd.exe" -s "%OCD_SCRIPTS%" -f %BOARD% -c "program %BUILD_F%/%PROJ%.elf verify reset exit"
if errorlevel 1 (
    echo.
    echo [烧录失败] 请检查：调试器 USB 线是否插好、SWD 四根线是否接对、板子是否供电。
    echo            手动探测命令：openocd -s "%OCD_SCRIPTS%" -f %BOARD% -c "init" -c "shutdown"
    pause & exit /b 1
)

echo.
echo [完成] 固件已烧录并校验，板子已复位开始运行。
echo.
pause
exit /b 0

:fail
echo.
echo [失败] 某一步出错了，请查看上面的输出。
pause
exit /b 1
