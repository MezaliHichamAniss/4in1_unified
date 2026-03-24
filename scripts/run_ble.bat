@echo off
REM ==========================================================================
REM  run_ble.bat  –  Launch 4in1_unified WITH BLE hardware
REM
REM  Usage:
REM    scripts\run_ble.bat                     (prompt for address)
REM    scripts\run_ble.bat AA:BB:CC:DD:EE:FF   (specify BLE MAC)
REM ==========================================================================
setlocal

set EXE=build\bin\Release\4in1_unified.exe
if not exist "%EXE%" set EXE=build\bin\4in1_unified.exe

if not exist "%EXE%" (
    echo [ERROR] Executable not found. Build first:
    echo   scripts\build_windows.bat
    exit /b 1
)

set BLE_ADDR=%~1
if "%BLE_ADDR%"=="" (
    echo Enter the BLE MAC address of your car (format AA:BB:CC:DD:EE:FF):
    set /p BLE_ADDR="> "
)

echo.
echo Launching 4in1_unified with BLE address: %BLE_ADDR%
echo   Camera 0, PP controller, CSV logging enabled
echo   Press Ctrl+C or close the window to stop.
echo.

"%EXE%" --ble-addr %BLE_ADDR% --camera 0 --ctrl PP --log
endlocal
