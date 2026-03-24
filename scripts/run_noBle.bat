@echo off
REM ==========================================================================
REM  run_noBle.bat  –  Launch 4in1_unified WITHOUT BLE hardware
REM  (great for testing the GUI and vision pipeline on your PC)
REM ==========================================================================
setlocal

set EXE=build\bin\Release\4in1_unified.exe
if not exist "%EXE%" set EXE=build\bin\4in1_unified.exe

if not exist "%EXE%" (
    echo [ERROR] Executable not found. Build first:
    echo   scripts\build_windows.bat
    exit /b 1
)

echo Launching 4in1_unified in --no-ble mode ...
echo   Camera 0, PP controller, no CSV logging
echo   Press Ctrl+C or close the window to stop.
echo.

"%EXE%" --no-ble --camera 0 --ctrl PP
endlocal
