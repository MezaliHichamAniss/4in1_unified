@echo off
REM ==========================================================================
REM  build_windows.bat  –  One-click build for 4in1_unified on Windows
REM
REM  Usage:
REM    build_windows.bat                   (auto-detect OpenCV)
REM    build_windows.bat C:\opencv         (specify OpenCV root)
REM
REM  Requirements (must be installed first):
REM    1. Visual Studio 2019 or 2022 (with "Desktop development with C++" workload)
REM    2. CMake 3.20+  →  winget install Kitware.CMake
REM    3. Git          →  winget install Git.Git
REM    4. OpenCV 4.x   →  run scripts\setup_opencv_windows.ps1 first,
REM                        OR download from https://opencv.org/releases/
REM ==========================================================================
setlocal EnableDelayedExpansion

echo.
echo ========================================
echo   4in1_unified  –  Windows Build Script
echo ========================================
echo.

REM ── Detect OpenCV root (arg1 overrides auto-detect) ──────────────────────
set OPENCV_ROOT=%~1

if "%OPENCV_ROOT%"=="" (
    REM Common install locations
    for %%D in (C:\opencv C:\tools\opencv C:\local\opencv) do (
        if exist "%%D\build\x64\vc17\lib\OpenCVConfig.cmake" (
            set OPENCV_ROOT=%%D
            goto :found_opencv
        )
        if exist "%%D\build\x64\vc16\lib\OpenCVConfig.cmake" (
            set OPENCV_ROOT=%%D
            goto :found_opencv
        )
    )
    echo [ERROR] OpenCV not found. Please either:
    echo   1. Run:  powershell -ExecutionPolicy Bypass -File scripts\setup_opencv_windows.ps1
    echo   2. Or:   build_windows.bat C:\path\to\opencv
    exit /b 1
)

:found_opencv
REM Try vc17 first, fall back to vc16
set OPENCV_DIR=%OPENCV_ROOT%\build\x64\vc17\lib
if not exist "%OPENCV_DIR%\OpenCVConfig.cmake" (
    set OPENCV_DIR=%OPENCV_ROOT%\build\x64\vc16\lib
)
if not exist "%OPENCV_DIR%\OpenCVConfig.cmake" (
    echo [ERROR] Cannot find OpenCVConfig.cmake under %OPENCV_ROOT%\build\x64\
    echo         Expected: %OPENCV_DIR%\OpenCVConfig.cmake
    exit /b 1
)
echo [OK] OpenCV found: %OPENCV_DIR%

REM ── Check cmake ───────────────────────────────────────────────────────────
where cmake >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] cmake not found. Install with:  winget install Kitware.CMake
    exit /b 1
)
echo [OK] cmake: found

REM ── Check git ────────────────────────────────────────────────────────────
where git >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] git not found. Install with:  winget install Git.Git
    exit /b 1
)
echo [OK] git: found

REM ── Detect Visual Studio generator ───────────────────────────────────────
set VS_GEN=
where cl >nul 2>&1
if %errorlevel% equ 0 (
    REM Already in a VS Developer Command Prompt
    set VS_GEN=NMake Makefiles
    echo [OK] Using NMake Makefiles (VS Developer Prompt detected)
) else (
    REM Try to find VS 2022 / 2019
    if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set VS_GEN=Visual Studio 17 2022
        echo [OK] Found Visual Studio 2022 Community
    ) else if exist "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" (
        call "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set VS_GEN=Visual Studio 17 2022
        echo [OK] Found Visual Studio 2022 Professional
    ) else if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
        set VS_GEN=Visual Studio 16 2019
        echo [OK] Found Visual Studio 2019 Community
    ) else (
        echo [ERROR] Visual Studio 2019 or 2022 not found.
        echo         Please install "Desktop development with C++" workload.
        exit /b 1
    )
)

echo.
echo ── Configuring CMake ────────────────────────────────────────────────────
if "%VS_GEN%"=="NMake Makefiles" (
    cmake -B build -G "NMake Makefiles" ^
          -DCMAKE_BUILD_TYPE=Release ^
          -DOpenCV_DIR="%OPENCV_DIR%"
) else (
    cmake -B build -G "%VS_GEN%" -A x64 ^
          -DOpenCV_DIR="%OPENCV_DIR%"
)

if %errorlevel% neq 0 (
    echo.
    echo [FAILED] CMake configure failed. Check errors above.
    exit /b 1
)

echo.
echo ── Building ─────────────────────────────────────────────────────────────
cmake --build build --config Release

if %errorlevel% neq 0 (
    echo.
    echo [FAILED] Build failed. Check errors above.
    exit /b 1
)

echo.
echo ========================================
echo   BUILD SUCCEEDED
echo ========================================
echo.
echo  Executable:
echo    build\bin\Release\4in1_unified.exe
echo    (or build\bin\4in1_unified.exe for NMake)
echo.
echo  Quick launch (no hardware needed):
echo    scripts\run_noBle.bat
echo.
echo  With BLE hardware:
echo    scripts\run_ble.bat  AA:BB:CC:DD:EE:FF
echo.
endlocal
