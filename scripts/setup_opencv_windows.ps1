# ============================================================================
#  setup_opencv_windows.ps1  –  Download and install OpenCV 4.x on Windows
#
#  Run from the repo root:
#    powershell -ExecutionPolicy Bypass -File scripts\setup_opencv_windows.ps1
#
#  This script:
#    1. Downloads OpenCV 4.10.0 installer from the official GitHub release
#    2. Extracts to C:\opencv
#    3. Adds C:\opencv\build\x64\vc17\bin to PATH (current session + user env)
# ============================================================================
param(
    [string]$InstallDir = "C:\opencv",
    [string]$Version    = "4.10.0"
)

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  OpenCV $Version Windows Setup" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Already installed?
$config = Join-Path $InstallDir "build\x64\vc17\lib\OpenCVConfig.cmake"
if (Test-Path $config) {
    Write-Host "[OK] OpenCV already found at $InstallDir" -ForegroundColor Green
    Write-Host "     Skipping download."
    Write-Host ""
    Write-Host "To build the project, run:" -ForegroundColor Yellow
    Write-Host "  scripts\build_windows.bat $InstallDir"
    exit 0
}

# Download
$exe = "$env:TEMP\opencv-$Version-windows.exe"
$url = "https://github.com/opencv/opencv/releases/download/$Version/opencv-$Version-windows.exe"

Write-Host "[1/3] Downloading OpenCV $Version (~300 MB)..." -ForegroundColor Yellow
Write-Host "      URL: $url"

try {
    $wc = New-Object System.Net.WebClient
    $wc.DownloadFile($url, $exe)
    Write-Host "[OK] Downloaded to $exe" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Download failed: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Manual steps:"
    Write-Host "  1. Go to: https://github.com/opencv/opencv/releases/tag/$Version"
    Write-Host "  2. Download: opencv-$Version-windows.exe"
    Write-Host "  3. Run it and extract to C:\opencv"
    exit 1
}

# Extract (the .exe is a 7-zip self-extractor)
Write-Host ""
Write-Host "[2/3] Extracting to $InstallDir..." -ForegroundColor Yellow

# The OpenCV self-extractor GUI is unavoidable; it extracts to the current dir
# by default. We use the -o flag supported by 7-zip self-extractors.
$parentDir = Split-Path $InstallDir -Parent
if (-not (Test-Path $parentDir)) { New-Item -ItemType Directory $parentDir | Out-Null }

$proc = Start-Process -FilePath $exe `
    -ArgumentList "-o`"$parentDir`"", "-y" `
    -Wait -PassThru -NoNewWindow

if ($proc.ExitCode -ne 0) {
    Write-Host "[ERROR] Extraction failed (exit code $($proc.ExitCode))" -ForegroundColor Red
    Write-Host "        Try running $exe manually and extracting to $parentDir"
    exit 1
}

# The extractor creates an 'opencv' subfolder inside $parentDir
$extracted = Join-Path $parentDir "opencv"
if ($extracted -ne $InstallDir) {
    if (Test-Path $InstallDir) { Remove-Item $InstallDir -Recurse -Force }
    Rename-Item $extracted $InstallDir
}

if (-not (Test-Path $config)) {
    Write-Host "[ERROR] OpenCV extracted but config not found at $config" -ForegroundColor Red
    exit 1
}
Write-Host "[OK] OpenCV extracted to $InstallDir" -ForegroundColor Green

# Add to PATH
Write-Host ""
Write-Host "[3/3] Adding OpenCV DLLs to PATH..." -ForegroundColor Yellow

$binDir = "$InstallDir\build\x64\vc17\bin"
$userPath = [System.Environment]::GetEnvironmentVariable("PATH", "User")
if ($userPath -notlike "*$binDir*") {
    [System.Environment]::SetEnvironmentVariable("PATH", "$binDir;$userPath", "User")
    $env:PATH = "$binDir;$env:PATH"
    Write-Host "[OK] Added to user PATH: $binDir" -ForegroundColor Green
} else {
    Write-Host "[OK] Already in PATH: $binDir" -ForegroundColor Green
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Green
Write-Host "  OpenCV $Version installed successfully!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green
Write-Host ""
Write-Host "Next step – build the project:" -ForegroundColor Yellow
Write-Host "  scripts\build_windows.bat $InstallDir"
Write-Host ""
