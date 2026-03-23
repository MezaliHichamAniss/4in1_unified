# 4in1_unified – Autonomous Car Controller (C++ Port)

C++ port of the Python autonomous car controller with **exact formula parity**.

## Controllers

| ID | Name | Steering Law |
|----|------|--------------|
| 0  | Pure Pursuit (PP) | `δ = -(K_α·α_pred + atan2(K_lat·e_y_pred, v+110))` |
| 1  | Linear Quadratic Regulator (LQR) | Discrete-time DARE → `δ = -K·[e_y, e_ψ]ᵀ` |
| 2  | Sliding Mode Control (SMC) | `s = e_ψ + λ·e_y`, `δ = d_eq − η·tanh(s/φ)/(v·K)` |
| 3  | Model Predictive Control (MPC) | Incremental (Δu) formulation, move-suppression weight R_du |

## Dependencies

| Library | Usage |
|---------|-------|
| OpenCV 4.x | Vision pipeline, GUI (highgui) |
| Eigen 3.4 | Matrix algebra (Riccati, MPC batch-QP) |
| nlohmann/json 3.11 | Configuration |
| Windows SDK 10.0.17763+ | WinRT BLE API |

Eigen and nlohmann/json are fetched automatically by CMake if not installed.

## Build (Windows, MSVC x64, NMake)

```bat
rmdir /s /q build_msvc
cmake -S . -B build_msvc -G "NMake Makefiles" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DOpenCV_DIR="C:\opencv\build\x64\vc16\lib"
cmake --build build_msvc
```

The binary will be `build_msvc\4in1_unified.exe`.

## Usage

```
4in1_unified.exe [camera_index]
```

- **camera_index** – OpenCV camera device index (default 0)
- ESC / close window → graceful shutdown
- Telemetry written to `telemetry.csv` in the working directory

## Project Structure

```
CMakeLists.txt
include/
  state.h        – Shared state struct + mutexes
  controllers.h  – PP, LQR, SMC, MPC (inline impls + declarations)
  vision.h       – CV thread, IMM tracker, centerline detection
  ble.h          – GATT packet builder, WinRT BLE device wrapper
  logging.h      – CSV telemetry logger
  gui.h          – Live parameter-tuning GUI (OpenCV highgui)
src/
  main.cpp       – Entry point, thread spawning, signal handlers
  controllers.cpp– LQR Riccati DARE solver, MPC batch-QP gain builder
  vision.cpp     – MOG2 background subtraction, IMM tracker, centerline
  ble.cpp        – CRC-8, GATT packet, 20 Hz BLE control loop
  logging.cpp    – CSV writer thread
  gui.cpp        – Trackbar callbacks, telemetry ring-buffer plot
```