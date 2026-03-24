# 4in1_unified – C++ Autonomous Car Controller

Exact C++ port of `4in1_unified.py` – an autonomous car controller with vision, four control algorithms, BLE, and a live tuning GUI.

## Architecture

```
┌──────────────┐   ┌──────────────────────┐   ┌────────────────────┐
│ vision_thread│   │  control_loop (20 Hz) │   │  gui_thread (60+ FPS)│
│              │   │                       │   │                    │
│ OpenCV MOG2  │──▶│ PP / LQR / SMC / MPC  │──▶│ ImGui + ImPlot     │
│ IMM tracker  │   │ BLE send              │   │ Live sliders       │
│ Centerline   │   │ CSV logging           │   │ Telemetry plots    │
└──────────────┘   └──────────────────────┘   └────────────────────┘
     shared: g_state, g_vision (mutex-protected)
```

## Components

| Component | C++ library | Python equivalent |
|-----------|-------------|------------------|
| Vision    | OpenCV 4.x  | `cv2`            |
| Tracking  | Eigen + custom IMM | `filterpy` |
| LQR CARE  | Eigen ComplexSchur | `scipy.linalg.solve_continuous_are` |
| MPC       | Eigen       | `numpy`          |
| GUI       | ImGui + ImPlot | `matplotlib`  |
| BLE       | WinRT (Windows) | `bleak`      |
| Threads   | `std::thread` | `threading`    |

## Controllers

- **PP** – Pure Pursuit with low-pass filter on `e_lat` and `pp_alpha`
- **LQR** – 2×2 Q matrix, Riccati solver via Hamiltonian eigendecomposition
- **SMC** – Sliding surface `σ = λ·e_lat + e_head`, reaching law `η·sign(σ)`
- **MPC** – Incremental MathWorks style, precomputed gain law

## IMM Tracker

Interacting Multiple Model (IMM) with:
- **CV model** – Constant velocity, state `[x, y, vx, vy]`
- **CT model** – Coordinated turn, state `[x, y, vx, vy, ω]`
- **Omega blending**: 40% CT + 30% heading-rate + 30% centerline (CDP)

## Building

### Prerequisites

- CMake ≥ 3.20
- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)
- OpenCV 4.x (install separately)
- Internet access for FetchContent (Eigen, GLFW, ImGui, ImPlot)

### Windows (MSVC)

```bat
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      -DOpenCV_DIR=C:\opencv\build\x64\vc17\lib
cmake --build build --config Release
.\build\bin\Release\4in1_unified.exe --no-ble
```

### Windows (NMake / nmake)

```bat
cmake -B build -G "NMake Makefiles" ^
      -DCMAKE_BUILD_TYPE=Release ^
      -DOpenCV_DIR=C:\opencv\build\x64\vc16\lib
cmake --build build
```

### Linux / macOS

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DOpenCV_DIR=/usr/local/lib/cmake/opencv4
cmake --build build -j$(nproc)
./build/bin/4in1_unified --no-ble
```

### Enable WinRT BLE (Windows only)

```bat
cmake -B build -DUSE_WINRT_BLE=ON ...
```

## Usage

```
Usage: 4in1_unified [OPTIONS]
  --no-ble             BLE stub mode (no hardware)
  --camera <idx>       Camera index (default 0)
  --ble-addr <addr>    BLE MAC address (e.g. AA:BB:CC:DD:EE:FF)
  --ctrl <PP|LQR|SMC|MPC>  Initial controller (default PP)
  --log                Start CSV logging immediately
```

## CSV Log Format

Identical column order to the Python version:

```
t_s, dt_ms, mode, used_meas, holding,
meas_cx, meas_cy, kf_x, kf_y, kf_vx, kf_vy,
blend_x, blend_y, speed_px_s,
theta_deg, theta_src, mu_cv, mu_ct, omega_deg_s,
e_lat, e_head_deg, pp_alpha_deg, pp_lookahead_px, kappa_ahead,
ctrl_name, delta_cmd, turn_deg, speed,
auto_enabled, braking, status
```

## Performance

| Metric | Python | C++ (this) |
|--------|--------|-----------|
| GUI FPS | ~20 | 60+ |
| Control latency | 50-100 ms | <5 ms |
| Memory | 200+ MB | 50-100 MB |
