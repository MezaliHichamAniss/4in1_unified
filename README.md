# 4in1_unified – C++ Autonomous Car Controller

Exact C++ port of `4in1_unified.py` – vision, 4 controllers, BLE, 60+ FPS live tuning GUI.

---

## 🔎 Job Search Agent (Werkstudent)

This repo also includes a **standalone Python job-search agent** (separate from the car app) for finding and ranking Werkstudent roles based on a CV.

```bash
cp job_agent/config.example.json job_agent/config.json
cp job_agent/cv.example.json job_agent/cv.json
python -m job_agent.main
```

See `job_agent/README.md` for sources, filters, and scheduler details.

---

## 🚀 Quick Start (Windows)

> **5 steps from zero to running.**

### Step 1 – Clone the repo

```bat
git clone https://github.com/MezaliHichamAniss/4in1_unified.git
cd 4in1_unified
```

### Step 2 – Install prerequisites

You need **three** tools. Copy-paste each line into a **PowerShell (Admin)** window:

```powershell
# 1. Visual Studio 2022 (C++ workload) – skip if already installed
winget install Microsoft.VisualStudio.2022.Community

# 2. CMake
winget install Kitware.CMake

# 3. Git
winget install Git.Git
```

> ⚠️ After installing Visual Studio, open the **Visual Studio Installer** → modify
> → make sure **"Desktop development with C++"** is checked → Install.

### Step 3 – Install OpenCV

```powershell
# Run from the repo root (downloads ~300 MB, extracts to C:\opencv)
powershell -ExecutionPolicy Bypass -File scripts\setup_opencv_windows.ps1
```

**Or** install manually:
1. Go to <https://github.com/opencv/opencv/releases/tag/4.10.0>
2. Download `opencv-4.10.0-windows.exe`
3. Run it, extract to `C:\opencv`

### Step 4 – Build

```bat
scripts\build_windows.bat
```

This automatically finds OpenCV, detects your Visual Studio version, downloads
Eigen / GLFW / ImGui / ImPlot via CMake FetchContent, and compiles everything.

If OpenCV is somewhere other than `C:\opencv`:

```bat
scripts\build_windows.bat C:\path\to\opencv
```

### Step 5 – Run

**Without BLE hardware (test mode – no car needed):**

```bat
scripts\run_noBle.bat
```

**With your BLE car:**

```bat
scripts\run_ble.bat AA:BB:CC:DD:EE:FF
```

Replace `AA:BB:CC:DD:EE:FF` with your car's Bluetooth MAC address.

---

## 🎮 Using the GUI

When the app launches you get two windows:

| Window | What it does |
|--------|-------------|
| **Controller Params** | Sliders for every tunable gain – changes take effect immediately |
| **Telemetry** | Live plots: lateral error, heading error, delta command, IMM weights |

Key controls:

| Button / Checkbox | Effect |
|---|---|
| **Controller** combo | Switch between PP / LQR / SMC / MPC live |
| **Auto** checkbox | Enable / disable autonomous steering |
| **Flip CL** | Flip centerline direction (if car goes wrong way) |
| **Start Log** | Start writing a CSV file; click again to stop |

Close the window (or press Ctrl+C in the terminal) to stop everything safely.

---

## 🔧 Manual Build (advanced)

### Windows MSVC – Visual Studio solution

```bat
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
      -DOpenCV_DIR=C:\opencv\build\x64\vc17\lib
cmake --build build --config Release
build\bin\Release\4in1_unified.exe --no-ble
```

### Windows – NMake (from VS Developer Command Prompt)

```bat
cmake -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ^
      -DOpenCV_DIR=C:\opencv\build\x64\vc16\lib
cmake --build build
build\bin\4in1_unified.exe --no-ble
```

### Linux / macOS

```bash
sudo apt install libopencv-dev   # Ubuntu/Debian
# or: brew install opencv         # macOS

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/bin/4in1_unified --no-ble
```

### Enable real WinRT BLE (Windows only)

```bat
cmake -B build -DUSE_WINRT_BLE=ON -DOpenCV_DIR=...
cmake --build build --config Release
```

---

## 📋 All CLI Options

```
4in1_unified.exe [OPTIONS]
  --no-ble             No BLE hardware (prints packets to console)
  --camera <idx>       Camera index (default 0)
  --ble-addr <addr>    BLE MAC address  e.g. AA:BB:CC:DD:EE:FF
  --ctrl <name>        Starting controller: PP | LQR | SMC | MPC  (default PP)
  --log                Start CSV logging immediately at launch
  --help               Show this help
```

---

## 🏗 Architecture

```
┌──────────────────┐   ┌──────────────────────┐   ┌───────────────────────┐
│  vision_thread   │   │  control_loop (20 Hz) │   │  gui_thread (60+ FPS) │
│                  │   │                       │   │                       │
│  OpenCV MOG2     │──▶│  PP / LQR / SMC / MPC │──▶│  ImGui + ImPlot       │
│  IMM tracker     │   │  BLE send             │   │  Live sliders         │
│  Centerline det. │   │  CSV logging          │   │  Telemetry plots      │
└──────────────────┘   └──────────────────────┘   └───────────────────────┘
         shared: g_state, g_vision  (mutex-protected, cdp namespace)
```

### Controllers

| Name | Algorithm | Best for |
|------|-----------|---------|
| **PP** | Pure Pursuit + low-pass filter | Smooth curves |
| **LQR** | CARE Riccati (Hamiltonian eigensolver) | Optimal linear |
| **SMC** | Sliding mode, reaching law `η·sign(σ)` | Robust/noisy |
| **MPC** | Incremental MathWorks, precomputed gains | Predictive |

### IMM Tracker

Two Kalman filter models run in parallel:
- **CV** – Constant Velocity `[x, y, vx, vy]`
- **CT** – Coordinated Turn `[x, y, vx, vy, ω]`

Omega blending: `40% CT + 30% heading-rate + 30% centerline (CDP)`

---

## 📊 CSV Log Format

31 columns – identical order to the Python version:

```
t_s, dt_ms, mode, used_meas, holding,
meas_cx, meas_cy, kf_x, kf_y, kf_vx, kf_vy,
blend_x, blend_y, speed_px_s,
theta_deg, theta_src, mu_cv, mu_ct, omega_deg_s,
e_lat, e_head_deg, pp_alpha_deg, pp_lookahead_px, kappa_ahead,
ctrl_name, delta_cmd, turn_deg, speed,
auto_enabled, braking, status
```

---

## ⚡ Performance vs Python

| Metric | Python | C++ (this) |
|--------|--------|-----------|
| GUI FPS | ~20 | **60+** |
| Control latency | 50–100 ms | **<5 ms** |
| Memory | 200+ MB | **50–100 MB** |
| Startup time | 3–5 s | **<0.5 s** |

---

## ❓ Troubleshooting

| Problem | Fix |
|---------|-----|
| `OpenCV not found` | Run `scripts\setup_opencv_windows.ps1` or pass path: `build_windows.bat C:\opencv` |
| `cmake not found` | `winget install Kitware.CMake` then restart terminal |
| Camera shows black | Try `--camera 1` (or `2`) if you have multiple cameras |
| Window opens then closes | Run from a terminal so you can see error messages |
| BLE connect fails | Check MAC address; try `--no-ble` first to verify the rest works |
| Build error: `cl not found` | Open a **VS Developer Command Prompt**, or let `build_windows.bat` auto-detect VS |
