#pragma once
// ============================================================================
//  cdp_state.h  –  Shared state structs (C++ mirror of Python 'state' dict)
// ============================================================================
#include <string>
#include <mutex>
#include <atomic>
#include <cmath>
#include <deque>

namespace cdp {

// ---------------------------------------------------------------------------
// Main car state (all tunable + runtime fields matching Python dict exactly)
// ---------------------------------------------------------------------------
struct State {
    // ── car model (tunable via sliders) ──────────────────────────────────
    double K_eff          = 0.046;    // yaw gain [1/px] from calibration
    double max_deg        = 9.0;      // max pod angle [deg]
    double steer_smooth   = 55.0;     // actuator rate limit [deg/s]
    double ratio          = 0.80;     // c-channel / b-channel ratio
    double c_sign         = -1.0;     // differential direction sign
    double speed_k        = 0.03;     // speed drop per BLE unit of |b|
    double speed_raw      = 3500.0;   // target BLE speed units
    double max_speed      = 5000.0;
    double speed_step     = 90.0;     // ramp rate [BLE units / control tick]

    // ── controller selection ──────────────────────────────────────────────
    std::string ctrl_name = "PP";     // PP | LQR | SMC | MPC

    // ── PP params ────────────────────────────────────────────────────────
    double pp_K_alpha     = 0.15;
    double pp_K_lat       = 0.014;
    double pp_lookahead   = 0.0;      // extra lookahead [px]; 0 = auto

    // ── LQR params (Q is 2×2 diagonal) ───────────────────────────────────
    double lqr_Q_ey       = 1.0;
    double lqr_Q_epsi     = 0.3;
    double lqr_R          = 1.0;

    // ── SMC params ────────────────────────────────────────────────────────
    double smc_lambda     = 0.05;
    double smc_eta        = 3.0;

    // ── MPC params ────────────────────────────────────────────────────────
    int    mpc_p          = 10;       // prediction horizon
    int    mpc_m          = 3;        // control horizon
    double mpc_W_y        = 1.0;
    double mpc_W_du       = 1.0;

    // ── runtime (written by control loop) ────────────────────────────────
    bool   auto_enabled   = true;
    double auto_steer_sign = 1.0;
    bool   flip_centerline = false;
    double speed          = 0.0;      // current BLE speed command
    double turn_deg       = 0.0;      // final pod angle command
    double target_deg     = 0.0;      // controller output before clamping
    double delta_cmd      = 0.0;      // raw controller delta [deg]
    bool   braking        = false;
    bool   logging        = false;
    std::string log_path  = "";
    std::string status    = "INIT";
    double last_track_t   = 0.0;

    // ── precomputed LQR gains (recomputed on param change) ───────────────
    double lqr_k1         = 0.0;     // gain on e_lat
    double lqr_k2         = 0.0;     // gain on e_head

    // ── precomputed MPC gains (recomputed on param change) ───────────────
    double mpc_g_ey       = 0.0;
    double mpc_g_epsi     = 0.0;
    double mpc_g_dprev    = 0.0;

    // ── PP filter states ──────────────────────────────────────────────────
    double e_lat_filt         = 0.0;
    double pp_alpha_filt      = 0.0;
    double e_lat_rate_filt    = 0.0;
    double pp_alpha_rate_filt = 0.0;
    double last_e_lat         = 0.0;
    double last_e_head_deg    = 0.0;
    double v_ref_raw          = 0.0;
    double last_delta_for_speed = 0.0;

    double auto_start_t = 0.0;

    // ── BLE mode ──────────────────────────────────────────────────────────
    bool   no_ble = false;            // true when running --no-ble
};

// ---------------------------------------------------------------------------
// Vision pipeline state (mirrors Python '_vision' dict)
// ---------------------------------------------------------------------------
struct VisionState {
    double      t_ms          = 0.0;
    double      dt_ms         = 0.0;
    std::string mode          = "GLOBAL";  // GLOBAL | LOCAL
    bool        used_meas     = false;
    bool        holding       = false;
    int         meas_cx       = -1;
    int         meas_cy       = -1;
    double      kf_x          = 0.0;
    double      kf_y          = 0.0;
    double      kf_vx         = 0.0;
    double      kf_vy         = 0.0;
    double      blend_x       = 0.0;
    double      blend_y       = 0.0;
    double      speed_px_s    = 0.0;
    double      theta_deg     = std::numeric_limits<double>::quiet_NaN();
    std::string theta_src     = "NONE";
    double      mu_cv         = 0.5;   // CV model probability
    double      mu_ct         = 0.5;   // CT model probability
    double      omega_deg_s   = 0.0;   // blended yaw rate [deg/s]
    double      e_lat         = std::numeric_limits<double>::quiet_NaN();
    double      e_head_deg    = std::numeric_limits<double>::quiet_NaN();
    double      pp_alpha_deg  = std::numeric_limits<double>::quiet_NaN();
    double      pp_lookahead_px = 0.0;
    double      kappa_ahead   = 0.0;   // centerline curvature ahead [1/px]
};

// ---------------------------------------------------------------------------
// Telemetry ring buffer (for ImPlot real-time plots)
// ---------------------------------------------------------------------------
struct TelemetryPoint {
    double t;           // seconds since start
    double e_lat;
    double e_head_deg;
    double delta_cmd;
    double speed;
    double omega_deg_s;
    double mu_cv;
};

static constexpr std::size_t TELEM_BUF_SIZE = 2000;

struct Telemetry {
    std::deque<TelemetryPoint> buf;  // protected by its own lock in main

    void push(TelemetryPoint p) {
        buf.push_back(p);
        if (buf.size() > TELEM_BUF_SIZE) buf.pop_front();
    }
};

// ---------------------------------------------------------------------------
// Global instances (defined in cdp_state.cpp)
// ---------------------------------------------------------------------------
extern std::mutex        state_lock;
extern std::mutex        vision_lock;
extern std::mutex        telem_lock;
extern State             g_state;
extern VisionState       g_vision;
extern Telemetry         g_telem;
extern std::atomic<bool> g_stop;

// Wall-clock helper: seconds since epoch
double now_sec();

}  // namespace cdp
