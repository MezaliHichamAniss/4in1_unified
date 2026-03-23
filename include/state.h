#pragma once
// ============================================================================
// state.h  –  Shared vehicle state visible to all threads
// ============================================================================
#include <mutex>
#include <atomic>
#include <string>
#include <chrono>
#include <array>

namespace car {

// ---------------------------------------------------------------------------
// Active controller selector
// ---------------------------------------------------------------------------
enum class ControllerID : int {
    PP  = 0,
    LQR = 1,
    SMC = 2,
    MPC = 3
};

// ---------------------------------------------------------------------------
// All tunable gains kept in one flat struct so the GUI can bind to them
// ---------------------------------------------------------------------------
struct Gains {
    // ── Pure Pursuit ────────────────────────────────────────────────────────
    double pp_K_alpha = 1.2;
    double pp_K_lat   = 0.8;

    // ── LQR ─────────────────────────────────────────────────────────────────
    double lqr_Q11    = 10.0;   // weight on lateral error
    double lqr_Q22    = 1.0;    // weight on heading error
    double lqr_R      = 0.1;    // control effort weight

    // ── SMC ─────────────────────────────────────────────────────────────────
    double smc_lambda  = 0.5;   // sliding surface slope  (λ)
    double smc_eta     = 0.3;   // reaching gain          (η)
    double smc_phi     = 0.05;  // boundary layer width   (φ)
    double smc_K       = 1.0;   // normalisation constant  (K)

    // ── MPC ─────────────────────────────────────────────────────────────────
    int    mpc_N       = 10;    // prediction horizon
    double mpc_Q_lat   = 100.0; // output weight – lateral error
    double mpc_Q_psi   = 10.0;  // output weight – heading error
    double mpc_R_du    = 1.0;   // move-suppression weight (incremental MPC)
};

// ---------------------------------------------------------------------------
// Shared vehicle state (written by vision thread, read by control thread)
// ---------------------------------------------------------------------------
struct State {
    // Raw / filtered vision outputs
    double e_y   = 0.0;   // lateral error  [pixels or m]
    double e_psi = 0.0;   // heading error  [rad]
    double v     = 0.0;   // longitudinal speed  [BLE units]

    // Predicted (one-step IMM propagation)
    double e_y_pred   = 0.0;
    double e_psi_pred = 0.0;
    double alpha_pred = 0.0;   // look-ahead angle for PP

    // Control output (written by control thread, read by BLE thread)
    double delta = 0.0;   // steering angle [normalised BLE units]
    double throttle = 0.0;

    // Active controller
    ControllerID active = ControllerID::PP;

    // Tunable gains (protected by gains_mtx)
    Gains gains;

    // Timestamp
    std::chrono::steady_clock::time_point ts;

    // ── Synchronisation ─────────────────────────────────────────────────────
    mutable std::mutex state_mtx;   // protects e_y, e_psi, v, predictions
    mutable std::mutex gains_mtx;   // protects gains struct
    mutable std::mutex ctrl_mtx;    // protects delta, throttle
    std::atomic<bool>  running{true};
    std::atomic<bool>  vision_ready{false};
};

}  // namespace car
