#pragma once
// ============================================================================
//  controllers.h  –  PP, LQR, SMC, MPC interfaces
//  All math is identical to the Python implementation.
// ============================================================================
#include "cdp_state.h"
#include <Eigen/Dense>
#include <string>

namespace cdp {

// ---------------------------------------------------------------------------
// Continuous Algebraic Riccati Equation solver
//   Solves: A'P + PA - P*B*R^-1*B'*P + Q = 0
//   Returns P; caller builds K = R^-1 * B' * P
// ---------------------------------------------------------------------------
Eigen::MatrixXd solveCARE(const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R);

// ---------------------------------------------------------------------------
// Recompute LQR gains from current State params (writes lqr_k1, lqr_k2)
//   System: ẋ = A x + B u   (x = [e_lat, e_head], u = delta [deg])
// ---------------------------------------------------------------------------
void lqr_recompute(State& s);

// ---------------------------------------------------------------------------
// Recompute MPC gains (writes mpc_g_ey, mpc_g_epsi, mpc_g_dprev)
//   Incremental MPC linearised to feedback law:
//     delta = g_ey*e_lat + g_epsi*e_head + g_dprev*delta_prev
// ---------------------------------------------------------------------------
void mpc_recompute(State& s);

// ---------------------------------------------------------------------------
// Run one control step. Returns commanded delta [deg] (unclamped).
//
//   ctrl_name selects: "PP" | "LQR" | "SMC" | "MPC"
//   e_lat       – lateral error [px]  (positive = car left of centre)
//   e_head_deg  – heading error [deg] (positive = car heading left)
//   pp_alpha_deg – PP angle to lookahead point [deg]
//   kappa_ahead  – centerline curvature [1/px]
//   dt           – time step [s]
//   delta_prev   – previous delta command [deg]
// ---------------------------------------------------------------------------
double run_controller(State& s,
                      double e_lat,
                      double e_head_deg,
                      double pp_alpha_deg,
                      double kappa_ahead,
                      double dt,
                      double delta_prev);

// Individual controllers (exposed for unit-testing)
double ctrl_PP (State& s, double e_lat, double e_head_deg,
                double pp_alpha_deg, double dt);
double ctrl_LQR(const State& s, double e_lat, double e_head_deg);
double ctrl_SMC(const State& s, double e_lat, double e_head_deg);
double ctrl_MPC(const State& s, double e_lat, double e_head_deg,
                double delta_prev);

// ---------------------------------------------------------------------------
// Apply steering smooth rate-limit and clamp to max_deg.
// Returns new turn_deg, updates s.turn_deg in-place.
// ---------------------------------------------------------------------------
double apply_steer_limit(State& s, double target, double dt);

}  // namespace cdp
