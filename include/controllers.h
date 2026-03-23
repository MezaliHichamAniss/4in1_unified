#pragma once
// ============================================================================
// controllers.h  –  PP, LQR, SMC, MPC  (declarations + lightweight helpers)
//
// Formula parity with 4in1_unified.py
//   PP  : delta = -(K_alpha·alpha_pred + atan2(K_lat·ey_pred, v+110))
//   LQR : discrete-time Riccati → K → delta = -K * [e_y, e_psi]^T
//   SMC : s = e_psi + λ·e_y,   delta = d_eq - η·tanh(s/φ)/(v·K)
//   MPC : incremental (delta-u) formulation, move-suppression weight R_du
// ============================================================================
#include <cmath>
#include <Eigen/Dense>
#include "state.h"

namespace car {

// ============================================================================
// 1. Pure Pursuit
// ============================================================================
//  delta = -(K_alpha * alpha_pred + atan2(K_lat * ey_pred, v + 110))
inline double pp_compute(const State& s)
{
    const Gains& g = s.gains;
    return -(g.pp_K_alpha * s.alpha_pred
             + std::atan2(g.pp_K_lat * s.e_y_pred, s.v + 110.0));
}

// ============================================================================
// 2. LQR  (gain K computed offline via discrete-time Riccati)
// ============================================================================
// Gain matrix K is solved in controllers.cpp and cached here.
Eigen::Matrix<double,1,2> lqr_solve_gain(const Gains& g);

inline double lqr_compute(const State& s,
                           const Eigen::Matrix<double,1,2>& K)
{
    Eigen::Vector2d x(s.e_y_pred, s.e_psi_pred);
    return -(K * x)(0);
}

// ============================================================================
// 3. Sliding-Mode Control
// ============================================================================
//  s     = e_psi + λ·e_y
//  d_eq  = equivalent (linear) control  → simplified as 0 (pure SMC)
//  delta = d_eq - η·tanh(s/φ) / (v·K)
inline double smc_compute(const State& s)
{
    const Gains& g = s.gains;
    const double sigma = s.e_psi_pred + g.smc_lambda * s.e_y_pred;
    const double d_eq  = 0.0;   // equivalent control (add model term if needed)
    const double denom = (std::fabs(s.v) + 1e-6) * g.smc_K;
    return d_eq - g.smc_eta * std::tanh(sigma / g.smc_phi) / denom;
}

// ============================================================================
// 4. MPC  (incremental, move-suppression weight)
// ============================================================================
// Build Φ / Γ / W matrices (done once in controllers.cpp) then solve:
//   ΔU* = (ΓᵀWΓ + R_du·I)⁻¹ Γᵀ W Φ x₀
//   u(k) = u(k-1) + ΔU*(0)
struct MpcGains {
    Eigen::MatrixXd GammaT_W_Gamma_R;   // (ΓᵀWΓ + R·I)
    Eigen::MatrixXd GammaT_W_Phi;       // ΓᵀWΦ
    int N = 10;
};

MpcGains mpc_build_gains(const Gains& g);

inline double mpc_compute(const State& s,
                           const MpcGains& mg,
                           double& u_prev)
{
    Eigen::Vector2d x0(s.e_y_pred, s.e_psi_pred);
    Eigen::VectorXd du_star = mg.GammaT_W_Gamma_R.llt()
                                  .solve(mg.GammaT_W_Phi * x0);
    u_prev += du_star(0);
    return u_prev;
}

// ============================================================================
// Dispatcher: select active controller and return delta
// ============================================================================
double compute_control(State& s,
                       const Eigen::Matrix<double,1,2>& lqr_K,
                       MpcGains& mpc_g,
                       double& mpc_u_prev);

}  // namespace car
