// ============================================================================
// controllers.cpp  –  LQR Riccati solver, MPC batch-QP gain builder,
//                     and the control dispatcher
//
// Formula parity with 4in1_unified.py (exact constants preserved)
// ============================================================================
#include "controllers.h"
#include <stdexcept>
#include <cmath>

namespace car {

// ============================================================================
// LQR – discrete-time algebraic Riccati equation (DARE) solver
//
// Continuous bicycle model (linearised, low-speed):
//   ẋ = A·x + B·u,   x = [e_y, e_psi]ᵀ,   u = delta
//
//   A = [0  v]      B = [0]
//       [0  0]          [v/L]
//
// where v is a nominal speed (mid-range BLE units) and L = wheelbase.
// We discretise with dt = 0.05 s (20 Hz) using Euler forward.
//
// The DARE is solved by iterating the Riccati equation until convergence:
//   P_{k+1} = Q + Aᵀ P_k A - Aᵀ P_k B (R + Bᵀ P_k B)⁻¹ Bᵀ P_k A
//
// K = (R + Bᵀ P B)⁻¹ Bᵀ P A
// ============================================================================
Eigen::Matrix<double,1,2> lqr_solve_gain(const Gains& g)
{
    // System parameters (nominal values – must match Python)
    static constexpr double v_nom = 200.0;   // nominal BLE speed units
    static constexpr double L     = 0.25;    // wheelbase [m or normalised]
    static constexpr double dt    = 0.05;    // 20 Hz

    Eigen::Matrix2d A;
    A << 1.0, v_nom * dt,
         0.0, 1.0;

    Eigen::Vector2d B;
    B << 0.0,
         (v_nom / L) * dt;

    Eigen::Matrix2d Q;
    Q << g.lqr_Q11, 0.0,
         0.0,       g.lqr_Q22;

    double R = g.lqr_R;

    // Iterate DARE (converges quickly – 200 iterations is ample)
    Eigen::Matrix2d P = Q;
    for (int iter = 0; iter < 200; ++iter) {
        Eigen::Matrix2d Pnew =
            Q + A.transpose() * P * A
            - A.transpose() * P * B
              * (1.0 / (R + (B.transpose() * P * B)(0,0)))
              * B.transpose() * P * A;
        if ((Pnew - P).norm() < 1e-10) { P = Pnew; break; }
        P = Pnew;
    }

    // K (1×2)
    double denom = R + (B.transpose() * P * B)(0, 0);
    Eigen::Matrix<double,1,2> K =
        (1.0 / denom) * B.transpose() * P * A;

    return K;
}

// ============================================================================
// MPC  –  incremental (Δu) formulation with move-suppression weight R_du
//
// State-space (augmented with integrator – incremental formulation):
//   x̃ = [e_y, e_psi]ᵀ
//   Prediction matrices Φ, Γ built for horizon N.
//
// Cost:
//   J = Σ_{i=1}^{N} x̃_i^T W x̃_i + Σ_{i=0}^{N-1} Δu_i^2 R_du
//
// Unconstrained solution:
//   ΔU* = (ΓᵀWΓ + R_du·I)⁻¹ Γᵀ W Φ x₀
// ============================================================================
MpcGains mpc_build_gains(const Gains& g)
{
    static constexpr double v_nom = 200.0;
    static constexpr double L     = 0.25;
    static constexpr double dt    = 0.05;

    const int N = g.mpc_N;

    Eigen::Matrix2d A;
    A << 1.0, v_nom * dt,
         0.0, 1.0;

    Eigen::Vector2d B;
    B << 0.0,
         (v_nom / L) * dt;

    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    W(0,0) = g.mpc_Q_lat;
    W(1,1) = g.mpc_Q_psi;

    // Build Φ  (2N×2) and Γ (2N×N)
    Eigen::MatrixXd Phi(2 * N, 2);
    Eigen::MatrixXd Gamma(2 * N, N);
    Gamma.setZero();

    Eigen::Matrix2d Ak = A;
    for (int i = 0; i < N; ++i) {
        Phi.block<2,2>(2*i, 0) = Ak;

        // Γ[i, j] = A^(i-j) B  for j <= i
        Eigen::Matrix2d Aij = Eigen::Matrix2d::Identity();
        for (int j = i; j >= 0; --j) {
            Gamma.block<2,1>(2*i, j) = Aij * B;
            if (j > 0) Aij = Aij * A;
        }
        Ak = Ak * A;
    }

    // Block-diagonal output weight W̄ (2N×2N)
    Eigen::MatrixXd Wbar = Eigen::MatrixXd::Zero(2*N, 2*N);
    for (int i = 0; i < N; ++i)
        Wbar.block<2,2>(2*i, 2*i) = W;

    // Hessian: H = ΓᵀW̄Γ + R_du·I  (N×N)
    Eigen::MatrixXd H = Gamma.transpose() * Wbar * Gamma
                        + g.mpc_R_du * Eigen::MatrixXd::Identity(N, N);

    MpcGains mg;
    mg.N                 = N;
    mg.GammaT_W_Gamma_R  = H;
    mg.GammaT_W_Phi      = Gamma.transpose() * Wbar * Phi;
    return mg;
}

// ============================================================================
// Dispatcher
// ============================================================================
double compute_control(State& s,
                       const Eigen::Matrix<double,1,2>& lqr_K,
                       MpcGains& mpc_g,
                       double& mpc_u_prev)
{
    double delta = 0.0;
    switch (s.active) {
        case ControllerID::PP:  delta = pp_compute(s);                  break;
        case ControllerID::LQR: delta = lqr_compute(s, lqr_K);         break;
        case ControllerID::SMC: delta = smc_compute(s);                 break;
        case ControllerID::MPC: delta = mpc_compute(s, mpc_g, mpc_u_prev); break;
    }
    // Clamp to [-1, 1]
    if (delta >  1.0) delta =  1.0;
    if (delta < -1.0) delta = -1.0;
    return delta;
}

}  // namespace car
