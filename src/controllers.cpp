// ============================================================================
//  controllers.cpp  –  PP, LQR, SMC, MPC implementations
//  Math is identical to the Python version.
// ============================================================================
#include "controllers.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace cdp {

// ============================================================================
// Continuous Algebraic Riccati Equation (CARE) solver
// Solves: A'P + PA - P*B*R^-1*B'*P + Q = 0
// Uses the Hamiltonian eigendecomposition method (same as scipy.linalg.solve_continuous_are)
// ============================================================================
Eigen::MatrixXd solveCARE(const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R) {
    int n = A.rows();
    auto Rinv = R.inverse();

    // Build 2n x 2n Hamiltonian matrix
    Eigen::MatrixXd Ham(2 * n, 2 * n);
    Ham.topLeftCorner(n, n)     =  A;
    Ham.topRightCorner(n, n)    = -B * Rinv * B.transpose();
    Ham.bottomLeftCorner(n, n)  = -Q;
    Ham.bottomRightCorner(n, n) = -A.transpose();

    // Complex Schur decomposition
    Eigen::ComplexSchur<Eigen::MatrixXcd> schur(Ham.cast<std::complex<double>>(),
                                                 /* computeU= */ true);
    auto T = schur.matrixT();
    auto U = schur.matrixU();

    // Identify columns with stable eigenvalues (Re < 0)
    std::vector<int> stable, unstable;
    for (int i = 0; i < 2 * n; ++i) {
        if (T(i, i).real() < 0.0)
            stable.push_back(i);
        else
            unstable.push_back(i);
    }

    if ((int)stable.size() != n) {
        // System may not be stabilizable; warn and use Q as a safe fallback.
        // This should not occur for well-posed car models.
        std::cerr << "[solveCARE] WARNING: expected " << n << " stable eigenvalues, "
                  << "got " << stable.size() << ". Using Q as P fallback.\n";
        return Q;
    }

    // Build reordered U from stable columns
    Eigen::MatrixXcd Us(2 * n, n);
    for (int k = 0; k < n; ++k) {
        Us.col(k) = U.col(stable[k]);
    }

    Eigen::MatrixXcd U11 = Us.topRows(n);     // upper half
    Eigen::MatrixXcd U21 = Us.bottomRows(n);  // lower half

    // P = U21 * inv(U11)  (real part)
    Eigen::MatrixXcd P_c = U21 * U11.inverse();
    Eigen::MatrixXd  P   = P_c.real();

    // Symmetrize
    P = 0.5 * (P + P.transpose());
    return P;
}

// ============================================================================
// LQR gain recomputation
// System: [e_lat_dot, e_head_dot] = A*[e_lat, e_head] + B*delta
// A = [[0, v_ref], [0, 0]]   (simplified bicycle model)
// B = [[0], [K_eff]]
// ============================================================================
void lqr_recompute(State& s) {
    // Continuous bicycle model (pixel units) – use MatrixXd throughout
    double v  = std::max(s.speed, 10.0);  // avoid singular A at v=0
    double Ke = s.K_eff;

    Eigen::MatrixXd A(2, 2);
    A << 0.0, v,
         0.0, 0.0;

    Eigen::MatrixXd B(2, 1);
    B << 0.0, Ke;

    Eigen::MatrixXd Qmat(2, 2);
    Qmat << s.lqr_Q_ey, 0.0,
            0.0,         s.lqr_Q_epsi;

    Eigen::MatrixXd Rmat(1, 1);
    Rmat(0, 0) = s.lqr_R;

    Eigen::MatrixXd P = solveCARE(A, B, Qmat, Rmat);

    // K = R^-1 * B' * P  →  [1×1] * [1×2] * [2×2] = [1×2]
    Eigen::MatrixXd K = Rmat.inverse() * B.transpose() * P;

    s.lqr_k1 = K(0, 0);  // gain on e_lat
    s.lqr_k2 = K(0, 1);  // gain on e_head
}

// ============================================================================
// MPC gain recomputation
// Incremental MPC (MathWorks style) linearised to a precomputed gain vector.
//
// Augmented state: x_aug = [e_lat, e_head, delta_prev]'
// Incremental output: y = C * x_aug
// Control: du = delta - delta_prev
//
// Prediction matrices (P-step horizon, M-step control horizon):
//   Y = Psi * x_aug + Gamma * DU
//   where DU = [du_0, du_1, ..., du_{M-1}]'
//
// Cost: J = DU' * (Gamma'*W_Y*Gamma + W_DU*I) * DU
//           + 2 * DU' * Gamma' * W_Y * Psi * x_aug
//
// Optimal: DU* = -(Gamma'*W_Y*Gamma + W_DU*I)^-1 * Gamma'*W_Y * Psi * x_aug
// Gains come from first row of -(...)^-1 * Gamma'*W_Y * Psi
// ============================================================================
void mpc_recompute(State& s) {
    int P = s.mpc_p;  // prediction horizon
    int M = s.mpc_m;  // control horizon
    double Wy  = s.mpc_W_y;
    double Wdu = s.mpc_W_du;
    double Ke  = s.K_eff;
    double dt  = 0.05; // 20 Hz control tick

    // State-space model (discrete, Euler)
    // x_aug = [e_lat, e_head, delta_prev]  (3-D)
    // u = du (incremental delta)
    // y = [e_lat, e_head] (2-D output)
    //
    // x_aug[k+1] = A_d * x_aug[k] + B_d * du[k]
    // y[k]       = C_d * x_aug[k]

    // Simplified: e_lat_dot ≈ v * e_head, e_head_dot ≈ Ke * delta
    double v = std::max(s.speed, 10.0);

    Eigen::Matrix3d A_d = Eigen::Matrix3d::Identity();
    A_d(0, 1) = v * dt;  // e_lat += v * e_head * dt
    A_d(1, 2) = Ke * dt; // e_head += Ke * delta * dt (delta = delta_prev + du)

    // Actually for incremental: delta_prev[k+1] = delta_prev[k] + du[k]
    A_d(2, 2) = 1.0; // delta_prev integrates

    Eigen::Vector3d B_d;
    B_d << 0.0, Ke * dt, 1.0; // du updates e_head and delta_prev

    // Observation matrix: y = [e_lat, e_head]
    Eigen::Matrix<double, 2, 3> C_d;
    C_d << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0;

    // Build Psi (P*2 x 3) and Gamma (P*2 x M)
    Eigen::MatrixXd Psi  (P * 2, 3);
    Eigen::MatrixXd Gamma(P * 2, M);
    Gamma.setZero();

    // Psi[i] = C * A^{i+1}
    Eigen::Matrix3d A_pow = A_d;
    for (int i = 0; i < P; ++i) {
        Psi.block(i * 2, 0, 2, 3) = C_d * A_pow;
        A_pow = A_d * A_pow;
    }

    // Gamma[i, j] = C * A^{i-j} * B  for i >= j, else 0
    for (int j = 0; j < M; ++j) {
        Eigen::Matrix3d A_ij = Eigen::Matrix3d::Identity();
        for (int i = j; i < P; ++i) {
            Gamma.block(i * 2, j, 2, 1) = C_d * A_ij * B_d;
            A_ij = A_d * A_ij;
        }
    }

    // Weight matrices
    Eigen::MatrixXd W_Y  = Wy  * Eigen::MatrixXd::Identity(P * 2, P * 2);
    Eigen::MatrixXd W_DU = Wdu * Eigen::MatrixXd::Identity(M, M);

    // H = Gamma' * W_Y * Gamma + W_DU
    Eigen::MatrixXd H_mat = Gamma.transpose() * W_Y * Gamma + W_DU;

    // G = H^-1 * Gamma' * W_Y * Psi   (M x 3)
    Eigen::MatrixXd G = H_mat.inverse() * Gamma.transpose() * W_Y * Psi;

    // First row of G gives gains: du = -G[0,:] * x_aug
    // x_aug = [e_lat, e_head, delta_prev]
    s.mpc_g_ey    = -G(0, 0);
    s.mpc_g_epsi  = -G(0, 1);
    s.mpc_g_dprev = -G(0, 2);
}

// ============================================================================
// Individual controller implementations
// ============================================================================

double ctrl_PP(State& s,
               double e_lat,
               double e_head_deg,
               double pp_alpha_deg,
               double dt) {
    // Low-pass filter gains
    double tau_lat  = 0.15;  // time constant for lateral error filter
    double tau_head = 0.10;  // time constant for heading error filter
    double alpha_lat  = dt / (dt + tau_lat);
    double alpha_head = dt / (dt + tau_head);

    s.e_lat_filt     += alpha_lat  * (e_lat      - s.e_lat_filt);
    s.pp_alpha_filt  += alpha_head * (pp_alpha_deg - s.pp_alpha_filt);

    // Derivative terms
    double e_lat_rate   = (e_lat - s.last_e_lat) / std::max(dt, 1e-6);
    double e_head_rate  = (e_head_deg - s.last_e_head_deg) / std::max(dt, 1e-6);
    s.e_lat_rate_filt    += alpha_lat  * (e_lat_rate  - s.e_lat_rate_filt);
    s.pp_alpha_rate_filt += alpha_head * (e_head_rate  - s.pp_alpha_rate_filt);

    s.last_e_lat     = e_lat;
    s.last_e_head_deg = e_head_deg;

    // Pure Pursuit: delta = K_alpha * alpha + K_lat * e_lat
    double delta = s.pp_K_alpha * s.pp_alpha_filt
                 + s.pp_K_lat   * s.e_lat_filt;
    return delta;
}

double ctrl_LQR(const State& s, double e_lat, double e_head_deg) {
    // delta = -(k1 * e_lat + k2 * e_head_rad)
    double e_head_rad = e_head_deg * M_PI / 180.0;
    return -(s.lqr_k1 * e_lat + s.lqr_k2 * e_head_rad);
}

double ctrl_SMC(const State& s, double e_lat, double e_head_deg) {
    double e_head_rad = e_head_deg * M_PI / 180.0;
    // Sliding surface: sigma = lambda * e_lat + e_head
    double sigma = s.smc_lambda * e_lat + e_head_rad;
    // Reaching law: delta = -eta * sign(sigma)
    double sign_s = (sigma > 0.0) ? 1.0 : (sigma < 0.0) ? -1.0 : 0.0;
    return -s.smc_eta * sign_s;
}

double ctrl_MPC(const State& s, double e_lat, double e_head_deg,
                double delta_prev) {
    // Precomputed gain law: du = g_ey*e_lat + g_epsi*e_head + g_dprev*delta_prev
    double du = s.mpc_g_ey   * e_lat
              + s.mpc_g_epsi * (e_head_deg * M_PI / 180.0)
              + s.mpc_g_dprev * delta_prev;
    return delta_prev + du;  // incremental: new_delta = old_delta + du
}

// ============================================================================
// Main dispatcher
// ============================================================================
double run_controller(State& s,
                      double e_lat,
                      double e_head_deg,
                      double pp_alpha_deg,
                      double kappa_ahead,
                      double dt,
                      double delta_prev) {
    double delta = 0.0;

    if (s.ctrl_name == "PP") {
        delta = ctrl_PP(s, e_lat, e_head_deg, pp_alpha_deg, dt);
    } else if (s.ctrl_name == "LQR") {
        delta = ctrl_LQR(s, e_lat, e_head_deg);
    } else if (s.ctrl_name == "SMC") {
        delta = ctrl_SMC(s, e_lat, e_head_deg);
    } else if (s.ctrl_name == "MPC") {
        delta = ctrl_MPC(s, e_lat, e_head_deg, delta_prev);
    }

    // Apply steer sign (auto_steer_sign accounts for camera mounting)
    delta *= s.auto_steer_sign;

    return delta;
}

// ============================================================================
// Steering rate limiter + clamp
// ============================================================================
double apply_steer_limit(State& s, double target, double dt) {
    double max_change = s.steer_smooth * dt;
    double diff       = target - s.turn_deg;
    diff = std::clamp(diff, -max_change, max_change);
    s.turn_deg = std::clamp(s.turn_deg + diff, -s.max_deg, s.max_deg);
    return s.turn_deg;
}

}  // namespace cdp
