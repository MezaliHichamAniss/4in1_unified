// ============================================================================
//  imm_tracker.cpp  –  Interacting Multiple Model tracker
//  Implements CV (constant velocity) and CT (coordinated turn) models.
//  Omega blending: 40% CT model + 30% heading-rate + 30% CDP (external).
// ============================================================================
#include "imm_tracker.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>

namespace cdp {

// ──────────────────────────────────────────────────────────────────────────
//  Constructor
// ──────────────────────────────────────────────────────────────────────────
IMMTracker::IMMTracker() {
    // Markov transition matrix
    double p_stay = 0.95;
    Pi_ << p_stay,      1.0 - p_stay,
           1.0 - p_stay, p_stay;

    // Equal initial model probabilities
    mu_ << 0.5, 0.5;

    // ── CV model setup (dim = 4: x, y, vx, vy) ──────────────────────────
    models_[0].dim = 4;
    models_[0].x   = Eigen::VectorXd::Zero(4);
    models_[0].P   = Eigen::MatrixXd::Identity(4, 4) * 100.0;

    // Process noise (q_cv = 0.5 px/s²)
    double q_cv = 0.5;
    Q_cv_ = Eigen::Matrix4d::Zero();
    Q_cv_(2, 2) = q_cv * q_cv;
    Q_cv_(3, 3) = q_cv * q_cv;

    // ── CT model setup (dim = 5: x, y, vx, vy, omega) ───────────────────
    models_[1].dim = 5;
    models_[1].x   = Eigen::VectorXd::Zero(5);
    models_[1].P   = Eigen::MatrixXd::Identity(5, 5) * 100.0;

    // Process noise (q_ct = 0.3 px/s²)
    double q_ct = 0.3;
    Q_ct_ = Eigen::Matrix<double, 5, 5>::Zero();
    Q_ct_(2, 2) = q_ct * q_ct;
    Q_ct_(3, 3) = q_ct * q_ct;
    Q_ct_(4, 4) = 0.01; // omega noise [rad/s]

    // Measurement noise (shared, r_meas = 3.0 px)
    double r_meas = 3.0;
    R_ = Eigen::Matrix2d::Identity() * (r_meas * r_meas);
}

// ──────────────────────────────────────────────────────────────────────────
//  Public API
// ──────────────────────────────────────────────────────────────────────────
void IMMTracker::feed_u(double delta_deg, double speed_raw) {
    delta_rad_  = delta_deg * M_PI / 180.0;
    speed_px_s_ = speed_raw * 0.001; // rough conversion: 1 BLE unit ≈ 0.001 px/s
}

std::array<double, 4> IMMTracker::correct_always(double zx, double zy, double dt) {
    if (!inited_) {
        init_at(zx, zy);
    }

    Eigen::Vector2d z(zx, zy);

    // 1. IMM interaction (mixing)
    std::array<Eigen::VectorXd, N> xm;
    std::array<Eigen::MatrixXd, N> Pm;
    mix_(xm, Pm);

    // 2. Predict each model
    Eigen::VectorXd x_cv = xm[0];
    Eigen::MatrixXd P_cv = Pm[0];
    predict_cv_(x_cv, P_cv, dt);

    Eigen::VectorXd x_ct = xm[1];
    Eigen::MatrixXd P_ct = Pm[1];
    predict_ct_(x_ct, P_ct, dt);

    // 3. Update (measurement correction) and compute likelihoods
    // Observation matrix H = [1 0 0 0 (0)]
    Eigen::Matrix<double, 2, 4> H_cv = Eigen::Matrix<double, 2, 4>::Zero();
    H_cv(0, 0) = 1.0;
    H_cv(1, 1) = 1.0;

    Eigen::Matrix<double, 2, 5> H_ct = Eigen::Matrix<double, 2, 5>::Zero();
    H_ct(0, 0) = 1.0;
    H_ct(1, 1) = 1.0;

    double ll_cv = update_(x_cv, P_cv, H_cv.cast<double>(), R_, z);
    double ll_ct = update_(x_ct, P_ct, H_ct.cast<double>(), R_, z);

    // 4. Update model probabilities
    // mu_new[j] = Lambda[j] * c_bar[j] / normaliser
    // c_bar[j] = sum_i Pi[i][j] * mu[i]
    Eigen::Vector2d c_bar;
    c_bar(0) = Pi_.col(0).dot(mu_);
    c_bar(1) = Pi_.col(1).dot(mu_);

    double l0 = std::exp(ll_cv) * c_bar(0);
    double l1 = std::exp(ll_ct) * c_bar(1);
    double norm = l0 + l1;
    if (norm < 1e-30) norm = 1e-30;
    mu_(0) = l0 / norm;
    mu_(1) = l1 / norm;

    // 5. Store updated models
    models_[0].x = x_cv;
    models_[0].P = P_cv;
    models_[1].x = x_ct;
    models_[1].P = P_ct;

    // 6. Fuse
    auto out = fuse_();

    // 7. Update omega estimate (40% CT, 30% heading-rate; CDP part from vision)
    //    Heading-rate omega from velocity change
    update_omega_(out[2], out[3], dt);

    return out;
}

std::array<double, 4> IMMTracker::hold(double dt) {
    if (!inited_) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    // Predict without updating; only CV model used in hold mode
    predict_cv_(models_[0].x, models_[0].P, dt);
    // CT predict too (keeps covariance growing)
    predict_ct_(models_[1].x, models_[1].P, dt);

    return fuse_();
}

void IMMTracker::reset() {
    inited_     = false;
    omega_est_  = 0.0;
    prev_theta_ = std::numeric_limits<double>::quiet_NaN();
    mu_ << 0.5, 0.5;
    models_[0].x = Eigen::VectorXd::Zero(4);
    models_[0].P = Eigen::MatrixXd::Identity(4, 4) * 100.0;
    models_[1].x = Eigen::VectorXd::Zero(5);
    models_[1].P = Eigen::MatrixXd::Identity(5, 5) * 100.0;
}

// ──────────────────────────────────────────────────────────────────────────
//  Private implementation
// ──────────────────────────────────────────────────────────────────────────
void IMMTracker::init_at(double x, double y) {
    models_[0].x = Eigen::VectorXd::Zero(4);
    models_[0].x(0) = x;
    models_[0].x(1) = y;
    models_[0].P = Eigen::MatrixXd::Identity(4, 4) * 100.0;

    models_[1].x = Eigen::VectorXd::Zero(5);
    models_[1].x(0) = x;
    models_[1].x(1) = y;
    models_[1].P = Eigen::MatrixXd::Identity(5, 5) * 100.0;

    inited_ = true;
}

void IMMTracker::mix_(std::array<Eigen::VectorXd, N>& xm,
                      std::array<Eigen::MatrixXd, N>& Pm) const {
    // Mixing probabilities: mu_ij = Pi[i][j] * mu[i] / c_bar[j]
    Eigen::Matrix2d mu_ij;
    Eigen::Vector2d c_bar;
    for (int j = 0; j < N; ++j) {
        double sum = 0.0;
        for (int i = 0; i < N; ++i) {
            mu_ij(i, j) = Pi_(i, j) * mu_(i);
            sum += mu_ij(i, j);
        }
        c_bar(j) = (sum < 1e-30) ? 1e-30 : sum;
        for (int i = 0; i < N; ++i) mu_ij(i, j) /= c_bar(j);
    }

    // Mixed CV (dim=4): mix from CV (dim=4) and CT (dim=5, truncated to 4)
    int d0 = models_[0].dim; // 4
    xm[0] = Eigen::VectorXd::Zero(d0);
    // contribution from CV
    xm[0] += mu_ij(0, 0) * models_[0].x;
    // contribution from CT (take first 4 dims)
    xm[0] += mu_ij(1, 0) * models_[1].x.head(d0);

    // Mixed CT (dim=5): mix from CV (padded to 5) and CT (dim=5)
    int d1 = models_[1].dim; // 5
    xm[1] = Eigen::VectorXd::Zero(d1);
    // contribution from CV (pad with omega=0)
    Eigen::VectorXd cv_pad = Eigen::VectorXd::Zero(d1);
    cv_pad.head(d0) = models_[0].x;
    xm[1] += mu_ij(0, 1) * cv_pad;
    // contribution from CT
    xm[1] += mu_ij(1, 1) * models_[1].x;

    // Covariances
    // CV covariance
    Pm[0] = Eigen::MatrixXd::Zero(d0, d0);
    {
        auto dx0 = models_[0].x - xm[0];
        Pm[0] += mu_ij(0, 0) * (models_[0].P + dx0 * dx0.transpose());
        auto dx1 = models_[1].x.head(d0) - xm[0];
        Pm[0] += mu_ij(1, 0) * (models_[1].P.topLeftCorner(d0, d0) +
                                  dx1 * dx1.transpose());
    }
    // CT covariance
    Pm[1] = Eigen::MatrixXd::Zero(d1, d1);
    {
        auto dx0 = cv_pad - xm[1];
        Eigen::MatrixXd P0_pad = Eigen::MatrixXd::Zero(d1, d1);
        P0_pad.topLeftCorner(d0, d0) = models_[0].P;
        Pm[1] += mu_ij(0, 1) * (P0_pad + dx0 * dx0.transpose());
        auto dx1 = models_[1].x - xm[1];
        Pm[1] += mu_ij(1, 1) * (models_[1].P + dx1 * dx1.transpose());
    }
}

void IMMTracker::predict_cv_(Eigen::VectorXd& x,
                              Eigen::MatrixXd& P,
                              double dt) const {
    // F = I + dt * F_cont
    // State: [x, y, vx, vy]
    // F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    x = F * x;
    P = F * P * F.transpose() + Q_cv_ * (dt * dt);
}

void IMMTracker::predict_ct_(Eigen::VectorXd& x,
                              Eigen::MatrixXd& P,
                              double dt) const {
    // Coordinated Turn model (Euler linearisation)
    // State: [x, y, vx, vy, omega]   (omega in rad/s)
    double omega = x(4);
    double abs_omega = std::abs(omega);

    Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();

    if (abs_omega < 1e-4) {
        // Near-straight: degrade to CV
        F(0, 2) = dt;
        F(1, 3) = dt;
    } else {
        double sin_wt  = std::sin(omega * dt);
        double cos_wt  = std::cos(omega * dt);
        double vx = x(2), vy = x(3);

        // Position update
        F(0, 2) =  sin_wt / omega;
        F(0, 3) = -(1.0 - cos_wt) / omega;
        F(1, 2) =  (1.0 - cos_wt) / omega;
        F(1, 3) =  sin_wt / omega;

        // Velocity update
        F(2, 2) =  cos_wt;
        F(2, 3) = -sin_wt;
        F(3, 2) =  sin_wt;
        F(3, 3) =  cos_wt;

        // Jacobian w.r.t. omega (partial linearization)
        F(0, 4) = (vx * cos_wt * dt - vx * sin_wt / omega +
                   vy * sin_wt * dt / omega + vy * cos_wt / omega - vy / omega) / omega;
        F(1, 4) = vx * (-sin_wt * dt / omega + (1.0 - cos_wt) / (omega * omega)) +
                  vy * cos_wt * dt / omega - vy * sin_wt / omega;
    }

    x = F * x;
    P = F * P * F.transpose() + Q_ct_ * (dt * dt);
}

double IMMTracker::update_(Eigen::VectorXd& x,
                            Eigen::MatrixXd& P,
                            const Eigen::MatrixXd& H,
                            const Eigen::MatrixXd& R,
                            const Eigen::Vector2d& z) const {
    // Innovation
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // Update
    x = x + K * y;
    int n = static_cast<int>(P.rows());
    P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;

    // Log-likelihood (Gaussian)
    double det = S.determinant();
    if (det < 1e-30) det = 1e-30;
    double mahal = y.transpose() * S.inverse() * y;
    double ll = -0.5 * (mahal + std::log(det) + 2.0 * std::log(2.0 * M_PI));
    return ll;
}

std::array<double, 4> IMMTracker::fuse_() const {
    // Blend x, y, vx, vy from both models
    double bx  = mu_(0) * models_[0].x(0) + mu_(1) * models_[1].x(0);
    double by  = mu_(0) * models_[0].x(1) + mu_(1) * models_[1].x(1);
    double bvx = mu_(0) * models_[0].x(2) + mu_(1) * models_[1].x(2);
    double bvy = mu_(0) * models_[0].x(3) + mu_(1) * models_[1].x(3);
    return {bx, by, bvx, bvy};
}

void IMMTracker::update_omega_(double vx, double vy, double dt) {
    // Heading from velocity
    double theta = std::atan2(vy, vx) * 180.0 / M_PI;

    double omega_head = 0.0;
    if (!std::isnan(prev_theta_) && dt > 1e-6) {
        double dtheta = theta - prev_theta_;
        // Wrap to [-180, 180]
        while (dtheta >  180.0) dtheta -= 360.0;
        while (dtheta < -180.0) dtheta += 360.0;
        omega_head = dtheta / dt;
    }
    prev_theta_ = theta;

    // CT model omega (rad/s → deg/s)
    double omega_ct = models_[1].x(4) * 180.0 / M_PI;

    // Blending: 40% CT + 30% heading-rate + 30% CDP (caller must add CDP part)
    // Here we store 40% CT + 30% heading-rate; vision.cpp adds 30% CDP externally
    omega_est_ = 0.40 * omega_ct + 0.30 * omega_head;
    // Note: the remaining 0.30 * omega_cdp is added in vision_thread after
    // centerline curvature is computed.
}

}  // namespace cdp
