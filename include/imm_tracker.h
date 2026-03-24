#pragma once
// ============================================================================
//  imm_tracker.h  –  Interacting Multiple Model tracker (CV + CT models)
//  Matches Python IMMTracker class exactly.
// ============================================================================
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>

namespace cdp {

// ---------------------------------------------------------------------------
//  IMMTracker
//
//  Two models:
//    Model 0 – Constant Velocity (CV): state = [x, y, vx, vy]
//    Model 1 – Coordinated Turn  (CT): state = [x, y, vx, vy, omega]
//
//  Probability transition matrix Pi (Markov):
//    Pi[i][j] = prob of switching from model i to model j
// ---------------------------------------------------------------------------
class IMMTracker {
public:
    // Number of models
    static constexpr int N = 2;

    IMMTracker();

    // Feed control input & BLE speed before predict step
    void feed_u(double delta_deg, double speed_raw);

    // Correct with pixel measurement; also runs predict internally.
    // Returns [blend_x, blend_y, blend_vx, blend_vy].
    std::array<double, 4> correct_always(double zx, double zy, double dt);

    // Hold: predict without measurement (lost-track).
    // Returns [blend_x, blend_y, blend_vx, blend_vy].
    std::array<double, 4> hold(double dt);

    // Query model mixture probabilities
    double mu_cv() const { return mu_(0); }
    double mu_ct() const { return mu_(1); }

    // Blended yaw-rate estimate [deg/s]
    double omega_deg_s() const { return omega_est_; }

    bool inited() const { return inited_; }

    // Reset tracker
    void reset();

private:
    // Per-model state
    struct Model {
        Eigen::VectorXd x;   // state vector
        Eigen::MatrixXd P;   // covariance
        int dim = 0;
    };

    std::array<Model, N> models_;
    Eigen::Vector2d      mu_;    // mixture weights
    Eigen::Matrix2d      Pi_;    // Markov transition probabilities

    bool   inited_     = false;
    double omega_est_  = 0.0;   // blended omega [deg/s]
    double prev_theta_ = std::numeric_limits<double>::quiet_NaN();
    double delta_rad_  = 0.0;   // last control input [rad]
    double speed_px_s_ = 0.0;   // speed in px/s

    // Process / measurement noise
    Eigen::Matrix4d Q_cv_;      // CV process noise
    Eigen::Matrix<double,5,5> Q_ct_;  // CT process noise
    Eigen::Matrix2d R_;         // measurement noise (both models share H)

    void init_at(double x, double y);

    // IMM interaction step: mix states before predict
    void mix_(std::array<Eigen::VectorXd, N>& xm,
              std::array<Eigen::MatrixXd, N>& Pm) const;

    // Predict CV model
    void predict_cv_(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                     double dt) const;

    // Predict CT model (Euler-linearised)
    void predict_ct_(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                     double dt) const;

    // KF update: returns log-likelihood
    double update_(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                   const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& R,
                   const Eigen::Vector2d& z) const;

    // Fuse models into a single blended estimate
    std::array<double, 4> fuse_() const;

    // Update omega estimate from fused velocity
    void update_omega_(double vx, double vy, double dt);
};

}  // namespace cdp
