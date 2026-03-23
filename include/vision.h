#pragma once
// ============================================================================
// vision.h  –  Background subtraction, IMM tracker, centerline detection
// ============================================================================
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <vector>
#include <array>
#include "state.h"

namespace car {

// ---------------------------------------------------------------------------
// IMM (Interacting Multiple Model) Kalman tracker
// Two motion models: constant-velocity (CV) and constant-turn-rate (CT)
// ---------------------------------------------------------------------------
struct ImmModel {
    cv::KalmanFilter kf;
    double mu;            // model probability
};

class ImmTracker {
public:
    // State vector: [x, y, vx, vy]  (4-DOF CV model)
    static constexpr int NS = 4;  // state dim
    static constexpr int NM = 2;  // number of models

    explicit ImmTracker();
    void predict();
    void update(const cv::Point2f& meas);
    cv::Point2f get_estimate() const;

private:
    std::array<ImmModel, NM> models_;
    cv::Point2f estimate_;

    // Transition probability matrix (fixed, tuned empirically)
    // π[i][j] = probability of switching from model i to model j
    static constexpr double PI[2][2] = {{0.95, 0.05},
                                         {0.05, 0.95}};
    void interaction_step();
    void fusion_step();
};

// ---------------------------------------------------------------------------
// Centerline detector
// Returns {lateral_error, heading_error} in image coords / radians
// ---------------------------------------------------------------------------
struct CenterlineResult {
    double e_y   = 0.0;   // lateral error   [pixels]
    double e_psi = 0.0;   // heading error   [rad]
    bool   valid = false;
};

CenterlineResult detect_centerline(const cv::Mat& frame,
                                   cv::Ptr<cv::BackgroundSubtractorMOG2> bg_sub);

// ---------------------------------------------------------------------------
// IMM-based look-ahead predictor
// Returns alpha_pred, ey_pred, epsi_pred
// ---------------------------------------------------------------------------
struct Prediction {
    double alpha = 0.0;
    double e_y   = 0.0;
    double e_psi = 0.0;
};

Prediction imm_predict(ImmTracker& tracker,
                        double e_y,
                        double e_psi,
                        double dt = 0.05  /* 20 Hz */);

// ---------------------------------------------------------------------------
// Vision thread entry point
// ---------------------------------------------------------------------------
void vision_thread(State& state, int camera_index = 0);

}  // namespace car
