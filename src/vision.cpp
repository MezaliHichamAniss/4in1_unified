// ============================================================================
// vision.cpp  –  Background subtraction, IMM tracker, centerline detection,
//                and the vision thread
// ============================================================================
#include "vision.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace car {

// ============================================================================
// ImmTracker
// ============================================================================
ImmTracker::ImmTracker()
{
    // Model 0: Constant Velocity (CV)
    // Model 1: Constant Turn Rate (CT) – approximated as CV with higher R
    for (int m = 0; m < NM; ++m) {
        auto& mdl = models_[m];
        mdl.mu = 1.0 / NM;

        mdl.kf.init(NS, 2, 0, CV_64F);  // state=4, meas=2

        // Transition matrix F (constant-velocity Euler)
        double dt = 0.05;
        mdl.kf.transitionMatrix = (cv::Mat_<double>(4,4) <<
            1, 0, dt, 0,
            0, 1, 0,  dt,
            0, 0, 1,  0,
            0, 0, 0,  1);

        // Measurement matrix H (observe x,y only)
        mdl.kf.measurementMatrix = (cv::Mat_<double>(2,4) <<
            1, 0, 0, 0,
            0, 1, 0, 0);

        // Process noise Q
        double q = (m == 0) ? 1.0 : 5.0;   // CT model has higher noise
        cv::setIdentity(mdl.kf.processNoiseCov,     cv::Scalar(q));

        // Measurement noise R
        double r = (m == 0) ? 4.0 : 10.0;
        cv::setIdentity(mdl.kf.measurementNoiseCov, cv::Scalar(r));

        cv::setIdentity(mdl.kf.errorCovPost, cv::Scalar(1.0));
        mdl.kf.statePost = cv::Mat::zeros(NS, 1, CV_64F);
    }
    estimate_ = {0.f, 0.f};
}

void ImmTracker::predict()
{
    interaction_step();
    for (auto& mdl : models_) mdl.kf.predict();
}

void ImmTracker::update(const cv::Point2f& meas)
{
    cv::Mat z = (cv::Mat_<double>(2,1) << meas.x, meas.y);
    for (auto& mdl : models_) {
        mdl.kf.correct(z);
    }
    fusion_step();
}

cv::Point2f ImmTracker::get_estimate() const
{
    return estimate_;
}

void ImmTracker::interaction_step()
{
    // Mixing probabilities: μ_{i|j} = π[i][j]·μ_i / c̄_j
    double c[NM] = {0.0, 0.0};
    for (int j = 0; j < NM; ++j)
        for (int i = 0; i < NM; ++i)
            c[j] += PI[i][j] * models_[i].mu;

    // Mixed initial conditions for each filter
    for (int j = 0; j < NM; ++j) {
        cv::Mat mixed_state = cv::Mat::zeros(NS, 1, CV_64F);
        for (int i = 0; i < NM; ++i) {
            double mu_ij = PI[i][j] * models_[i].mu / (c[j] + 1e-12);
            mixed_state += mu_ij * models_[i].kf.statePost;
        }
        models_[j].kf.statePost = mixed_state.clone();
    }
}

void ImmTracker::fusion_step()
{
    // Likelihood (assume Gaussian innovation; simplified scalar version)
    // Innovation = z - H·x̂  (measurement minus predicted measurement)
    double Lambda[NM];
    double sum_lam = 0.0;
    for (int m = 0; m < NM; ++m) {
        const auto& kf = models_[m].kf;
        // predicted measurement: H·x̂ = first 2 rows of statePost
        cv::Mat innov = kf.measurementMatrix * kf.statePost - kf.measurementMatrix * kf.statePre;
        double e2 = innov.dot(innov);
        Lambda[m] = std::exp(-0.5 * e2);
        sum_lam  += Lambda[m] * models_[m].mu;
    }
    // Update model probabilities
    for (int m = 0; m < NM; ++m)
        models_[m].mu = Lambda[m] * models_[m].mu / (sum_lam + 1e-12);

    // Fused estimate
    cv::Mat fused = cv::Mat::zeros(NS, 1, CV_64F);
    for (int m = 0; m < NM; ++m)
        fused += models_[m].mu * models_[m].kf.statePost;

    estimate_.x = static_cast<float>(fused.at<double>(0));
    estimate_.y = static_cast<float>(fused.at<double>(1));
}

// ============================================================================
// Centerline detector  (MOG2 + contour fitting)
// ============================================================================
CenterlineResult detect_centerline(
    const cv::Mat& frame,
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg_sub)
{
    CenterlineResult res;
    if (frame.empty()) return res;

    // 1. Apply background subtraction
    cv::Mat fg_mask;
    bg_sub->apply(frame, fg_mask);

    // 2. Morphological cleanup (kernel sizes tuned to match Python version)
    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3});
    cv::Mat kernel9 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {9,9});
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN,  kernel3);
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel9);

    // 3. Extract contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return res;

    // 4. Keep largest contour only
    auto largest = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b){
            return cv::contourArea(a) < cv::contourArea(b);
        });
    if (cv::contourArea(*largest) < 200.0) return res;  // noise threshold

    // 5. Fit a line to get heading
    cv::Vec4f line;
    cv::fitLine(*largest, line, cv::DIST_L2, 0, 0.01, 0.01);
    double vx = line[0], vy = line[1];
    double cx = line[2], cy = line[3];

    // 6. Lateral error: signed distance from image centre to the fitted line
    double img_cx = frame.cols / 2.0;
    double img_cy = frame.rows / 2.0;

    // Signed perpendicular distance: positive = car is to the right of centerline
    // Standard formula: d = (P - P0) × d̂  where d̂ = (vx, vy) is the line direction
    // Cross product (2D): (img_cx - cx)*vy - (img_cy - cy)*vx
    // Note: image y-axis points downward, so vy sign gives correct lateral sense
    res.e_y   = (img_cx - cx) * (-vy) + (img_cy - cy) * vx;
    // Heading error: angle between line direction and vertical (upward = 0)
    res.e_psi = std::atan2(vx, vy + 1e-9);
    res.valid = true;

    return res;
}

// ============================================================================
// IMM look-ahead predictor
// ============================================================================
Prediction imm_predict(ImmTracker& tracker, double e_y, double e_psi, double dt)
{
    // Push current measurement into tracker
    tracker.update({static_cast<float>(e_y), static_cast<float>(e_psi)});
    tracker.predict();

    cv::Point2f est = tracker.get_estimate();

    Prediction p;
    p.e_y   = est.x;
    p.e_psi = est.y;
    // Pure-Pursuit look-ahead angle (one step ahead)
    p.alpha = std::atan2(p.e_y, 1.0);  // look-ahead distance = 1 (normalised)
    return p;
}

// ============================================================================
// Vision thread
// ============================================================================
void vision_thread(State& state, int camera_index)
{
    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::cerr << "[vision] Cannot open camera " << camera_index << "\n";
        state.running.store(false);
        return;
    }

    // MOG2 parameters tuned to match Python version
    auto bg_sub = cv::createBackgroundSubtractorMOG2(
        /*history=*/500, /*varThreshold=*/16.0, /*detectShadows=*/false);

    ImmTracker tracker;
    constexpr double dt = 0.05;  // 20 Hz

    std::cout << "[vision] Camera " << camera_index << " opened.\n";

    while (state.running.load(std::memory_order_relaxed)) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "[vision] Frame capture failed.\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        CenterlineResult cl = detect_centerline(frame, bg_sub);

        if (cl.valid) {
            Prediction pred = imm_predict(tracker, cl.e_y, cl.e_psi, dt);

            std::lock_guard<std::mutex> lk(state.state_mtx);
            state.e_y        = cl.e_y;
            state.e_psi      = cl.e_psi;
            state.e_y_pred   = pred.e_y;
            state.e_psi_pred = pred.e_psi;
            state.alpha_pred = pred.alpha;
            state.ts         = std::chrono::steady_clock::now();
            state.vision_ready.store(true, std::memory_order_release);
        }

        // Throttle to ~20 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    cap.release();
    std::cout << "[vision] Thread exiting.\n";
}

}  // namespace car
