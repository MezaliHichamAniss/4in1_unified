// ============================================================================
//  vision.cpp  –  Vision pipeline
//  MOG2 background subtraction + IMM tracker + centerline detection
//  Red/white pixel analysis for lane markings (matches Python logic exactly).
// ============================================================================
#include "vision.h"
#include "cdp_state.h"
#include "imm_tracker.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#include <chrono>
#include <thread>
#include <iostream>

namespace cdp {

// ─────────────────────────────────────────────────────────────────────────────
// Helper: detect whether a BGR pixel is a red or white lane marking
// ─────────────────────────────────────────────────────────────────────────────
static bool is_lane_pixel(const cv::Vec3b& bgr,
                           double red_thresh,
                           double white_thresh) {
    double b = bgr[0], g = bgr[1], r = bgr[2];
    // White
    if (b > white_thresh && g > white_thresh && r > white_thresh)
        return true;
    // Red (high red, low green/blue)
    if (r > red_thresh && r > 1.5 * g && r > 1.5 * b)
        return true;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Centerline detection from a single scan row
// ─────────────────────────────────────────────────────────────────────────────
struct ScanResult {
    double left_x = -1.0, right_x = -1.0;
    bool   found_left = false, found_right = false;
};

static ScanResult scan_row(const cv::Mat& frame,
                            int row,
                            int car_x,
                            double red_thresh,
                            double white_thresh) {
    ScanResult res;
    int W = frame.cols;
    if (row < 0 || row >= frame.rows) return res;

    for (int c = car_x; c >= 0; --c) {
        if (is_lane_pixel(frame.at<cv::Vec3b>(row, c), red_thresh, white_thresh)) {
            res.left_x     = static_cast<double>(c);
            res.found_left = true;
            break;
        }
    }
    for (int c = car_x; c < W; ++c) {
        if (is_lane_pixel(frame.at<cv::Vec3b>(row, c), red_thresh, white_thresh)) {
            res.right_x     = static_cast<double>(c);
            res.found_right = true;
            break;
        }
    }
    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// Centerline result struct
// ─────────────────────────────────────────────────────────────────────────────
struct CenterlineResult {
    double e_lat        = std::numeric_limits<double>::quiet_NaN();
    double e_head_deg   = std::numeric_limits<double>::quiet_NaN();
    double pp_alpha_deg = std::numeric_limits<double>::quiet_NaN();
    double kappa_ahead  = 0.0;
    bool   valid        = false;
};

static CenterlineResult detect_centerline(const cv::Mat& frame,
                                           int car_x,
                                           int car_y,
                                           double lookahead_px,
                                           const VisionConfig& cfg,
                                           bool flip) {
    CenterlineResult res;
    std::vector<double> cx_pts, cy_pts;
    int H = frame.rows;

    for (int s = 0; s < cfg.scan_rows; ++s) {
        int row = cfg.invert_y
                ? car_y + (s + 1) * cfg.scan_step_px
                : car_y - (s + 1) * cfg.scan_step_px;
        if (row < 0 || row >= H) continue;

        auto sr = scan_row(frame, row, car_x, cfg.red_thresh, cfg.white_thresh);

        if (sr.found_left && sr.found_right) {
            cx_pts.push_back(0.5 * (sr.left_x + sr.right_x));
            cy_pts.push_back(static_cast<double>(row));
        } else if (sr.found_left) {
            cx_pts.push_back(sr.left_x + frame.cols * 0.15);
            cy_pts.push_back(static_cast<double>(row));
        } else if (sr.found_right) {
            cx_pts.push_back(sr.right_x - frame.cols * 0.15);
            cy_pts.push_back(static_cast<double>(row));
        }
    }

    if (cx_pts.empty()) return res;

    double cx0 = cx_pts.front();
    double e_lat = cx0 - static_cast<double>(car_x);
    if (flip) e_lat = -e_lat;
    res.e_lat = e_lat;
    res.valid = true;

    // ── Heading error (linear fit) ────────────────────────────────────────
    if (cx_pts.size() >= 2) {
        double n = static_cast<double>(cx_pts.size());
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        for (size_t i = 0; i < cx_pts.size(); ++i) {
            sum_x  += cx_pts[i]; sum_y  += cy_pts[i];
            sum_xx += cx_pts[i] * cx_pts[i];
            sum_xy += cx_pts[i] * cy_pts[i];
        }
        double denom = n * sum_xx - sum_x * sum_x;
        double slope = (std::abs(denom) > 1e-6)
                     ? (n * sum_xy - sum_x * sum_y) / denom
                     : 0.0;
        double theta_line = std::atan2(1.0, -slope) * 180.0 / M_PI;
        res.e_head_deg = theta_line - 90.0;
        if (flip) res.e_head_deg = -res.e_head_deg;

        // Menger curvature from 3 points
        if (cx_pts.size() >= 3) {
            size_t mid = cx_pts.size() / 2;
            double x1 = cx_pts.front(), y1 = cy_pts.front();
            double x2 = cx_pts[mid],    y2 = cy_pts[mid];
            double x3 = cx_pts.back(),  y3 = cy_pts.back();
            double a   = std::hypot(x2-x1, y2-y1);
            double b   = std::hypot(x3-x2, y3-y2);
            double c   = std::hypot(x3-x1, y3-y1);
            double area2 = std::abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1));
            res.kappa_ahead = (a*b*c > 1e-6) ? area2 / (a*b*c) : 0.0;
        }
    } else {
        res.e_head_deg = 0.0;
    }

    // ── PP lookahead angle ────────────────────────────────────────────────
    if (cx_pts.size() >= 2 && lookahead_px > 0) {
        double dist_acc = 0.0;
        double lah_cx = cx_pts.front(), lah_cy = cy_pts.front();
        for (size_t i = 1; i < cx_pts.size(); ++i) {
            double seg = std::hypot(cx_pts[i]-cx_pts[i-1], cy_pts[i]-cy_pts[i-1]);
            if (dist_acc + seg >= lookahead_px) {
                double t = (lookahead_px - dist_acc) / seg;
                lah_cx = cx_pts[i-1] + t * (cx_pts[i] - cx_pts[i-1]);
                lah_cy = cy_pts[i-1] + t * (cy_pts[i] - cy_pts[i-1]);
                break;
            }
            dist_acc += seg;
            lah_cx = cx_pts[i]; lah_cy = cy_pts[i];
        }
        double dx = lah_cx - car_x;
        double dy = lah_cy - car_y;
        double alpha = std::atan2(dx, -dy) * 180.0 / M_PI;
        if (flip) alpha = -alpha;
        res.pp_alpha_deg = alpha;
    } else {
        res.pp_alpha_deg = res.e_head_deg;
    }

    return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// Vision thread
// ─────────────────────────────────────────────────────────────────────────────
void vision_thread(const VisionConfig& cfg) {
    cv::VideoCapture cap(cfg.camera_index);
    if (!cap.isOpened()) {
        std::cerr << "[vision] ERROR: cannot open camera " << cfg.camera_index << "\n";
        return;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg.frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.frame_height);
    cap.set(cv::CAP_PROP_FPS,          cfg.fps_cap);

    auto mog2 = cv::createBackgroundSubtractorMOG2(
        cfg.mog2_history, cfg.mog2_threshold, cfg.mog2_shadows);

    IMMTracker imm;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});

    double t_prev   = now_sec();
    double t_start  = t_prev;
    int    hold_cnt = 0;
    static constexpr int MAX_HOLD = 10;

    // MOG2 warm-up
    for (int w = 0; w < 30 && !g_stop; ++w) {
        cv::Mat f; cap >> f;
        if (!f.empty()) { cv::Mat fg; mog2->apply(f, fg); }
    }

    while (!g_stop) {
        cv::Mat frame; cap >> frame;
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        double t_now = now_sec();
        double dt    = t_now - t_prev;
        t_prev = t_now;
        if (dt < 1e-6) dt = 1.0 / cfg.fps_cap;

        // Background subtraction
        cv::Mat fg_mask;
        mog2->apply(frame, fg_mask);
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN,  kernel);
        cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);

        // Find largest contour (the car)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        bool   used_meas = false;
        int    meas_cx = -1, meas_cy = -1;
        double blend_x = 0, blend_y = 0, kf_vx = 0, kf_vy = 0;

        // Read live state params
        bool flip_cl; double delta_cur, speed_cur, lah_extra;
        {
            std::lock_guard<std::mutex> lk(state_lock);
            flip_cl   = g_state.flip_centerline;
            delta_cur = g_state.turn_deg;
            speed_cur = g_state.speed;
            lah_extra = g_state.pp_lookahead;
        }
        imm.feed_u(delta_cur, speed_cur);

        std::string mode = "GLOBAL";
        if (!contours.empty()) {
            auto best = std::max_element(contours.begin(), contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });
            if (cv::contourArea(*best) > 100.0) {
                cv::Moments m = cv::moments(*best);
                if (m.m00 > 0.0) {
                    meas_cx = static_cast<int>(m.m10 / m.m00);
                    meas_cy = static_cast<int>(m.m01 / m.m00);
                    used_meas = true;
                    mode = "LOCAL";
                    hold_cnt = 0;
                    auto bl = imm.correct_always(meas_cx, meas_cy, dt);
                    blend_x = bl[0]; blend_y = bl[1];
                    kf_vx = bl[2];   kf_vy  = bl[3];
                }
            }
        }

        if (!used_meas) {
            if (hold_cnt < MAX_HOLD && imm.inited()) {
                auto bl = imm.hold(dt);
                blend_x = bl[0]; blend_y = bl[1];
                kf_vx = bl[2];   kf_vy  = bl[3];
                mode = "HOLD";
            } else {
                mode = "LOST";
            }
            ++hold_cnt;
        }

        double speed_px_s = std::hypot(kf_vx, kf_vy);

        // Heading from velocity
        double theta_deg = std::numeric_limits<double>::quiet_NaN();
        std::string theta_src = "NONE";
        if (speed_px_s > 1.0) {
            theta_deg = std::atan2(kf_vy, kf_vx) * 180.0 / M_PI;
            theta_src = "KF";
        }

        // Adaptive lookahead
        double lookahead_px = cfg.lookahead_min;
        if (imm.inited()) {
            lookahead_px = std::clamp(
                cfg.lookahead_gain * speed_px_s / cfg.fps_cap + lah_extra,
                cfg.lookahead_min, cfg.lookahead_max);
        }

        // Centerline detection
        int cx_int = imm.inited() ? static_cast<int>(blend_x) : frame.cols / 2;
        int cy_int = imm.inited() ? static_cast<int>(blend_y) : frame.rows / 2;
        auto cl = detect_centerline(frame, cx_int, cy_int,
                                    lookahead_px, cfg, flip_cl);

        // Omega blending: 40% CT + 30% heading-rate (from IMM) + 30% CDP
        double omega_cdp = cl.valid
                         ? speed_px_s * cl.kappa_ahead * 180.0 / M_PI
                         : 0.0;
        double omega_blend = imm.omega_deg_s()       // 0.40*CT + 0.30*head
                           + cfg.omega_w_cdp * omega_cdp;

        // Publish vision state
        {
            std::lock_guard<std::mutex> lk(vision_lock);
            auto& v = g_vision;
            v.t_ms           = (t_now - t_start) * 1000.0;
            v.dt_ms          = dt * 1000.0;
            v.mode           = mode;
            v.used_meas      = used_meas;
            v.holding        = (mode == "HOLD");
            v.meas_cx        = meas_cx;
            v.meas_cy        = meas_cy;
            v.kf_x           = blend_x;
            v.kf_y           = blend_y;
            v.kf_vx          = kf_vx;
            v.kf_vy          = kf_vy;
            v.blend_x        = blend_x;
            v.blend_y        = blend_y;
            v.speed_px_s     = speed_px_s;
            v.theta_deg      = theta_deg;
            v.theta_src      = theta_src;
            v.mu_cv          = imm.mu_cv();
            v.mu_ct          = imm.mu_ct();
            v.omega_deg_s    = omega_blend;
            v.e_lat          = cl.e_lat;
            v.e_head_deg     = cl.e_head_deg;
            v.pp_alpha_deg   = cl.pp_alpha_deg;
            v.pp_lookahead_px = lookahead_px;
            v.kappa_ahead    = cl.kappa_ahead;
        }
    }

    cap.release();
}

}  // namespace cdp
