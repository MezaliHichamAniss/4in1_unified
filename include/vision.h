#pragma once
// ============================================================================
//  vision.h  –  Vision pipeline interface
//  MOG2 background subtraction + IMM tracker + centerline detection
// ============================================================================
#include "cdp_state.h"
#include <string>

namespace cdp {

struct VisionConfig {
    int    camera_index   = 0;
    double fps_cap        = 30.0;
    int    frame_width    = 640;
    int    frame_height   = 480;

    // MOG2 params
    int    mog2_history   = 200;
    double mog2_threshold = 16.0;
    bool   mog2_shadows   = false;

    // Centerline detection
    int    scan_rows      = 5;     // number of horizontal scan lines below car
    int    scan_step_px   = 20;    // vertical spacing between scan lines [px]
    double red_thresh     = 50.0;  // min red channel for lane mark detection
    double white_thresh   = 180.0; // min brightness for white lane marks
    bool   invert_y       = true;  // camera y is inverted vs. world y

    // IMM params
    double imm_q_cv       = 0.5;   // CV process noise sigma [px/s²]
    double imm_q_ct       = 0.3;   // CT process noise sigma [px/s²]
    double imm_r          = 3.0;   // measurement noise sigma [px]
    double imm_pi_stay    = 0.95;  // prob of staying in same model

    // Lookahead params
    double lookahead_gain = 2.0;   // lookahead_px = lookahead_gain * speed_px_s / fps
    double lookahead_min  = 30.0;  // minimum lookahead [px]
    double lookahead_max  = 150.0; // maximum lookahead [px]

    // Omega blending weights (must sum to 1)
    double omega_w_ct     = 0.40;  // weight of CT model omega
    double omega_w_head   = 0.30;  // weight of heading-rate omega
    double omega_w_cdp    = 0.30;  // weight of centerline-derived omega
};

// ---------------------------------------------------------------------------
// Vision thread entry point.
// Runs until g_stop is set; updates g_vision and g_state under locks.
// ---------------------------------------------------------------------------
void vision_thread(const VisionConfig& cfg);

}  // namespace cdp
