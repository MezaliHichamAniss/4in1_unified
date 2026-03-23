// ============================================================================
// gui.cpp  –  OpenCV-highgui parameter-tuning GUI and telemetry plot
// ============================================================================
#include "gui.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>

namespace car {

// ============================================================================
// TelemetryPlot
// ============================================================================
TelemetryPlot::TelemetryPlot(int width, int height, int history)
    : width_(width), height_(height), history_(history)
{
    buf_ey_.assign(history, 0.0);
    buf_epsi_.assign(history, 0.0);
    buf_delta_.assign(history, 0.0);
}

void TelemetryPlot::push(double e_y, double e_psi, double delta)
{
    std::lock_guard<std::mutex> lk(mtx_);
    buf_ey_   [head_] = e_y;
    buf_epsi_ [head_] = e_psi;
    buf_delta_[head_] = delta;
    head_ = (head_ + 1) % history_;
    if (!full_ && head_ == 0) full_ = true;
}

void TelemetryPlot::draw_channel(cv::Mat& img,
                                  const std::vector<double>& buf,
                                  cv::Scalar colour,
                                  double y_min, double y_max,
                                  int row_y, int row_h) const
{
    int n = full_ ? history_ : head_;
    if (n < 2) return;

    double range = y_max - y_min;
    if (range < 1e-9) range = 1.0;

    auto val_to_px = [&](double v, int x_i) -> cv::Point {
        int px_x = static_cast<int>(static_cast<double>(x_i) / history_ * width_);
        int px_y = row_y + row_h - static_cast<int>((v - y_min) / range * row_h);
        px_y = std::clamp(px_y, row_y, row_y + row_h);
        return {px_x, px_y};
    };

    for (int i = 0; i < n - 1; ++i) {
        int idx0 = (head_ - n + i     + history_) % history_;
        int idx1 = (head_ - n + i + 1 + history_) % history_;
        cv::line(img, val_to_px(buf[idx0], i),
                      val_to_px(buf[idx1], i + 1),
                      colour, 1, cv::LINE_AA);
    }
}

cv::Mat TelemetryPlot::render()
{
    cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(30, 30, 30));
    int row_h = height_ / 3;

    std::lock_guard<std::mutex> lk(mtx_);
    // Row 0: lateral error
    cv::putText(img, "e_y",   {4, 14}, cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(200,200,200), 1);
    draw_channel(img, buf_ey_,    {0, 200, 0},   -200.0, 200.0, 0,       row_h);

    // Row 1: heading error
    cv::putText(img, "e_psi", {4, row_h + 14}, cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(200,200,200), 1);
    draw_channel(img, buf_epsi_,  {0, 150, 255}, -1.6,   1.6,   row_h,   row_h);

    // Row 2: steering output
    cv::putText(img, "delta", {4, 2*row_h + 14}, cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(200,200,200), 1);
    draw_channel(img, buf_delta_, {0, 80,  255},  -1.0,   1.0,   2*row_h, row_h);

    // Grid lines between rows
    cv::line(img, {0, row_h},   {width_, row_h},   {80,80,80}, 1);
    cv::line(img, {0, 2*row_h}, {width_, 2*row_h}, {80,80,80}, 1);
    return img;
}

// ============================================================================
// Trackbar callback helpers
// Each callback reads the slider position and writes the corresponding gain
// into state.gains (protected by gains_mtx).
// ============================================================================

// We need to pass the State pointer to the callback.
// OpenCV trackbar callbacks only receive an int and a void*.
struct TrackbarCtx {
    State* state = nullptr;
    // Which parameter to update
    enum class Param {
        PP_K_alpha, PP_K_lat,
        LQR_Q11, LQR_Q22, LQR_R,
        SMC_lambda, SMC_eta, SMC_phi, SMC_K,
        MPC_Q_lat, MPC_Q_psi, MPC_R_du, MPC_N,
        Controller, Throttle
    } param;
    double scale = 1.0 / GUI_SLIDER_SCALE;
};

// Global context pool (one per trackbar, max 16)
static TrackbarCtx g_ctx[16];
static int         g_ctx_n = 0;

static void on_trackbar(int val, void* userdata)
{
    auto* ctx = static_cast<TrackbarCtx*>(userdata);
    if (!ctx || !ctx->state) return;

    double fval = val * ctx->scale;

    std::lock_guard<std::mutex> lk(ctx->state->gains_mtx);
    Gains& g = ctx->state->gains;

    using P = TrackbarCtx::Param;
    switch (ctx->param) {
        case P::PP_K_alpha:  g.pp_K_alpha  = fval;           break;
        case P::PP_K_lat:    g.pp_K_lat    = fval;           break;
        case P::LQR_Q11:     g.lqr_Q11     = fval;           break;
        case P::LQR_Q22:     g.lqr_Q22     = fval;           break;
        case P::LQR_R:       g.lqr_R       = std::max(fval, 0.001); break;
        case P::SMC_lambda:  g.smc_lambda  = fval;           break;
        case P::SMC_eta:     g.smc_eta     = fval;           break;
        case P::SMC_phi:     g.smc_phi     = std::max(fval, 0.001); break;
        case P::SMC_K:       g.smc_K       = std::max(fval, 0.001); break;
        case P::MPC_Q_lat:   g.mpc_Q_lat   = fval;           break;
        case P::MPC_Q_psi:   g.mpc_Q_psi   = fval;           break;
        case P::MPC_R_du:    g.mpc_R_du    = std::max(fval, 0.001); break;
        case P::MPC_N: {
            int n = std::max(2, std::min(val, 50));
            g.mpc_N = n;
            break;
        }
        case P::Controller:
            ctx->state->active = static_cast<ControllerID>(
                std::clamp(val, 0, 3));
            break;
        case P::Throttle: {
            std::lock_guard<std::mutex> lk2(ctx->state->ctrl_mtx);
            ctx->state->throttle = fval;
            break;
        }
    }
}

static TrackbarCtx* alloc_ctx(State& state, TrackbarCtx::Param param, double scale)
{
    if (g_ctx_n >= 16) return nullptr;
    auto* ctx   = &g_ctx[g_ctx_n++];
    ctx->state  = &state;
    ctx->param  = param;
    ctx->scale  = scale;
    return ctx;
}

// Helper: create a trackbar with an initial integer position
static void add_trackbar(const std::string& label, const std::string& window,
                          int init_val, int max_val,
                          State& state, TrackbarCtx::Param param, double scale)
{
    auto* ctx = alloc_ctx(state, param, scale);
    cv::createTrackbar(label, window, nullptr, max_val, on_trackbar, ctx);
    cv::setTrackbarPos(label, window, init_val);
}

// ============================================================================
// gui_init  –  create windows and register trackbars
// ============================================================================
void gui_init(State& state, TelemetryPlot& /*plot*/)
{
    cv::namedWindow(GUI_WINDOW_CTRL, cv::WINDOW_NORMAL);
    cv::resizeWindow(GUI_WINDOW_CTRL, 600, 500);

    cv::namedWindow(GUI_WINDOW_PLOT, cv::WINDOW_NORMAL);
    cv::resizeWindow(GUI_WINDOW_PLOT, 800, 300);

    using P = TrackbarCtx::Param;
    const double S = GUI_SLIDER_SCALE;

    // Controller selector  (0=PP, 1=LQR, 2=SMC, 3=MPC)
    add_trackbar("Controller (0-3)", GUI_WINDOW_CTRL,
                  0, 3, state, P::Controller, 1.0);
    // Throttle  [0, 1]
    add_trackbar("Throttle x1000", GUI_WINDOW_CTRL,
                  0, static_cast<int>(S),
                  state, P::Throttle, 1.0 / S);

    // ── PP ──
    add_trackbar("PP K_alpha x1000", GUI_WINDOW_CTRL,
                  static_cast<int>(1.2 * S), static_cast<int>(5 * S),
                  state, P::PP_K_alpha, 1.0 / S);
    add_trackbar("PP K_lat x1000",   GUI_WINDOW_CTRL,
                  static_cast<int>(0.8 * S), static_cast<int>(5 * S),
                  state, P::PP_K_lat,   1.0 / S);

    // ── LQR ──
    add_trackbar("LQR Q11 x100", GUI_WINDOW_CTRL,
                  static_cast<int>(10.0 * 100), static_cast<int>(500 * 100),
                  state, P::LQR_Q11, 1.0 / 100.0);
    add_trackbar("LQR Q22 x100", GUI_WINDOW_CTRL,
                  static_cast<int>(1.0 * 100),  static_cast<int>(100 * 100),
                  state, P::LQR_Q22, 1.0 / 100.0);
    add_trackbar("LQR R x1000",  GUI_WINDOW_CTRL,
                  static_cast<int>(0.1 * S),    static_cast<int>(10 * S),
                  state, P::LQR_R,   1.0 / S);

    // ── SMC ──
    add_trackbar("SMC lambda x1000", GUI_WINDOW_CTRL,
                  static_cast<int>(0.5 * S),    static_cast<int>(5 * S),
                  state, P::SMC_lambda, 1.0 / S);
    add_trackbar("SMC eta x1000",    GUI_WINDOW_CTRL,
                  static_cast<int>(0.3 * S),    static_cast<int>(5 * S),
                  state, P::SMC_eta,    1.0 / S);
    add_trackbar("SMC phi x1000",    GUI_WINDOW_CTRL,
                  static_cast<int>(0.05 * S),   static_cast<int>(1 * S),
                  state, P::SMC_phi,    1.0 / S);
    add_trackbar("SMC K x1000",      GUI_WINDOW_CTRL,
                  static_cast<int>(1.0 * S),    static_cast<int>(10 * S),
                  state, P::SMC_K,      1.0 / S);

    // ── MPC ──
    add_trackbar("MPC N",         GUI_WINDOW_CTRL,
                  10, 50,  state, P::MPC_N,     1.0);
    add_trackbar("MPC Q_lat x10", GUI_WINDOW_CTRL,
                  1000, 10000, state, P::MPC_Q_lat, 0.1);
    add_trackbar("MPC Q_psi x10", GUI_WINDOW_CTRL,
                  100, 5000,   state, P::MPC_Q_psi, 0.1);
    add_trackbar("MPC R_du x1000",GUI_WINDOW_CTRL,
                  static_cast<int>(1.0 * S), static_cast<int>(100 * S),
                  state, P::MPC_R_du, 1.0 / S);

    std::cout << "[gui] Windows created. Use trackbars to tune gains.\n";
}

// ============================================================================
// gui_thread  –  main event loop (runs on main thread)
// ============================================================================
void gui_thread(State& state, TelemetryPlot& plot)
{
    std::cout << "[gui] Event loop running. Press ESC or close window to quit.\n";
    while (state.running.load(std::memory_order_relaxed)) {
        // Push latest telemetry into plot
        double e_y, e_psi, delta;
        {
            std::lock_guard<std::mutex> lk(state.state_mtx);
            e_y   = state.e_y;
            e_psi = state.e_psi;
        }
        {
            std::lock_guard<std::mutex> lk(state.ctrl_mtx);
            delta = state.delta;
        }
        plot.push(e_y, e_psi, delta);

        // Render telemetry plot
        cv::Mat plot_img = plot.render();
        cv::imshow(GUI_WINDOW_PLOT, plot_img);

        // Show a blank control panel (trackbars already drawn by OpenCV)
        cv::Mat ctrl_img(10, 600, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::imshow(GUI_WINDOW_CTRL, ctrl_img);

        int key = cv::waitKey(50);  // ~20 fps
        if (key == 27 /* ESC */ || key == 'q') {
            state.running.store(false);
            break;
        }
        // Check if windows were closed
        if (cv::getWindowProperty(GUI_WINDOW_CTRL, cv::WND_PROP_VISIBLE) < 1 ||
            cv::getWindowProperty(GUI_WINDOW_PLOT, cv::WND_PROP_VISIBLE) < 1) {
            state.running.store(false);
            break;
        }
    }
    cv::destroyAllWindows();
    std::cout << "[gui] Event loop exiting.\n";
}

}  // namespace car
