#pragma once
// ============================================================================
// gui.h  –  Live parameter-tuning GUI (OpenCV highgui trackbars + plot)
// ============================================================================
#include <opencv2/opencv.hpp>
#include <string>
#include "state.h"

namespace car {

// ---------------------------------------------------------------------------
// Window / trackbar names (kept in sync with Python GUI labels)
// ---------------------------------------------------------------------------
constexpr const char* GUI_WINDOW_CTRL  = "4in1 Controller Tuning";
constexpr const char* GUI_WINDOW_PLOT  = "4in1 Telemetry";

// Trackbar scaling factor: slider integer value / SLIDER_SCALE = parameter
constexpr int GUI_SLIDER_SCALE = 1000;

// ---------------------------------------------------------------------------
// Telemetry ring buffer for live plotting
// ---------------------------------------------------------------------------
class TelemetryPlot {
public:
    explicit TelemetryPlot(int width = 800, int height = 300,
                           int history = 400);

    // Push latest state; call from any thread (internally locked)
    void push(double e_y, double e_psi, double delta);

    // Render to an OpenCV Mat and return it (BGR)
    cv::Mat render();

private:
    int width_, height_, history_;
    std::vector<double> buf_ey_, buf_epsi_, buf_delta_;
    std::mutex mtx_;
    int head_ = 0;
    bool full_ = false;

    void draw_channel(cv::Mat& img, const std::vector<double>& buf,
                      cv::Scalar colour, double y_min, double y_max,
                      int row_y, int row_h) const;
};

// ---------------------------------------------------------------------------
// GUI initialisation (creates windows, registers trackbar callbacks)
// ---------------------------------------------------------------------------
void gui_init(State& state, TelemetryPlot& plot);

// ---------------------------------------------------------------------------
// GUI thread entry point (runs OpenCV event loop, refreshes plot)
// ---------------------------------------------------------------------------
void gui_thread(State& state, TelemetryPlot& plot);

}  // namespace car
