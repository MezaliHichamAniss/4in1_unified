#pragma once
// ============================================================================
//  logger.h  –  CSV logger with identical column format to Python version
// ============================================================================
#include "cdp_state.h"
#include <string>
#include <fstream>
#include <mutex>
#include <ctime>

namespace cdp {

// CSV columns (order matches Python version):
// t_s, dt_ms, mode, used_meas, holding,
// meas_cx, meas_cy, kf_x, kf_y, kf_vx, kf_vy,
// blend_x, blend_y, speed_px_s,
// theta_deg, theta_src, mu_cv, mu_ct, omega_deg_s,
// e_lat, e_head_deg, pp_alpha_deg, pp_lookahead_px, kappa_ahead,
// ctrl_name, delta_cmd, turn_deg, speed,
// auto_enabled, braking, status

class CsvLogger {
public:
    CsvLogger() = default;
    ~CsvLogger() { close(); }

    // Open (or re-open) a CSV file. Returns true on success.
    bool open(const std::string& path);

    // Write one row from current global state / vision (thread-safe snap).
    void write_row(double t_s,
                   const VisionState& vis,
                   const State& st);

    // Flush and close.
    void close();

    bool is_open() const { return file_.is_open(); }

private:
    std::ofstream file_;
    std::mutex    mu_;

    void write_header_();
};

// Generate a timestamped filename like "log_20260324_013730.csv"
std::string make_log_filename();

// Global logger instance (used by control loop)
extern CsvLogger g_logger;

}  // namespace cdp
