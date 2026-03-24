// ============================================================================
//  logger.cpp  –  CSV logger with identical column format to Python version
// ============================================================================
#include "logger.h"
#include <ctime>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace cdp {

// Global logger instance
CsvLogger g_logger;

// ─────────────────────────────────────────────────────────────────────────────
// CSV column header (order matches Python version exactly)
// ─────────────────────────────────────────────────────────────────────────────
void CsvLogger::write_header_() {
    file_ << "t_s,dt_ms,mode,used_meas,holding,"
          << "meas_cx,meas_cy,kf_x,kf_y,kf_vx,kf_vy,"
          << "blend_x,blend_y,speed_px_s,"
          << "theta_deg,theta_src,mu_cv,mu_ct,omega_deg_s,"
          << "e_lat,e_head_deg,pp_alpha_deg,pp_lookahead_px,kappa_ahead,"
          << "ctrl_name,delta_cmd,turn_deg,speed,"
          << "auto_enabled,braking,status\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Open
// ─────────────────────────────────────────────────────────────────────────────
bool CsvLogger::open(const std::string& path) {
    std::lock_guard<std::mutex> lk(mu_);
    if (file_.is_open()) file_.close();
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) return false;
    write_header_();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write one row
// ─────────────────────────────────────────────────────────────────────────────
static std::string nan_or(double v) {
    return std::isnan(v) ? "nan" : std::to_string(v);
}

void CsvLogger::write_row(double t_s,
                           const VisionState& vis,
                           const State& st) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!file_.is_open()) return;

    file_ << std::fixed << std::setprecision(6)
          << t_s                  << ","
          << vis.dt_ms            << ","
          << vis.mode             << ","
          << (vis.used_meas ? 1 : 0) << ","
          << (vis.holding   ? 1 : 0) << ","
          << vis.meas_cx          << ","
          << vis.meas_cy          << ","
          << vis.kf_x             << ","
          << vis.kf_y             << ","
          << vis.kf_vx            << ","
          << vis.kf_vy            << ","
          << vis.blend_x          << ","
          << vis.blend_y          << ","
          << vis.speed_px_s       << ","
          << nan_or(vis.theta_deg) << ","
          << vis.theta_src        << ","
          << vis.mu_cv            << ","
          << vis.mu_ct            << ","
          << vis.omega_deg_s      << ","
          << nan_or(vis.e_lat)    << ","
          << nan_or(vis.e_head_deg) << ","
          << nan_or(vis.pp_alpha_deg) << ","
          << vis.pp_lookahead_px  << ","
          << vis.kappa_ahead      << ","
          << st.ctrl_name         << ","
          << st.delta_cmd         << ","
          << st.turn_deg          << ","
          << st.speed             << ","
          << (st.auto_enabled ? 1 : 0) << ","
          << (st.braking      ? 1 : 0) << ","
          << st.status            << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Close
// ─────────────────────────────────────────────────────────────────────────────
void CsvLogger::close() {
    std::lock_guard<std::mutex> lk(mu_);
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Timestamped filename helper
// ─────────────────────────────────────────────────────────────────────────────
std::string make_log_filename() {
    std::time_t now = std::time(nullptr);
    struct tm tm_info;
#ifdef _WIN32
    localtime_s(&tm_info, &now);
#else
    localtime_r(&now, &tm_info);
#endif
    std::ostringstream oss;
    oss << "log_"
        << std::put_time(&tm_info, "%Y%m%d_%H%M%S")
        << ".csv";
    return oss.str();
}

}  // namespace cdp
