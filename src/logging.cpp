// ============================================================================
// logging.cpp  –  CSV telemetry writer and logging thread
// ============================================================================
#include "logging.h"
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>

namespace car {

// ============================================================================
// CsvLogger
// ============================================================================
CsvLogger::CsvLogger(const std::string& filepath)
    : t0_(std::chrono::steady_clock::now())
{
    ofs_.open(filepath, std::ios::out | std::ios::trunc);
    if (ofs_.is_open()) {
        ofs_ << CSV_HEADER;
        ofs_.flush();
        std::cout << "[logging] Telemetry -> " << filepath << "\n";
    } else {
        std::cerr << "[logging] Cannot open " << filepath << "\n";
    }
}

CsvLogger::~CsvLogger()
{
    if (ofs_.is_open()) ofs_.close();
}

void CsvLogger::log(const State& s)
{
    if (!ofs_.is_open()) return;

    // Timestamp in milliseconds
    auto now = std::chrono::steady_clock::now();
    double ts_ms = std::chrono::duration<double, std::milli>(now - t0_).count();

    double e_y, e_psi, v, ey_p, epsi_p, alpha_p, delta, throttle;
    ControllerID ctrl;
    {
        std::lock_guard<std::mutex> lk(s.state_mtx);
        e_y     = s.e_y;
        e_psi   = s.e_psi;
        v       = s.v;
        ey_p    = s.e_y_pred;
        epsi_p  = s.e_psi_pred;
        alpha_p = s.alpha_pred;
    }
    {
        std::lock_guard<std::mutex> lk(s.ctrl_mtx);
        delta    = s.delta;
        throttle = s.throttle;
    }
    ctrl = s.active;   // atomic enum – no lock needed for read

    std::lock_guard<std::mutex> lk(mtx_);
    ofs_ << std::fixed << std::setprecision(6)
         << ts_ms    << ","
         << e_y      << ","
         << e_psi    << ","
         << v        << ","
         << ey_p     << ","
         << epsi_p   << ","
         << alpha_p  << ","
         << delta    << ","
         << throttle << ","
         << controller_name(ctrl) << "\n";
}

const char* CsvLogger::controller_name(ControllerID id)
{
    switch (id) {
        case ControllerID::PP:  return "PP";
        case ControllerID::LQR: return "LQR";
        case ControllerID::SMC: return "SMC";
        case ControllerID::MPC: return "MPC";
        default:                return "UNKNOWN";
    }
}

// ============================================================================
// Logging thread
// ============================================================================
void logging_thread(State& state, CsvLogger& logger)
{
    using clock = std::chrono::steady_clock;
    constexpr auto PERIOD = std::chrono::milliseconds(50);  // 20 Hz

    std::cout << "[logging] Thread started.\n";
    while (state.running.load(std::memory_order_relaxed)) {
        auto t0 = clock::now();
        if (state.vision_ready.load(std::memory_order_acquire))
            logger.log(state);
        std::this_thread::sleep_until(t0 + PERIOD);
    }
    std::cout << "[logging] Thread exiting.\n";
}

}  // namespace car
