#pragma once
// ============================================================================
// logging.h  –  CSV telemetry logger
// ============================================================================
#include <fstream>
#include <string>
#include <mutex>
#include <chrono>
#include "state.h"

namespace car {

// ---------------------------------------------------------------------------
// CSV header matches Python telemetry columns exactly
// ---------------------------------------------------------------------------
constexpr const char* CSV_HEADER =
    "timestamp_ms,e_y,e_psi,v,e_y_pred,e_psi_pred,alpha_pred,"
    "delta,throttle,controller\n";

class CsvLogger {
public:
    explicit CsvLogger(const std::string& filepath = "telemetry.csv");
    ~CsvLogger();

    // Append one row; thread-safe
    void log(const State& s);

    bool is_open() const { return ofs_.is_open(); }

private:
    std::ofstream ofs_;
    std::mutex    mtx_;
    std::chrono::steady_clock::time_point t0_;

    static const char* controller_name(ControllerID id);
};

// ---------------------------------------------------------------------------
// Logging thread entry point (runs at ~20 Hz alongside BLE loop)
// ---------------------------------------------------------------------------
void logging_thread(State& state, CsvLogger& logger);

}  // namespace car
