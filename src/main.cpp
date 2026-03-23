// ============================================================================
// main.cpp  –  Entry point: spawn threads, install signal handler
// ============================================================================
#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>

#include "state.h"
#include "controllers.h"
#include "vision.h"
#include "ble.h"
#include "logging.h"
#include "gui.h"

// Global shared state (accessible from signal handler)
static car::State g_state;

static void signal_handler(int sig)
{
    std::cout << "\n[main] Signal " << sig << " – shutting down.\n";
    g_state.running.store(false, std::memory_order_relaxed);
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "4in1 Autonomous Car Controller (C++ port)\n"
              << "Controllers: PP | LQR | SMC | MPC\n"
              << "Press Ctrl-C to quit.\n\n";

    // ── Camera index from command line (default 0) ───────────────────────
    int cam_idx = 0;
    if (argc >= 2) cam_idx = std::atoi(argv[1]);

    // ── CSV logger ────────────────────────────────────────────────────────
    car::CsvLogger logger("telemetry.csv");
    if (!logger.is_open()) {
        std::cerr << "[main] WARNING: could not open telemetry.csv\n";
    }

    // ── Telemetry plot (shared between GUI thread and logging thread) ──────
    car::TelemetryPlot plot;

    // ── Launch threads ────────────────────────────────────────────────────
    std::thread t_vision([&]() {
        try { car::vision_thread(g_state, cam_idx); }
        catch (const std::exception& e) {
            std::cerr << "[vision] exception: " << e.what() << "\n";
            g_state.running.store(false);
        }
    });

    std::thread t_ble([&]() {
        try { car::ble_control_thread(g_state); }
        catch (const std::exception& e) {
            std::cerr << "[ble] exception: " << e.what() << "\n";
            g_state.running.store(false);
        }
    });

    std::thread t_log([&]() {
        try { car::logging_thread(g_state, logger); }
        catch (const std::exception& e) {
            std::cerr << "[logging] exception: " << e.what() << "\n";
        }
    });

    // GUI runs on the main thread (required by most OpenCV/Windows backends)
    car::gui_init(g_state, plot);
    car::gui_thread(g_state, plot);

    // ── Tear down ─────────────────────────────────────────────────────────
    g_state.running.store(false);
    t_vision.join();
    t_ble.join();
    t_log.join();

    std::cout << "[main] Clean exit.\n";
    return 0;
}
