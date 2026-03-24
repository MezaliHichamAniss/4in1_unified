// ============================================================================
//  main.cpp  –  Entry point for 4in1_unified C++ port
//
//  Threading model (matches Python exactly):
//    Thread 1: vision_thread      – OpenCV pipeline (runs at camera fps)
//    Thread 2: control_loop       – 20 Hz control tick + BLE send
//    Thread 3 (main): gui_thread  – ImGui at 60+ FPS
//
//  CLI flags:
//    --no-ble            use BLE stub (print packets to stdout)
//    --camera <idx>      camera index (default 0)
//    --ble-addr <addr>   BLE device address (default empty = scan)
//    --ctrl <name>       initial controller: PP|LQR|SMC|MPC
//    --log               start logging immediately
// ============================================================================
#include "cdp_state.h"
#include "controllers.h"
#include "vision.h"
#include "ble_layer.h"
#include "logger.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>
#include <algorithm>
#include <csignal>

// Declare gui_thread (defined in gui.cpp)
namespace cdp { void gui_thread(); }

// ─────────────────────────────────────────────────────────────────────────────
// Signal handler (Ctrl-C)
// ─────────────────────────────────────────────────────────────────────────────
static void on_signal(int) { cdp::g_stop = true; }

// ─────────────────────────────────────────────────────────────────────────────
// Control loop (20 Hz)
// ─────────────────────────────────────────────────────────────────────────────
static void control_loop(cdp::BleLayer* ble) {
    using namespace cdp;
    using namespace std::chrono;

    static constexpr double CTRL_HZ  = 20.0;
    static constexpr auto   CTRL_DT  = duration<double>(1.0 / CTRL_HZ);

    double t_start = now_sec();
    double t_prev  = t_start;
    double delta_prev = 0.0;

    auto next_tick = steady_clock::now();

    while (!g_stop) {
        next_tick += duration_cast<steady_clock::duration>(CTRL_DT);
        std::this_thread::sleep_until(next_tick);

        double t_now = now_sec();
        double dt    = t_now - t_prev;
        t_prev       = t_now;

        // ── Snapshot vision state ─────────────────────────────────────────
        VisionState vis;
        {
            std::lock_guard<std::mutex> lk(vision_lock);
            vis = g_vision;
        }

        // ── Snapshot + modify state ───────────────────────────────────────
        State st;
        {
            std::lock_guard<std::mutex> lk(state_lock);
            st = g_state;
        }

        if (!st.auto_enabled) {
            // Manual / idle
            std::lock_guard<std::mutex> lk(state_lock);
            g_state.status = "MANUAL";
            continue;
        }

        // Check for valid vision data
        if (std::isnan(vis.e_lat) || std::isnan(vis.e_head_deg)) {
            std::lock_guard<std::mutex> lk(state_lock);
            g_state.status = "NO_VISION";
            // Send zero command
            if (ble->connected()) {
                ble->send({0.0, 0.0});
            }
            continue;
        }

        // ── Run controller ────────────────────────────────────────────────
        double pp_alpha = std::isnan(vis.pp_alpha_deg) ? 0.0 : vis.pp_alpha_deg;
        double delta_cmd = run_controller(
            st,
            vis.e_lat,
            vis.e_head_deg,
            pp_alpha,
            vis.kappa_ahead,
            dt,
            delta_prev);

        delta_prev = delta_cmd;

        // Apply rate limit + clamp
        double turn = apply_steer_limit(st, delta_cmd, dt);

        // ── Speed computation ─────────────────────────────────────────────
        // Python: speed drops with |turn| according to speed_k
        double b_chan = turn / st.max_deg;        // normalised turn [-1,1]
        double speed_drop = st.speed_k * std::abs(b_chan * st.speed_raw);
        double v_ref = std::max(0.0, st.speed_raw - speed_drop);

        // Ramp current speed toward v_ref
        double speed_cur;
        {
            std::lock_guard<std::mutex> lk(state_lock);
            speed_cur = g_state.speed;
        }
        double speed_err = v_ref - speed_cur;
        double ramp      = std::clamp(speed_err,
                                      -st.speed_step,
                                       st.speed_step);
        double speed_new = std::clamp(speed_cur + ramp, 0.0, st.max_speed);

        // ── Write back state ──────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(state_lock);
            g_state.delta_cmd  = delta_cmd;
            g_state.turn_deg   = turn;
            g_state.target_deg = delta_cmd;
            g_state.speed      = speed_new;
            g_state.status     = "AUTO";
            g_state.last_track_t = t_now;
            g_state.last_delta_for_speed = turn;
        }

        // ── BLE send ──────────────────────────────────────────────────────
        if (ble->connected()) {
            ble->send({speed_new, turn});
        }

        // ── CSV logging ───────────────────────────────────────────────────
        bool do_log;
        {
            std::lock_guard<std::mutex> lk(state_lock);
            do_log = g_state.logging;
        }
        if (do_log && g_logger.is_open()) {
            State st_log; VisionState vis_log;
            {
                std::lock_guard<std::mutex> lk(state_lock);
                st_log = g_state;
            }
            {
                std::lock_guard<std::mutex> lk(vision_lock);
                vis_log = g_vision;
            }
            g_logger.write_row(t_now - t_start, vis_log, st_log);
        }

        // ── Telemetry push ────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(telem_lock);
            g_telem.push({
                t_now - t_start,
                vis.e_lat,
                std::isnan(vis.e_head_deg) ? 0.0 : vis.e_head_deg,
                delta_cmd,
                speed_new,
                vis.omega_deg_s,
                vis.mu_cv
            });
        }
    }

    // Stop: send zero
    if (ble->connected()) {
        ble->send({0.0, 0.0});
        ble->disconnect();
    }
    g_logger.close();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    using namespace cdp;

    // ── Parse CLI args ────────────────────────────────────────────────────
    bool        no_ble      = false;
    int         camera_idx  = 0;
    std::string ble_addr    = "";
    std::string ctrl_init   = "PP";
    bool        log_at_start = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--no-ble") {
            no_ble = true;
        } else if (arg == "--camera" && i + 1 < argc) {
            camera_idx = std::stoi(argv[++i]);
        } else if (arg == "--ble-addr" && i + 1 < argc) {
            ble_addr = argv[++i];
        } else if (arg == "--ctrl" && i + 1 < argc) {
            ctrl_init = argv[++i];
        } else if (arg == "--log") {
            log_at_start = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout <<
                "Usage: 4in1_unified [OPTIONS]\n"
                "  --no-ble            BLE stub mode (no hardware)\n"
                "  --camera <idx>      Camera index (default 0)\n"
                "  --ble-addr <addr>   BLE MAC address (e.g. AA:BB:CC:DD:EE:FF)\n"
                "  --ctrl <PP|LQR|SMC|MPC>  Initial controller (default PP)\n"
                "  --log               Start CSV logging immediately\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << arg << "\n";
        }
    }

    // ── Apply CLI to global state ─────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(state_lock);
        g_state.ctrl_name = ctrl_init;
        g_state.no_ble    = no_ble;

        // Precompute gains for chosen controller
        lqr_recompute(g_state);
        mpc_recompute(g_state);

        if (log_at_start) {
            auto path = make_log_filename();
            if (g_logger.open(path)) {
                g_state.log_path = path;
                g_state.logging  = true;
                std::cout << "[main] logging to " << path << "\n";
            }
        }
    }

    // ── Signal handler ────────────────────────────────────────────────────
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ── BLE connection ────────────────────────────────────────────────────
    auto ble = make_ble(no_ble);
    if (!ble_addr.empty()) {
        std::cout << "[main] connecting BLE to " << ble_addr << " …\n";
        if (!ble->connect(ble_addr)) {
            std::cerr << "[main] WARNING: BLE connect failed, continuing anyway\n";
        }
    } else if (!no_ble) {
        // Placeholder address used when no explicit address given
        static constexpr const char* BLE_AUTO_SCAN_ADDR = "00:00:00:00:00:00";
        ble->connect(BLE_AUTO_SCAN_ADDR);
    } else {
        ble->connect("STUB");
    }

    // ── Vision config ─────────────────────────────────────────────────────
    VisionConfig vis_cfg;
    vis_cfg.camera_index = camera_idx;

    // ── Launch threads ────────────────────────────────────────────────────
    std::cout << "[main] launching vision thread…\n";
    std::thread t_vision([&vis_cfg]() {
        vision_thread(vis_cfg);
    });

    std::cout << "[main] launching control loop (20 Hz)…\n";
    std::thread t_ctrl([&ble]() {
        control_loop(ble.get());
    });

    std::cout << "[main] launching GUI…\n";
    gui_thread();   // runs on main thread (required by GLFW/OpenGL on macOS)

    // ── Shutdown ──────────────────────────────────────────────────────────
    g_stop = true;
    if (t_vision.joinable()) t_vision.join();
    if (t_ctrl.joinable())   t_ctrl.join();

    std::cout << "[main] shutdown complete.\n";
    return 0;
}
