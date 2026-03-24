// ============================================================================
//  gui.cpp  –  ImGui + ImPlot live tuning GUI
//  Replaces Python Matplotlib sliders.
//  Targets 60+ FPS via GLFW + OpenGL3 backend.
// ============================================================================
#include "cdp_state.h"
#include "controllers.h"
#include "logger.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <GLFW/glfw3.h>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

namespace cdp {

// ─────────────────────────────────────────────────────────────────────────────
// Helper: double slider (ImGui only has SliderFloat natively; use SliderScalar)
// ─────────────────────────────────────────────────────────────────────────────
static bool SliderDouble(const char* label, double* v, double v_min, double v_max) {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal telemetry ring buffer (GUI owns this copy; updated each frame)
// ─────────────────────────────────────────────────────────────────────────────
struct PlotBuf {
    static constexpr int CAP = 2000;
    float t[CAP]     = {};
    float e_lat[CAP] = {};
    float e_head[CAP]= {};
    float delta[CAP] = {};
    float speed[CAP] = {};
    float omega[CAP] = {};
    float mu_cv[CAP] = {};
    int   head = 0, count = 0;

    void push(const TelemetryPoint& p) {
        t[head]      = static_cast<float>(p.t);
        e_lat[head]  = static_cast<float>(p.e_lat);
        e_head[head] = static_cast<float>(p.e_head_deg);
        delta[head]  = static_cast<float>(p.delta_cmd);
        speed[head]  = static_cast<float>(p.speed);
        omega[head]  = static_cast<float>(p.omega_deg_s);
        mu_cv[head]  = static_cast<float>(p.mu_cv);
        head  = (head + 1) % CAP;
        count = std::min(count + 1, CAP);
    }
};

static PlotBuf g_plot;

// ─────────────────────────────────────────────────────────────────────────────
// Draw one telemetry line (reorders ring buffer for ImPlot)
// ─────────────────────────────────────────────────────────────────────────────
static void plot_line(const char* label,
                      const float* xs, const float* ys,
                      int n, int head,
                      ImVec4 color) {
    if (n == 0) return;
    static float tmp_x[PlotBuf::CAP];
    static float tmp_y[PlotBuf::CAP];
    int start = (n < PlotBuf::CAP) ? 0 : head;
    for (int i = 0; i < n; ++i) {
        int idx = (start + i) % PlotBuf::CAP;
        tmp_x[i] = xs[idx];
        tmp_y[i] = ys[idx];
    }
    ImPlot::SetNextLineStyle(color);
    ImPlot::PlotLine(label, tmp_x, tmp_y, n);
}

// ─────────────────────────────────────────────────────────────────────────────
// Controller parameter panel
// ─────────────────────────────────────────────────────────────────────────────
static void draw_param_panel(State& s, bool& gains_dirty) {
    ImGui::Begin("Controller Params");

    // Controller selector
    const char* ctrs[] = {"PP", "LQR", "SMC", "MPC"};
    int idx = 0;
    for (int i = 0; i < 4; ++i)
        if (s.ctrl_name == ctrs[i]) { idx = i; break; }
    if (ImGui::Combo("Controller", &idx, ctrs, 4)) {
        s.ctrl_name = ctrs[idx];
        gains_dirty = true;
    }

    ImGui::Separator();

    // ── Car model ──────────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("Car Model", ImGuiTreeNodeFlags_DefaultOpen)) {
        gains_dirty |= SliderDouble("K_eff",        &s.K_eff,        0.001, 0.2);
        gains_dirty |= SliderDouble("max_deg",       &s.max_deg,      1.0,   30.0);
        gains_dirty |= SliderDouble("steer_smooth",  &s.steer_smooth, 10.0,  200.0);
        gains_dirty |= SliderDouble("ratio",         &s.ratio,        0.5,   1.5);
        gains_dirty |= SliderDouble("c_sign",        &s.c_sign,      -1.0,   1.0);
        gains_dirty |= SliderDouble("speed_k",       &s.speed_k,      0.0,   0.2);
        gains_dirty |= SliderDouble("speed_raw",     &s.speed_raw,    0.0,   s.max_speed);
        gains_dirty |= SliderDouble("speed_step",    &s.speed_step,   10.0,  500.0);
    }

    // ── PP ─────────────────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("Pure Pursuit")) {
        gains_dirty |= SliderDouble("pp_K_alpha",    &s.pp_K_alpha,   0.0,  1.0);
        gains_dirty |= SliderDouble("pp_K_lat",      &s.pp_K_lat,     0.0,  0.1);
        gains_dirty |= SliderDouble("pp_lookahead",  &s.pp_lookahead, 0.0, 200.0);
    }

    // ── LQR ────────────────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("LQR")) {
        gains_dirty |= SliderDouble("lqr_Q_ey",   &s.lqr_Q_ey,   0.01, 100.0);
        gains_dirty |= SliderDouble("lqr_Q_epsi", &s.lqr_Q_epsi, 0.01, 100.0);
        gains_dirty |= SliderDouble("lqr_R",      &s.lqr_R,      0.01, 100.0);
        ImGui::Text("k1=%.4f  k2=%.4f", s.lqr_k1, s.lqr_k2);
    }

    // ── SMC ────────────────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("SMC")) {
        gains_dirty |= SliderDouble("smc_lambda", &s.smc_lambda, 0.0,  1.0);
        gains_dirty |= SliderDouble("smc_eta",    &s.smc_eta,    0.0, 20.0);
    }

    // ── MPC ────────────────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("MPC")) {
        int p = s.mpc_p, m = s.mpc_m;
        if (ImGui::SliderInt("mpc_p", &p, 1, 30)) { s.mpc_p = p; gains_dirty = true; }
        if (ImGui::SliderInt("mpc_m", &m, 1, p))  { s.mpc_m = m; gains_dirty = true; }
        gains_dirty |= SliderDouble("mpc_W_y",  &s.mpc_W_y,  0.01, 100.0);
        gains_dirty |= SliderDouble("mpc_W_du", &s.mpc_W_du, 0.01, 100.0);
        ImGui::Text("g_ey=%.4f  g_epsi=%.4f  g_dprev=%.4f",
                    s.mpc_g_ey, s.mpc_g_epsi, s.mpc_g_dprev);
    }

    ImGui::Separator();

    // ── Runtime controls ───────────────────────────────────────────────────
    ImGui::Checkbox("Auto", &s.auto_enabled);
    ImGui::SameLine();
    ImGui::Checkbox("Flip CL", &s.flip_centerline);
    ImGui::SameLine();
    if (ImGui::Button(s.logging ? "Stop Log" : "Start Log")) {
        if (!s.logging) {
            std::string path = make_log_filename();
            if (g_logger.open(path)) {
                s.log_path = path;
                s.logging  = true;
            }
        } else {
            s.logging = false;
            g_logger.close();
        }
    }
    if (s.logging) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4{1.0f, 0.5f, 0.0f, 1.0f},
                           "REC %s", s.log_path.c_str());
    }

    ImGui::End();
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry panel
// ─────────────────────────────────────────────────────────────────────────────
static void draw_telemetry_panel(const VisionState& vis,
                                  const State& st) {
    ImGui::Begin("Telemetry");

    ImGui::Text("Mode: %s  |  Status: %s  |  ctrl: %s",
                vis.mode.c_str(), st.status.c_str(), st.ctrl_name.c_str());
    ImGui::Text("speed=%.0f  turn=%.2f deg  delta=%.2f deg",
                st.speed, st.turn_deg, st.delta_cmd);
    ImGui::Text("e_lat=%.1f px  e_head=%.1f deg  omega=%.1f deg/s",
                std::isnan(vis.e_lat)      ? 0.0 : vis.e_lat,
                std::isnan(vis.e_head_deg) ? 0.0 : vis.e_head_deg,
                vis.omega_deg_s);
    ImGui::Text("mu_cv=%.2f  mu_ct=%.2f  speed_px=%.1f px/s",
                vis.mu_cv, vis.mu_ct, vis.speed_px_s);

    ImGui::Separator();

    int n    = g_plot.count;
    int head = g_plot.head;

    if (ImPlot::BeginPlot("Errors##plt", ImVec2(-1, 150))) {
        ImPlot::SetupAxes("t [s]", "px / deg");
        plot_line("e_lat [px]",   g_plot.t, g_plot.e_lat,  n, head,
                  ImVec4{1.0f, 0.4f, 0.0f, 1.0f});
        plot_line("e_head [deg]", g_plot.t, g_plot.e_head, n, head,
                  ImVec4{0.0f, 0.8f, 1.0f, 1.0f});
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Control##plt", ImVec2(-1, 150))) {
        ImPlot::SetupAxes("t [s]", "deg / BLE");
        plot_line("delta [deg]",  g_plot.t, g_plot.delta, n, head,
                  ImVec4{1.0f, 1.0f, 0.0f, 1.0f});
        plot_line("speed/1000",   g_plot.t, g_plot.speed, n, head,
                  ImVec4{0.0f, 1.0f, 0.0f, 1.0f});
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("IMM Weights##plt", ImVec2(-1, 120))) {
        ImPlot::SetupAxes("t [s]", "prob");
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 1.0);
        plot_line("mu_cv", g_plot.t, g_plot.mu_cv, n, head,
                  ImVec4{0.8f, 0.3f, 1.0f, 1.0f});
        ImPlot::EndPlot();
    }

    ImGui::End();
}

// ─────────────────────────────────────────────────────────────────────────────
// GUI thread entry point
// ─────────────────────────────────────────────────────────────────────────────
void gui_thread() {
    if (!glfwInit()) {
        std::cerr << "[gui] glfwInit failed\n";
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* win = glfwCreateWindow(1280, 800,
                                        "4in1_unified – Live Tuning",
                                        nullptr, nullptr);
    if (!win) {
        std::cerr << "[gui] glfwCreateWindow failed\n";
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!g_stop && !glfwWindowShouldClose(win)) {
        glfwPollEvents();

        // Pull telemetry ring buffer
        {
            std::lock_guard<std::mutex> lk(telem_lock);
            while (!g_telem.buf.empty()) {
                g_plot.push(g_telem.buf.front());
                g_telem.buf.pop_front();
            }
        }

        // Snapshot of shared state
        State       st_snap;
        VisionState vis_snap;
        {
            std::lock_guard<std::mutex> lk(state_lock);
            st_snap = g_state;
        }
        {
            std::lock_guard<std::mutex> lk(vision_lock);
            vis_snap = g_vision;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

        bool gains_dirty = false;
        draw_param_panel(st_snap, gains_dirty);
        draw_telemetry_panel(vis_snap, st_snap);

        // Write back changed params and recompute gains
        if (gains_dirty) {
            std::lock_guard<std::mutex> lk(state_lock);
            // Preserve runtime fields; only update tunable params
            g_state.K_eff          = st_snap.K_eff;
            g_state.max_deg        = st_snap.max_deg;
            g_state.steer_smooth   = st_snap.steer_smooth;
            g_state.ratio          = st_snap.ratio;
            g_state.c_sign         = st_snap.c_sign;
            g_state.speed_k        = st_snap.speed_k;
            g_state.speed_raw      = st_snap.speed_raw;
            g_state.speed_step     = st_snap.speed_step;
            g_state.ctrl_name      = st_snap.ctrl_name;
            g_state.pp_K_alpha     = st_snap.pp_K_alpha;
            g_state.pp_K_lat       = st_snap.pp_K_lat;
            g_state.pp_lookahead   = st_snap.pp_lookahead;
            g_state.lqr_Q_ey       = st_snap.lqr_Q_ey;
            g_state.lqr_Q_epsi     = st_snap.lqr_Q_epsi;
            g_state.lqr_R          = st_snap.lqr_R;
            g_state.smc_lambda     = st_snap.smc_lambda;
            g_state.smc_eta        = st_snap.smc_eta;
            g_state.mpc_p          = st_snap.mpc_p;
            g_state.mpc_m          = st_snap.mpc_m;
            g_state.mpc_W_y        = st_snap.mpc_W_y;
            g_state.mpc_W_du       = st_snap.mpc_W_du;
            g_state.auto_enabled   = st_snap.auto_enabled;
            g_state.flip_centerline = st_snap.flip_centerline;
            // Recompute controller gains with new params
            lqr_recompute(g_state);
            mpc_recompute(g_state);
        }
        // Handle logging toggle (doesn't need gains recompute)
        if (st_snap.logging != g_state.logging) {
            std::lock_guard<std::mutex> lk(state_lock);
            g_state.logging  = st_snap.logging;
            g_state.log_path = st_snap.log_path;
        }

        ImGui::Render();
        int W, H;
        glfwGetFramebufferSize(win, &W, &H);
        glViewport(0, 0, W, H);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    g_stop = true;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
}

// gui_init() is kept for API compatibility.
// All actual initialization happens inside gui_thread() itself.
void gui_init() { }

}  // namespace cdp
