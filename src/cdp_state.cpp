// ============================================================================
//  cdp_state.cpp  –  Global state definitions
// ============================================================================
#include "cdp_state.h"
#include <chrono>

namespace cdp {

std::mutex        state_lock;
std::mutex        vision_lock;
std::mutex        telem_lock;
State             g_state;
VisionState       g_vision;
Telemetry         g_telem;
std::atomic<bool> g_stop{false};

double now_sec() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace cdp
