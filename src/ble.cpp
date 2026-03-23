// ============================================================================
// ble.cpp  –  GATT packet builder, Windows WinRT BLE device wrapper,
//             and the 20 Hz BLE control loop
// ============================================================================
#include "ble.h"
#include "controllers.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#include <winrt/Windows.Foundation.Collections.h>
using namespace winrt;
using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Devices::Enumeration;
#endif

namespace car {

// ============================================================================
// CRC-8 Dallas / Maxim  (polynomial 0x31, init 0x00)
// Matches Python: crcmod.predefined.mkCrcFun('crc-8')
// ============================================================================
uint8_t crc8_dallas(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80)
                crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================================================================
// GATT packet builder
//   Layout: [tag(1), value_lo(1), value_hi(1), crc8(1)]  – 4 bytes total
// ============================================================================
std::vector<uint8_t> build_gatt_packet(PacketTag tag, int16_t value)
{
    std::vector<uint8_t> pkt(4);
    pkt[0] = static_cast<uint8_t>(tag);
    pkt[1] = static_cast<uint8_t>(value & 0xFF);          // little-endian low
    pkt[2] = static_cast<uint8_t>((value >> 8) & 0xFF);   // little-endian high
    pkt[3] = crc8_dallas(pkt.data(), 3);
    return pkt;
}

// ============================================================================
// BleDevice  (Windows WinRT implementation)
// ============================================================================
#ifdef _WIN32

BleDevice::~BleDevice()
{
    disconnect();
}

bool BleDevice::connect(const std::wstring& device_name)
{
    try {
        winrt::init_apartment();

        // Enumerate BLE devices
        std::wstring selector = BluetoothLEDevice::GetDeviceSelector();
        auto devices = DeviceInformation::FindAllAsync(selector).get();

        winrt::Windows::Devices::Enumeration::DeviceInformation target{nullptr};
        for (auto&& dev : devices) {
            if (dev.Name() == device_name) {
                target = dev;
                break;
            }
        }
        if (!target) {
            std::wcerr << L"[ble] Device not found: " << device_name << L"\n";
            return false;
        }

        ble_dev_ = BluetoothLEDevice::FromIdAsync(target.Id()).get();
        if (!ble_dev_) {
            std::cerr << "[ble] Failed to create BLE device handle.\n";
            return false;
        }

        // Get GATT services
        auto svc_result = ble_dev_.GetGattServicesAsync().get();
        if (svc_result.Status() != GattCommunicationStatus::Success) {
            std::cerr << "[ble] Failed to enumerate GATT services.\n";
            return false;
        }

        // Find our service by UUID
        winrt::guid svc_uuid = winrt::guid(BLE_SERVICE_UUID);
        GattDeviceService target_svc{nullptr};
        for (auto&& svc : svc_result.Services()) {
            if (svc.Uuid() == svc_uuid) { target_svc = svc; break; }
        }
        if (!target_svc) {
            std::cerr << "[ble] Service UUID not found on device.\n";
            return false;
        }

        // Steering characteristic
        auto char_result_steer = target_svc
            .GetCharacteristicsForUuidAsync(winrt::guid(BLE_CHAR_UUID_STEER)).get();
        if (char_result_steer.Status() != GattCommunicationStatus::Success ||
            char_result_steer.Characteristics().Size() == 0) {
            std::cerr << "[ble] Steering characteristic not found.\n";
            return false;
        }
        steer_char_ = char_result_steer.Characteristics().GetAt(0);

        // Throttle characteristic
        auto char_result_thr = target_svc
            .GetCharacteristicsForUuidAsync(winrt::guid(BLE_CHAR_UUID_TELEM)).get();
        if (char_result_thr.Status() != GattCommunicationStatus::Success ||
            char_result_thr.Characteristics().Size() == 0) {
            std::cerr << "[ble] Throttle characteristic not found.\n";
            return false;
        }
        throttle_char_ = char_result_thr.Characteristics().GetAt(0);

        connected_ = true;
        std::cout << "[ble] Connected to " << target.Name().c_str() << "\n";
        return true;
    }
    catch (const winrt::hresult_error& e) {
        std::wcerr << L"[ble] WinRT error: " << e.message().c_str() << L"\n";
        return false;
    }
}

bool BleDevice::write_steer(double delta_normalised)
{
    if (!connected_) return false;
    try {
        int16_t val = normalise_to_int16(delta_normalised);
        auto pkt = build_gatt_packet(PacketTag::Steer, val);

        DataWriter dw;
        dw.WriteBytes(winrt::array_view<const uint8_t>(pkt));
        auto buf = dw.DetachBuffer();

        auto status = steer_char_
            .WriteValueAsync(buf, GattWriteOption::WriteWithoutResponse).get();
        return status == GattCommunicationStatus::Success;
    }
    catch (...) { return false; }
}

bool BleDevice::write_throttle(double throttle_normalised)
{
    if (!connected_) return false;
    try {
        int16_t val = normalise_to_int16(throttle_normalised);
        auto pkt = build_gatt_packet(PacketTag::Throttle, val);

        DataWriter dw;
        dw.WriteBytes(winrt::array_view<const uint8_t>(pkt));
        auto buf = dw.DetachBuffer();

        auto status = throttle_char_
            .WriteValueAsync(buf, GattWriteOption::WriteWithoutResponse).get();
        return status == GattCommunicationStatus::Success;
    }
    catch (...) { return false; }
}

void BleDevice::disconnect()
{
    if (connected_) {
        connected_ = false;
        ble_dev_   = nullptr;
        std::cout << "[ble] Disconnected.\n";
    }
}

#endif  // _WIN32

// ============================================================================
// BLE control loop  (20 Hz)
// ============================================================================
void ble_control_thread(State& state)
{
    using clock = std::chrono::steady_clock;
    constexpr auto PERIOD = std::chrono::milliseconds(50);  // 20 Hz

    // Pre-compute LQR gain (recomputed if gains change – simple threshold check)
    Gains prev_gains;
    {
        std::lock_guard<std::mutex> lk(state.gains_mtx);
        prev_gains = state.gains;
    }
    auto lqr_K   = lqr_solve_gain(prev_gains);
    auto mpc_g   = mpc_build_gains(prev_gains);
    double mpc_u = 0.0;

#ifdef _WIN32
    BleDevice ble;
    {
        std::wstring dev_name = L"AutoCar_BLE";
        if (!ble.connect(dev_name)) {
            std::cerr << "[ble] Running in simulation mode (no BLE device).\n";
        }
    }
#else
    std::cout << "[ble] Non-Windows build – BLE disabled, running in sim mode.\n";
#endif

    std::cout << "[ble] Control loop started at 20 Hz.\n";

    while (state.running.load(std::memory_order_relaxed)) {
        auto t0 = clock::now();

        // Re-solve gains if they changed
        Gains cur_gains;
        {
            std::lock_guard<std::mutex> lk(state.gains_mtx);
            cur_gains = state.gains;
        }
        // Simple dirty-check on a few key params
        if (std::fabs(cur_gains.lqr_Q11 - prev_gains.lqr_Q11) > 1e-9 ||
            std::fabs(cur_gains.lqr_Q22 - prev_gains.lqr_Q22) > 1e-9 ||
            std::fabs(cur_gains.lqr_R   - prev_gains.lqr_R)   > 1e-9) {
            lqr_K = lqr_solve_gain(cur_gains);
        }
        if (std::fabs(cur_gains.mpc_Q_lat - prev_gains.mpc_Q_lat) > 1e-9 ||
            std::fabs(cur_gains.mpc_Q_psi - prev_gains.mpc_Q_psi) > 1e-9 ||
            std::fabs(cur_gains.mpc_R_du  - prev_gains.mpc_R_du)  > 1e-9 ||
            cur_gains.mpc_N != prev_gains.mpc_N) {
            mpc_g = mpc_build_gains(cur_gains);
            mpc_u = 0.0;  // reset integrator
        }
        prev_gains = cur_gains;

        // Wait until vision is ready
        if (!state.vision_ready.load(std::memory_order_acquire)) {
            std::this_thread::sleep_until(t0 + PERIOD);
            continue;
        }

        // Snapshot state (no nested locks – gains already copied above)
        double delta, throttle;
        {
            std::lock_guard<std::mutex> lk(state.state_mtx);
            state.gains = cur_gains;   // write gains under state_mtx only
            delta    = compute_control(state, lqr_K, mpc_g, mpc_u);
            throttle = state.throttle;  // set by GUI
        }

        // Write back
        {
            std::lock_guard<std::mutex> lk(state.ctrl_mtx);
            state.delta    = delta;
            state.throttle = throttle;
        }

        // Transmit over BLE
#ifdef _WIN32
        if (ble.is_connected()) {
            ble.write_steer(delta);
            ble.write_throttle(throttle);
        }
#endif

        std::this_thread::sleep_until(t0 + PERIOD);
    }

#ifdef _WIN32
    ble.disconnect();
#endif
    std::cout << "[ble] Control thread exiting.\n";
}

}  // namespace car
