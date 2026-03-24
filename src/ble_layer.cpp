// ============================================================================
//  ble_layer.cpp  –  BLE abstraction
//  Stub (no-BLE) always available; WinRT only on Windows.
// ============================================================================
#include "ble_layer.h"
#include <memory>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>

namespace cdp {

// ============================================================================
//  Packet builder (matches Python format exactly)
//  Byte layout: [0xAA, 0x01, speed_h, speed_l, turn_h, turn_l, checksum]
// ============================================================================
static void build_packet(const BlePacket& pkt,
                          uint8_t out[7]) {
    uint16_t speed = static_cast<uint16_t>(
        std::max(0.0, std::min(pkt.speed_raw, 65535.0)));
    int16_t  turn  = static_cast<int16_t>(
        std::max(-3200.0, std::min(pkt.turn_deg * 100.0, 3200.0)));

    out[0] = 0xAA;
    out[1] = 0x01;
    out[2] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>( speed        & 0xFF);
    out[4] = static_cast<uint8_t>((static_cast<uint16_t>(turn) >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>( static_cast<uint16_t>(turn)        & 0xFF);
    out[6] = out[1] ^ out[2] ^ out[3] ^ out[4] ^ out[5]; // XOR checksum
}

// ============================================================================
//  No-BLE stub – always "connected", prints packets to stdout
// ============================================================================
class BleStub final : public BleLayer {
public:
    bool connect(const BleAddress& addr) override {
        std::cout << "[BLE-stub] connect(" << addr << ") → OK\n";
        connected_ = true;
        return true;
    }
    void send(const BlePacket& pkt) override {
        uint8_t buf[7];
        build_packet(pkt, buf);
        std::cout << "[BLE-stub] pkt speed=" << pkt.speed_raw
                  << " turn=" << std::fixed << std::setprecision(2)
                  << pkt.turn_deg << "°  raw=[";
        for (int i = 0; i < 7; ++i)
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(buf[i])
                      << (i < 6 ? " " : "");
        std::cout << std::dec << "]\n";
    }
    void disconnect() override {
        std::cout << "[BLE-stub] disconnect\n";
        connected_ = false;
    }
    bool connected() const override { return connected_; }
private:
    bool connected_ = false;
};

std::unique_ptr<BleLayer> make_ble_stub() {
    return std::make_unique<BleStub>();
}

// ============================================================================
//  WinRT BLE layer (Windows 10+ only)
// ============================================================================
#ifdef _WIN32

// Pull in WinRT/C++ headers
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Storage.Streams.h>

using namespace winrt;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Storage::Streams;

// Parse "AA:BB:CC:DD:EE:FF" → uint64 address
static uint64_t parse_ble_address(const std::string& addr) {
    uint64_t v = 0;
    unsigned b;
    for (int i = 0; i < 6; ++i) {
        std::sscanf(addr.c_str() + i * 3, "%02x", &b);
        v = (v << 8) | (b & 0xFF);
    }
    return v;
}

class BleWinRT final : public BleLayer {
public:
    ~BleWinRT() { disconnect(); }

    bool connect(const BleAddress& addr) override {
        try {
            uint64_t mac = parse_ble_address(addr);
            device_ = BluetoothLEDevice::FromBluetoothAddressAsync(mac).get();
            if (!device_) return false;

            auto result = device_.GetGattServicesAsync().get();
            if (result.Status() != GattCommunicationStatus::Success) return false;

            for (auto svc : result.Services()) {
                auto cr = svc.GetCharacteristicsAsync().get();
                if (cr.Status() != GattCommunicationStatus::Success) continue;
                for (auto c : cr.Characteristics()) {
                    auto uuid = to_string(c.Uuid());
                    if (uuid == BLE_CHAR_UUID) {
                        char_ = c;
                        connected_ = true;
                        return true;
                    }
                }
            }
        } catch (const hresult_error& ex) {
            std::cerr << "[BLE-winrt] connect error: "
                      << to_string(ex.message()) << "\n";
        }
        return false;
    }

    void send(const BlePacket& pkt) override {
        if (!connected_) return;
        try {
            uint8_t buf[7];
            build_packet(pkt, buf);

            DataWriter dw;
            dw.WriteBytes(buf);
            auto ibuf = dw.DetachBuffer();
            char_.WriteValueAsync(ibuf,
                GattWriteOption::WriteWithoutResponse).get();
        } catch (...) {
            connected_ = false;
        }
    }

    void disconnect() override {
        connected_ = false;
        if (device_) device_.Close();
    }

    bool connected() const override { return connected_; }

private:
    BluetoothLEDevice        device_{nullptr};
    GattCharacteristic       char_{nullptr};
    bool                     connected_ = false;
};

std::unique_ptr<BleLayer> make_ble_winrt() {
    return std::make_unique<BleWinRT>();
}

#endif  // _WIN32

// ============================================================================
//  Factory
// ============================================================================
std::unique_ptr<BleLayer> make_ble(bool no_ble) {
    if (no_ble) {
        return make_ble_stub();
    }
#ifdef _WIN32
    return make_ble_winrt();
#else
    std::cerr << "[BLE] WinRT not available on this platform; falling back to stub\n";
    return make_ble_stub();
#endif
}

}  // namespace cdp
