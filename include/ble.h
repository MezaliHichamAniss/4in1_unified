#pragma once
// ============================================================================
// ble.h  –  Windows BLE (WinRT) packet builder and control loop
// ============================================================================
#ifdef _WIN32
// C++/WinRT coroutine machinery
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Devices.Enumeration.h>
#endif

#include <cstdint>
#include <vector>
#include <string>
#include "state.h"

namespace car {

// ---------------------------------------------------------------------------
// GATT service / characteristic UUIDs (match Python version exactly)
// ---------------------------------------------------------------------------
constexpr const char* BLE_SERVICE_UUID    = "12345678-1234-5678-1234-56789abcdef0";
constexpr const char* BLE_CHAR_UUID_STEER = "12345678-1234-5678-1234-56789abcdef1";
constexpr const char* BLE_CHAR_UUID_TELEM = "12345678-1234-5678-1234-56789abcdef2";

// ---------------------------------------------------------------------------
// GATT packet builder
// Packet layout (matches Python struct.pack):
//   Byte 0    : command tag  (0x01 = steer, 0x02 = throttle, 0xFF = ping)
//   Bytes 1-2 : int16 payload, little-endian
//   Byte 3    : CRC8 (Dallas/Maxim) of bytes 0-2
// ---------------------------------------------------------------------------
enum class PacketTag : uint8_t {
    Steer    = 0x01,
    Throttle = 0x02,
    Ping     = 0xFF
};

std::vector<uint8_t> build_gatt_packet(PacketTag tag, int16_t value);
uint8_t              crc8_dallas(const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
// BLE device wrapper (Windows WinRT)
// ---------------------------------------------------------------------------
#ifdef _WIN32
class BleDevice {
public:
    BleDevice() = default;
    ~BleDevice();

    // Connect to device by display name (e.g. "AutoCar_BLE")
    bool connect(const std::wstring& device_name);
    bool is_connected() const { return connected_; }

    // Write a raw packet to the steering characteristic
    bool write_steer(double delta_normalised);
    bool write_throttle(double throttle_normalised);
    void disconnect();

private:
    bool connected_  = false;

#ifdef _WIN32
    winrt::Windows::Devices::Bluetooth::BluetoothLEDevice   ble_dev_{nullptr};
    winrt::Windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristic
        steer_char_{nullptr};
    winrt::Windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristic
        throttle_char_{nullptr};
#endif

    // Scale normalised [-1,1] → int16 range [-10000, 10000] (matches Python)
    static int16_t normalise_to_int16(double v) {
        int val = static_cast<int>(v * 10000.0);
        if (val >  10000) val =  10000;
        if (val < -10000) val = -10000;
        return static_cast<int16_t>(val);
    }
};
#endif  // _WIN32

// ---------------------------------------------------------------------------
// BLE control loop (20 Hz)  –  thread entry point
// ---------------------------------------------------------------------------
void ble_control_thread(State& state);

}  // namespace car
