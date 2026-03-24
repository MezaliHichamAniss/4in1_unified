#pragma once
// ============================================================================
//  ble_layer.h  –  BLE communication abstraction
//  Windows: WinRT async (windowsapp.lib)
//  Other:   stub (prints packets to stdout)
// ============================================================================
#include <cstdint>
#include <memory>
#include <string>
#include <functional>

namespace cdp {

// ---------------------------------------------------------------------------
// BLE packet – matches Python packet format exactly
//   header  0xAA
//   cmd     0x01  (drive command)
//   speed_h speed_l  (uint16 big-endian)
//   turn_h  turn_l   (int16 big-endian, centidegrees)
//   checksum = XOR of bytes 1..5
// ---------------------------------------------------------------------------
struct BlePacket {
    double speed_raw;   // BLE speed units
    double turn_deg;    // pod angle [deg]
};

// ---------------------------------------------------------------------------
// BLE device address (e.g. "AA:BB:CC:DD:EE:FF")
// ---------------------------------------------------------------------------
using BleAddress = std::string;

// Characteristic UUID used by the car (must match firmware)
static constexpr const char* BLE_CHAR_UUID =
    "0000ffe1-0000-1000-8000-00805f9b34fb";

// ---------------------------------------------------------------------------
// BleLayer – abstract base
// ---------------------------------------------------------------------------
class BleLayer {
public:
    virtual ~BleLayer() = default;

    // Connect to device by address.  Returns true on success.
    virtual bool connect(const BleAddress& addr) = 0;

    // Send one packet (non-blocking best-effort).
    virtual void send(const BlePacket& pkt) = 0;

    // Disconnect / cleanup.
    virtual void disconnect() = 0;

    // True if currently connected.
    virtual bool connected() const = 0;
};

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

// No-BLE stub (always "connected", packets printed to stdout)
std::unique_ptr<BleLayer> make_ble_stub();

#ifdef _WIN32
// WinRT async BLE layer (Windows 10+ only)
std::unique_ptr<BleLayer> make_ble_winrt();
#endif

// Return appropriate layer based on no_ble flag
std::unique_ptr<BleLayer> make_ble(bool no_ble);

}  // namespace cdp
