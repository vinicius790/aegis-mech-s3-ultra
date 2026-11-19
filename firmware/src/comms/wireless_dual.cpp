#include "wireless_dual.h"
#include "protocol_frames.h"
#include <cstring>

#if defined(ESP_PLATFORM)
#include "esp_wifi.h"
#include "esp_now.h"
#endif

namespace aegis::comms {

struct __attribute__((packed)) BleHidKeyboardReport {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keycodes[6];
};

static bool s_esp_now_ready = false;
static bool s_ble_hid_ready = false;
static uint32_t s_espnow_tx_count = 0;
static uint32_t s_ble_hid_tx_count = 0;
static BleHidKeyboardReport s_last_hid_report{};

static uint8_t ascii_to_hid_keycode(char c, uint8_t& out_modifier) {
    out_modifier = 0x00;
    if (c >= 'a' && c <= 'z') {
        return static_cast<uint8_t>(0x04 + (c - 'a'));
    }
    if (c >= 'A' && c <= 'Z') {
        out_modifier = 0x02; // Left Shift
        return static_cast<uint8_t>(0x04 + (c - 'A'));
    }
    if (c >= '1' && c <= '9') {
        return static_cast<uint8_t>(0x1E + (c - '1'));
    }
    if (c == '0') return 0x27;
    if (c == '\n' || c == '\r') return 0x28; // Enter
    if (c == ' ') return 0x2C;               // Space
    return 0x00;
}

bool WirelessDualManager::esp_now_init() {
#if defined(ESP_PLATFORM)
    if (esp_now_init() != ESP_OK) {
        return false;
    }
    esp_now_peer_info_t peer{};
    std::memset(peer.peer_addr, 0xFF, 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
#endif
    s_esp_now_ready = true;
    return true;
}

bool WirelessDualManager::esp_now_broadcast_telemetry(const uint8_t* data, size_t len) {
    if (!s_esp_now_ready || data == nullptr || len == 0 || len > 250) {
        return false;
    }
    const uint16_t crc = aegis::protocol::crc16_ccitt_false(data, len);
    (void)crc;
#if defined(ESP_PLATFORM)
    static const uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (esp_now_send(kBroadcastMac, data, len) != ESP_OK) {
        return false;
    }
#endif
    ++s_espnow_tx_count;
    return true;
}

void WirelessDualManager::ble_hid_init() {
    std::memset(&s_last_hid_report, 0, sizeof(s_last_hid_report));
    s_ble_hid_ready = true;
}

void WirelessDualManager::ble_hid_send_key(char ascii_key) {
    if (!s_ble_hid_ready) {
        ble_hid_init();
    }
    uint8_t mod = 0;
    const uint8_t usage = ascii_to_hid_keycode(ascii_key, mod);
    s_last_hid_report.modifiers = mod;
    s_last_hid_report.reserved = 0x00;
    s_last_hid_report.keycodes[0] = usage;
    for (int i = 1; i < 6; ++i) {
        s_last_hid_report.keycodes[i] = 0x00;
    }
    ++s_ble_hid_tx_count;
}

} // namespace aegis::comms
