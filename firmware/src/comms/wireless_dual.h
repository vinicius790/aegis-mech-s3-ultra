#pragma once
#include <stdint.h>

namespace aegis::comms {

struct WandPacket {
    uint8_t  magic; // 0xA5
    uint16_t seq;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint8_t  button_flags;
    uint8_t  battery_pct;
};

class WirelessDual {
public:
    WirelessDual() : esp_now_active_(false), ble_mouse_active_(false) {}

    bool init_esp_now();
    bool init_ble_hid_mouse();
    void send_hid_move(int8_t dx, int8_t dy);
    void send_hid_click(uint8_t button);
    bool receive_wand_packet(WandPacket& pkt);

    bool is_esp_now_active() const { return esp_now_active_; }
    bool is_ble_mouse_active() const { return ble_mouse_active_; }

private:
    bool esp_now_active_;
    bool ble_mouse_active_;
};

} // namespace aegis::comms
