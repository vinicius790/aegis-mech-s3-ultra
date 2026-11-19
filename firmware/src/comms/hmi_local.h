#pragma once
#include <stdint.h>

namespace aegis::comms {

enum class LEDMode : uint8_t {
    OFF = 0,
    BREATHING_IDLE,
    SPINNING_AUTH,
    SUCCESS_GREEN,
    ALARM_PULSE_RED,
    LOW_POWER_UPS
};

class LocalHMI {
public:
    LocalHMI() : mode_(LEDMode::OFF), page_idx_(0), last_tick_ms_(0), oled_ready_(false) {}

    void init(int ws2812_pin = 38, int buzzer_pin = 39);
    void next_page() { page_idx_ = (page_idx_ + 1) % 3; }
    void render_oled_status(const char* fsm_state, float pos_mm, float vbus_v, const char* fault_str);
    void update_led_ring(uint32_t now_ms, uint8_t& out_r, uint8_t& out_g, uint8_t& out_b, bool low_power_shedding = false);
    void set_mode(LEDMode mode) { mode_ = mode; }

    uint16_t get_buzzer_tone(uint32_t now_ms) const;

private:
    LEDMode  mode_;
    uint8_t  page_idx_;
    uint32_t last_tick_ms_;
    bool     oled_ready_;
};

} // namespace aegis::comms
