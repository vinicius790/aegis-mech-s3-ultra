#pragma once
#include <stdint.h>
#include <math.h>

#if !defined(AEGIS_NATIVE_TEST)
#include <driver/rmt.h>
#endif

namespace aegis::sensors {

// Velocidade do som termohigrometrica Magnus-Tetens c(T, RH) em m/s
inline float compute_sound_speed_mps(float temp_c, float rh_pct = 50.0f) {
    const float es = 6.112f * expf((17.67f * temp_c) / (temp_c + 243.5f));
    const float vapor_frac = (fmaxf(0.0f, fminf(100.0f, rh_pct)) / 100.0f) * (es / 1013.25f);
    const float c_dry = 331.3f * sqrtf(1.0f + temp_c / 273.15f);
    return c_dry * (1.0f + 0.0016f * vapor_frac);
}

// Driver HC-SR04 nao-bloqueante usando ESP-IDF 4.4 <driver/rmt.h> + timeout 38ms + Ping-Pong
class UltrasonicRMT {
public:
    static constexpr uint32_t MAX_ECHO_US = 38000U; // Datasheet HC-SR04 (lab-uc)

    UltrasonicRMT(int trig_pin, int echo_pin, int rmt_tx_ch = 0, int rmt_rx_ch = 4)
        : trig_(trig_pin), echo_(echo_pin), tx_ch_(rmt_tx_ch), rx_ch_(rmt_rx_ch),
          last_dist_mm_(NAN), pulse_start_us_(0), pulse_duration_us_(0), waiting_echo_(false) {}

    void init();
    void trigger_pulse(uint32_t now_us);
    void poll_rmt_rx();
    void handle_echo_change(bool is_high, uint32_t now_us);
    float read_distance_mm(float temp_c = 25.0f, float rh_pct = 50.0f);
    bool is_timeout(uint32_t now_us, uint32_t timeout_us = MAX_ECHO_US) const;

private:
    int trig_;
    int echo_;
    int tx_ch_;
    int rx_ch_;
    float last_dist_mm_;
    uint32_t pulse_start_us_;
    uint32_t pulse_duration_us_;
    bool waiting_echo_;
#if !defined(AEGIS_NATIVE_TEST)
    RingbufHandle_t rb_handle_ = nullptr;
#endif
};

} // namespace aegis::sensors
