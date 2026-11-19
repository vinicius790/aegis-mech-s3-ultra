#include "sensor_ultrasonic_rmt.h"

namespace aegis::sensors {

void UltrasonicRMT::init() {
    waiting_echo_ = false;
    pulse_duration_us_ = 0;
    last_dist_mm_ = NAN;
#if !defined(AEGIS_NATIVE_TEST)
    rmt_config_t tx_cfg = RMT_DEFAULT_CONFIG_TX(static_cast<gpio_num_t>(trig_), static_cast<rmt_channel_t>(tx_ch_));
    tx_cfg.clk_div = 80; // 1 MHz -> 1 tick = 1 us
    rmt_config(&tx_cfg);
    rmt_driver_install(static_cast<rmt_channel_t>(tx_ch_), 0, 0);

    rmt_config_t rx_cfg = RMT_DEFAULT_CONFIG_RX(static_cast<gpio_num_t>(echo_), static_cast<rmt_channel_t>(rx_ch_));
    rx_cfg.clk_div = 80;
    rx_cfg.rx_config.filter_en = true;
    rx_cfg.rx_config.filter_ticks_thresh = 250; // Filtro glitch < ~3.1 us @ APB 80 MHz
    rx_cfg.rx_config.idle_threshold = MAX_ECHO_US;
    rmt_config(&rx_cfg);
    rmt_driver_install(static_cast<rmt_channel_t>(rx_ch_), 1024, 0);
    rmt_get_ringbuf_handle(static_cast<rmt_channel_t>(rx_ch_), &rb_handle_);
    rmt_rx_start(static_cast<rmt_channel_t>(rx_ch_), true);
#endif
}

void UltrasonicRMT::trigger_pulse(uint32_t now_us) {
    pulse_start_us_ = now_us;
    pulse_duration_us_ = 0;
    waiting_echo_ = true;
#if !defined(AEGIS_NATIVE_TEST)
    const rmt_item32_t pulse = {{{ 10, 1, 2, 0 }}}; // 10 us HIGH, 2 us LOW sem bloqueio!
    rmt_write_items(static_cast<rmt_channel_t>(tx_ch_), &pulse, 1, false);
#endif
}

void UltrasonicRMT::poll_rmt_rx() {
#if !defined(AEGIS_NATIVE_TEST)
    if (rb_handle_ == nullptr) return;
    size_t rx_size = 0;
    rmt_item32_t* items = reinterpret_cast<rmt_item32_t*>(xRingbufferReceive(rb_handle_, &rx_size, 0));
    if (items != nullptr) {
        const size_t n = rx_size / sizeof(rmt_item32_t);
        for (size_t i = 0; i < n; ++i) {
            if (items[i].level0 == 1 && items[i].duration0 > 100U) {
                pulse_duration_us_ = items[i].duration0;
                waiting_echo_ = false;
                break;
            }
            if (items[i].level1 == 1 && items[i].duration1 > 100U) {
                pulse_duration_us_ = items[i].duration1;
                waiting_echo_ = false;
                break;
            }
        }
        vRingbufferReturnItem(rb_handle_, reinterpret_cast<void*>(items));
    }
#endif
}

void UltrasonicRMT::handle_echo_change(bool is_high, uint32_t now_us) {
    if (is_high) {
        pulse_start_us_ = now_us;
    } else if (waiting_echo_) {
        pulse_duration_us_ = now_us - pulse_start_us_;
        waiting_echo_ = false;
    }
}

float UltrasonicRMT::read_distance_mm(float temp_c, float rh_pct) {
    poll_rmt_rx();
    if (waiting_echo_ || pulse_duration_us_ == 0 || pulse_duration_us_ >= MAX_ECHO_US) {
        return NAN; // Contrato estrito: 0 / timeout -> NaN
    }
    const float c_t = compute_sound_speed_mps(temp_c, rh_pct);
    const float dist = (static_cast<float>(pulse_duration_us_) * c_t) / 2000.0f;
    if (dist < 20.0f || dist > 4000.0f) {
        last_dist_mm_ = NAN;
    } else {
        last_dist_mm_ = dist;
    }
    return last_dist_mm_;
}

bool UltrasonicRMT::is_timeout(uint32_t now_us, uint32_t timeout_us) const {
    return waiting_echo_ && ((now_us - pulse_start_us_) >= timeout_us);
}

} // namespace aegis::sensors
