#include "hmi_local.h"
#include <math.h>
#include <stdio.h>

#if !defined(AEGIS_NATIVE_TEST)
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
static Adafruit_SSD1306 g_oled(128, 64, &Wire, -1);
static int g_ws_pin = 38;
static int g_bz_pin = 39;
#endif

namespace aegis::comms {

void LocalHMI::init(int ws2812_pin, int buzzer_pin) {
    mode_ = LEDMode::BREATHING_IDLE;
#if !defined(AEGIS_NATIVE_TEST)
    g_ws_pin = ws2812_pin;
    g_bz_pin = buzzer_pin;
    oled_ready_ = g_oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (oled_ready_) {
        g_oled.clearDisplay();
        g_oled.setTextSize(1);
        g_oled.setTextColor(SSD1306_WHITE);
        g_oled.setCursor(0, 0);
        g_oled.println(F("AEGIS-MECH S3 v3.0"));
        g_oled.display();
    }
    ledcSetup(2, 2000, 8); // Canal LEDC 2 p/ Buzzer passivo BZ1 via 2N2222
    ledcAttachPin(g_bz_pin, 2);
#else
    (void)ws2812_pin;
    (void)buzzer_pin;
    oled_ready_ = false;
#endif
}

void LocalHMI::render_oled_status(const char* fsm_state, float pos_mm, float vbus_v, const char* fault_str) {
#if !defined(AEGIS_NATIVE_TEST)
    if (!oled_ready_) return;
    g_oled.clearDisplay();
    g_oled.setCursor(0, 0);
    g_oled.printf("AEGIS v3.0 | P%u\n", page_idx_ + 1);
    g_oled.printf("FSM: %s\n", fsm_state ? fsm_state : "?");
    if (page_idx_ == 0) {
        g_oled.printf("Bolt: %.1f / 28.0 mm\n", pos_mm);
        g_oled.printf("Bus : %.2f V\n", vbus_v);
    } else if (page_idx_ == 1) {
        g_oled.printf("Power: %.2f V\n", vbus_v);
        g_oled.printf("Code : %s\n", fault_str ? fault_str : "E00");
    } else {
        g_oled.printf("FAULT/STATUS:\n%s\n", fault_str ? fault_str : "NOMINAL");
    }
    g_oled.display();
#else
    (void)fsm_state;
    (void)pos_mm;
    (void)vbus_v;
    (void)fault_str;
#endif
}

void LocalHMI::update_led_ring(uint32_t now_ms, uint8_t& out_r, uint8_t& out_g, uint8_t& out_b, bool low_power_shedding) {
    last_tick_ms_ = now_ms;
    switch (mode_) {
        case LEDMode::BREATHING_IDLE: {
            const float phase = static_cast<float>(now_ms % 2000U) / 2000.0f * 6.28318f;
            const uint8_t brightness = static_cast<uint8_t>(20.0f + 30.0f * (sinf(phase) + 1.0f) * 0.5f);
            out_r = 0; out_g = brightness / 2; out_b = brightness;
            break;
        }
        case LEDMode::SPINNING_AUTH:
            out_r = 80; out_g = 80; out_b = 0;
            break;
        case LEDMode::SUCCESS_GREEN:
            out_r = 0; out_g = 120; out_b = 0;
            break;
        case LEDMode::ALARM_PULSE_RED: {
            const bool flash = ((now_ms / 200U) % 2U) == 0U;
            out_r = flash ? 200 : 0; out_g = 0; out_b = 0;
            break;
        }
        case LEDMode::LOW_POWER_UPS:
            out_r = 35; out_g = 20; out_b = 0;
            break;
        default:
            out_r = 0; out_g = 0; out_b = 0;
            break;
    }
    if (low_power_shedding) {
        // Load shedding (D0D236D9): limita brilho WS2812B a 25% sob UPS baixa!
        out_r /= 4U;
        out_g /= 4U;
        out_b /= 4U;
    }
#if !defined(AEGIS_NATIVE_TEST)
    neopixelWrite(g_ws_pin, out_r, out_g, out_b);
#endif
}

uint16_t LocalHMI::get_buzzer_tone(uint32_t now_ms) const {
    uint16_t freq = 0;
    if (mode_ == LEDMode::ALARM_PULSE_RED) {
        freq = ((now_ms / 250U) % 2U == 0U) ? 2400U : 1800U;
    } else if (mode_ == LEDMode::SUCCESS_GREEN) {
        freq = 1200U;
    }
#if !defined(AEGIS_NATIVE_TEST)
    if (freq > 0U) {
        ledcWriteTone(2, freq);
    } else {
        ledcWrite(2, 0);
    }
#endif
    return freq;
}

} // namespace aegis::comms
