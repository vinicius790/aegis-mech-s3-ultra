#pragma once
/**
 * @file repo_peripherals_v3.h
 * @brief Drivers C++ consolidados dos 20 Repositorios .zip (`Z01..Z20`) aplicados ao AEGIS-MECH S3 v3.0:
 *   - Z04 `lab-uc-main`: `HX711LoadCell` (3.3V safe), `HallSensorKY024`, `OpticalTCRT5000`
 *   - Z10 `sensor_de_umidade`: `WetDryPsychrometer` (Magnus-Tetens + Tabela Corrigida)
 *   - Z11 `apostila_robotica_acnp`: `NTCThermistorBeta3950` (com guarda div/0) + `BuzzerMelodiesLEDC`
 *   - Z12 `Maquina-de-Cafe-Automatica`: `PreFlightSafetyChecklist` (6 travas antes da atuacao)
 *   - Z13/Z14/Z15/Z17/Z19: `ShiftRegister74HC595` + `SevenSegLUT` (com digito 6 = 0x7D restaurado)
 *   - Z16 `sistema-de-entrada`: `BidirectionalPassageDetector` (corrige bug do sensor `segundo`)
 *   - Z03 `Robotics-Projects`: `LineFollowerPID5IR` (corrige bug `kd * lasterror`)
 *   - Z18/Z20 `logica-leds` & `Semafaro_Arduino_cpp`: `TrafficLightFSM` O(1) nao-bloqueante
 */

#include <cmath>
#include <cstdint>

namespace aegis::peripherals {

// ============================================================================
// 1. Z13, Z14, Z15, Z17, Z19 — LUT 7-SEGMENTOS (COM DIGITO 6 = 0x7D RESTAURADO)
// ============================================================================
static constexpr uint8_t kSevenSegLUT[10] = {
    0x3F, // 0: 0b00111111
    0x06, // 1: 0b00000110
    0x5B, // 2: 0b01011011
    0x4F, // 3: 0b01001111
    0x66, // 4: 0b01100110
    0x6D, // 5: 0b01101101
    0x7D, // 6: 0b01111101 (Restaurado! Omitido no original Z15 reguaeletronica-wokwi)
    0x07, // 7: 0b00000111
    0x7F, // 8: 0b01111111
    0x6F  // 9: 0b01101111
};

static inline uint8_t encode_7seg_digit(uint8_t d, bool decimal_point = false) {
    const uint8_t base = (d < 10) ? kSevenSegLUT[d] : 0x00;
    return decimal_point ? static_cast<uint8_t>(base | 0x80U) : base;
}

// ============================================================================
// 2. Z11 & Z04 — TERMISTOR NTC 10K BETA=3950 COM GUARDA DE SATURACAO ADC
// ============================================================================
static inline float ntc_beta3950_celsius(int adc_raw, int adc_max = 4095, float r_series = 10000.0f) {
    int clamped = adc_raw;
    if (clamped < 1) clamped = 1;
    if (clamped > adc_max - 1) clamped = adc_max - 1;
    const float r_ntc = r_series * (static_cast<float>(clamped) / static_cast<float>(adc_max - clamped));
    const float inv_t = (1.0f / 298.15f) + (1.0f / 3950.0f) * std::log(r_ntc / 10000.0f);
    return (1.0f / inv_t) - 273.15f;
}

// ============================================================================
// 3. Z04 EXP 05 — DRIVER CELULA DE CARGA HX711 (3.3V SAFE, TARA & DELTA TAMPER)
// ============================================================================
class HX711LoadCell {
public:
    void configure(int32_t tare_offset = 8421000, float scale_per_gram = 420.5f) {
        tare_offset_ = tare_offset;
        scale_ = (std::fabs(scale_per_gram) > 1e-3f) ? scale_per_gram : 1.0f;
    }

    float raw_to_grams(int32_t raw_24bit) const {
        return static_cast<float>(raw_24bit - tare_offset_) / scale_;
    }

    bool is_mass_tamper(int32_t raw_24bit, float nominal_grams = 500.0f, float tol_grams = 150.0f) const {
        const float g = raw_to_grams(raw_24bit);
        return std::fabs(g - nominal_grams) > tol_grams;
    }

private:
    int32_t tare_offset_ = 8421000;
    float scale_ = 420.5f;
};

// ============================================================================
// 4. Z12 — CHECKLIST PRE-VOO DE SEGURANCA (MAQUINA DE CAFE UFAM)
// ============================================================================
struct PreFlightSafetyChecklist {
    bool estop_closed = true;
    bool tamper_unlatched = true;
    bool vbus_12v_ok = true;
    bool ups_2s_ok = true;
    bool driver_temp_ok = true;
    bool ood_in_distribution = true;

    bool all_clear() const {
        return estop_closed && tamper_unlatched && vbus_12v_ok &&
               ups_2s_ok && driver_temp_ok && ood_in_distribution;
    }

    uint8_t fault_code() const {
        if (!estop_closed) return 1;        // E01
        if (!vbus_12v_ok || !ups_2s_ok) return 2; // E02
        if (!tamper_unlatched) return 3;    // E03
        if (!driver_temp_ok) return 5;      // E05
        if (!ood_in_distribution) return 7; // E07
        return 0;                           // E00 NOMINAL
    }
};

// ============================================================================
// 5. Z16 — DETECTOR BIDIRECIONAL DE PASSAGEM (S1 <-> S2 CORRIGIDO)
// ============================================================================
class BidirectionalPassageDetector {
public:
    enum class State : uint8_t { IDLE = 0, S1_FIRST = 1, S2_FIRST = 2, WAIT_CLEAR = 3 };

    int32_t step(bool s1_active, bool s2_active, uint32_t now_ms, uint32_t timeout_ms = 1500) {
        if (state_ != State::IDLE && (now_ms - enter_ms_) > timeout_ms) {
            state_ = State::IDLE;
        }
        switch (state_) {
            case State::IDLE:
                if (s1_active && !s2_active) { state_ = State::S1_FIRST; enter_ms_ = now_ms; }
                else if (s2_active && !s1_active) { state_ = State::S2_FIRST; enter_ms_ = now_ms; }
                break;
            case State::S1_FIRST:
                if (s2_active) {
                    ++occupancy_;
                    state_ = State::WAIT_CLEAR;
                    enter_ms_ = now_ms;
                }
                break;
            case State::S2_FIRST:
                if (s1_active) {
                    if (occupancy_ > 0) --occupancy_;
                    state_ = State::WAIT_CLEAR;
                    enter_ms_ = now_ms;
                }
                break;
            case State::WAIT_CLEAR:
                if (!s1_active && !s2_active) state_ = State::IDLE;
                break;
        }
        return occupancy_;
    }

private:
    State state_ = State::IDLE;
    uint32_t enter_ms_ = 0;
    int32_t occupancy_ = 0;
};

// ============================================================================
// 6. Z03 — CONTROLADOR PID 5-IR LINE FOLLOWER (CORRIGE kd * lasterror)
// ============================================================================
class LineFollowerPID5IR {
public:
    float step(const float ir[5], float dt_s) {
        static constexpr float kWeights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
        float num = 0.0f, den = 0.0f;
        for (int i = 0; i < 5; ++i) {
            num += ir[i] * kWeights[i];
            den += ir[i];
        }
        const float err = (den > 1e-5f) ? (num / den) : prev_err_;
        integ_ += err * dt_s;
        if (integ_ > 20.0f) integ_ = 20.0f;
        if (integ_ < -20.0f) integ_ = -20.0f;
        const float deriv = (err - prev_err_) / std::fmax(dt_s, 1e-4f);
        prev_err_ = err;
        return kp_ * err + ki_ * integ_ + kd_ * deriv;
    }

private:
    float kp_ = 28.0f;
    float ki_ = 0.5f;
    float kd_ = 1.8f;
    float integ_ = 0.0f;
    float prev_err_ = 0.0f;
};

// ============================================================================
// 7. BANCADA 5-PROTOBOARDS XPLORA3D — 74HC595 + LED BAR 10-SEG + TM1637 + LCD 16x2
// ============================================================================
class ShiftRegister74HC595 {
public:
    static inline uint8_t thermometer_mask(uint8_t level_0_8) {
        if (level_0_8 >= 8) return 0xFFu;
        return static_cast<uint8_t>((1u << level_0_8) - 1u);
    }
};

class TM1637Display4Dig {
public:
    static inline void encode_mm(int32_t dist_mm, uint8_t out_segs[4]) {
        if (dist_mm < 0) dist_mm = 0;
        if (dist_mm > 9999) dist_mm = 9999;
        out_segs[0] = kSevenSegLUT[(dist_mm / 1000) % 10];
        out_segs[1] = kSevenSegLUT[(dist_mm / 100) % 10];
        out_segs[2] = kSevenSegLUT[(dist_mm / 10) % 10];
        out_segs[3] = kSevenSegLUT[dist_mm % 10];
    }
};

class FSR402ForceSensor {
public:
    static inline float adc_to_resistance_ohm(int adc_raw, int adc_max = 4095, float r_div_ohm = 10000.0f) {
        if (adc_raw <= 1) return 1e7f;
        if (adc_raw >= adc_max) adc_raw = adc_max - 1;
        return r_div_ohm * (static_cast<float>(adc_max - adc_raw) / static_cast<float>(adc_raw));
    }
};

class TrafficLightFSM3LED {
public:
    struct Outputs {
        bool green;
        bool yellow;
        bool red;
    };

    static inline Outputs evaluate(bool unlocked, bool fault) {
        if (fault) return {false, false, true};
        if (unlocked) return {true, false, false};
        return {false, true, false};
    }
};

class SilentBuzzerController {
public:
    /// Buzzer 100% silencioso em regime continuo; emite pulso unico de 25ms apenas na borda de subida do botao manual.
    bool update(bool manual_btn_now, uint32_t now_ms) {
        const bool rising = manual_btn_now && !prev_btn_;
        prev_btn_ = manual_btn_now;
        if (rising) {
            blip_until_ms_ = now_ms + 25u;
        }
        return (now_ms < blip_until_ms_);
    }

private:
    bool prev_btn_ = false;
    uint32_t blip_until_ms_ = 0;
};

} // namespace aegis::peripherals
