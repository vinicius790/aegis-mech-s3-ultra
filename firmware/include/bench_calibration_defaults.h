#pragma once
/**
 * @file bench_calibration_defaults.h
 * @brief Parametros Padrao de Comissionamento de Bancada e Calibracao Inicial (AEGIS-MECH S3 v3.0 Ultra).
 *
 * Garante boot seguro e deterministico em hardware fisico antes mesmo da gravacao
 * de um novo template biometrico na NVS via botao BOOT/ENROLL:
 *   - Offsets estacionarios iniciais do MPU6050 (calibracao 500 amostras Z05)
 *   - Limites mecanicos seguros do servo SG90 / trava linear (0..90 deg)
 *   - Template biometrico 16-D normalizado (norma L2 = 1.0)
 *   - Parametros do filtro de Kalman na Forma de Joseph (auto-inicializado no 1o frame)
 */

#include <stdint.h>

namespace aegis::calibration {

struct ImuZeroBias {
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 1.0f;
    uint16_t samples = 500u;
};

struct ActuatorLimits {
    float servo_closed_deg = 0.0f;
    float servo_open_deg = 90.0f;
    float scurve_duration_s = 0.65f;
    uint8_t solenoid_peak_pwm = 230u;   // Soft-start pico inicial
    uint8_t solenoid_hold_pwm = 95u;    // Retencao termica segura
    uint16_t buzzer_manual_blip_ms = 25u;
};

struct KalmanDefaults {
    float q_pos = 1.5f;
    float r_meas = 25.0f;
    float mahalanobis_soft_gate = 4.5f;
    bool auto_init_first_frame = true;
};

// Template biometrico 16-D normalizado (norma L2 = 1.0) para boot inicial antes do enroll na NVS
static constexpr float kDefaultBioTemplate16D[16] = {
    0.3125f,  0.2823f, -0.1915f,  0.2419f,
    0.3327f, -0.1512f,  0.2218f,  0.2924f,
   -0.2117f,  0.2621f,  0.1815f, -0.2520f,
    0.2722f,  0.2016f, -0.1714f,  0.3024f
};

static constexpr float kBioCosineThreshold = 0.78f;
static constexpr float kOodMseMax = 0.25f;

} // namespace aegis::calibration
