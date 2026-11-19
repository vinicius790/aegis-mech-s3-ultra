#pragma once
#include <stdint.h>
#include <math.h>
#include "../protocol_frames.h"

namespace aegis::sensors {

struct PowerStats {
    float bus_voltage_v;
    float current_ma;
    float power_mw;
    bool  is_stall;
};

struct ATSStatus {
    float v_mains_v;
    float v_batt_v;
    bool  on_ups_batt;
    bool  voltage_abnormal;
    bool  estop_active;
};

struct IMURawData {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
};

class SecurityPowerMonitor {
public:
    SecurityPowerMonitor()
        : bias_gx_(0.0f), bias_gy_(0.0f), bias_gz_(0.0f),
          ema_ax_(0.0f), ema_ay_(0.0f), ema_az_(1.0f),
          ema_gx_(0.0f), ema_gy_(0.0f), ema_gz_(0.0f),
          zero_deg_offset_(0.0f), calibrated_(false), stall_detected_(false) {}

    bool init();
    // Calibracao estatica dual de 500 amostras validando VARIANCIA NOS 3 EIXOS (X, Y, Z)
    bool calibrate_gyro_500(const IMURawData samples[500]);
    void set_gyro_bias_fallback(float bx, float by, float bz);
    void update_imu(const IMURawData& raw, float smoothing = 0.82f);
    float compute_tamper_score() const;

    // Leitura hardware I2C & ATS ADC
    PowerStats read_power(float shunt_voltage_mv, float bus_voltage_v);
    ATSStatus evaluate_ats(float v_mains_v, float v_batt_v, bool estop_pin_high) const;
    float raw_angle_to_bolt_mm(uint16_t raw_12bit, float mm_per_deg = 0.174532925f) const;
    void set_zero_deg(float deg) { zero_deg_offset_ = deg; }

#if !defined(AEGIS_NATIVE_TEST)
    bool init_hardware_i2c();
    bool read_mpu6050(IMURawData& out_raw);
    bool read_ina219(PowerStats& out_stats);
    bool read_as5600_bolt_mm(float& out_mm);
#endif

private:
    float bias_gx_, bias_gy_, bias_gz_;
    float ema_ax_, ema_ay_, ema_az_;
    float ema_gx_, ema_gy_, ema_gz_;
    float zero_deg_offset_;
    bool  calibrated_;
    bool  stall_detected_;
};

} // namespace aegis::sensors
