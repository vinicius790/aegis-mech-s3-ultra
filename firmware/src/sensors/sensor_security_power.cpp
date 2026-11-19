#include "sensor_security_power.h"

#if !defined(AEGIS_NATIVE_TEST)
#include <Arduino.h>
#include <Wire.h>
#endif

namespace aegis::sensors {

bool SecurityPowerMonitor::init() {
    calibrated_ = false;
    stall_detected_ = false;
    return true;
}

void SecurityPowerMonitor::set_gyro_bias_fallback(float bx, float by, float bz) {
    bias_gx_ = bx;
    bias_gy_ = by;
    bias_gz_ = bz;
    calibrated_ = true;
}

bool SecurityPowerMonitor::calibrate_gyro_500(const IMURawData samples[500]) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (int i = 0; i < 500; ++i) {
        sum_x += samples[i].gx_dps;
        sum_y += samples[i].gy_dps;
        sum_z += samples[i].gz_dps;
    }
    const float mx = sum_x / 500.0f;
    const float my = sum_y / 500.0f;
    const float mz = sum_z / 500.0f;

    // Bug G10 corrigido: checa variancia nos 3 eixos (X, Y e Z) contra ataque/vibracao no boot!
    float var_x = 0.0f, var_y = 0.0f, var_z = 0.0f;
    for (int i = 0; i < 500; ++i) {
        const float dx = samples[i].gx_dps - mx;
        const float dy = samples[i].gy_dps - my;
        const float dz = samples[i].gz_dps - mz;
        var_x += dx * dx;
        var_y += dy * dy;
        var_z += dz * dz;
    }
    var_x /= 500.0f;
    var_y /= 500.0f;
    var_z /= 500.0f;

    if (fmaxf(var_x, fmaxf(var_y, var_z)) > 2.0f) {
        calibrated_ = false;
        return false; // Sinaliza ao caller para usar bias salvo na NVS
    }

    bias_gx_ = mx;
    bias_gy_ = my;
    bias_gz_ = mz;
    calibrated_ = true;
    return true;
}

void SecurityPowerMonitor::update_imu(const IMURawData& raw, float smoothing) {
    const float inv = 1.0f - smoothing;
    ema_ax_ = smoothing * ema_ax_ + inv * raw.ax_g;
    ema_ay_ = smoothing * ema_ay_ + inv * raw.ay_g;
    ema_az_ = smoothing * ema_az_ + inv * raw.az_g;

    float gx = raw.gx_dps - bias_gx_;
    float gy = raw.gy_dps - bias_gy_;
    float gz = raw.gz_dps - bias_gz_;

    if (fabsf(gx) < 0.5f) gx = 0.0f;
    if (fabsf(gy) < 0.5f) gy = 0.0f;
    if (fabsf(gz) < 0.5f) gz = 0.0f;

    ema_gx_ = smoothing * ema_gx_ + inv * gx;
    ema_gy_ = smoothing * ema_gy_ + inv * gy;
    ema_gz_ = smoothing * ema_gz_ + inv * gz;
}

float SecurityPowerMonitor::compute_tamper_score() const {
    const float acc_mag = sqrtf(ema_ax_ * ema_ax_ + ema_ay_ * ema_ay_ + ema_az_ * ema_az_);
    const float acc_perturb = fabsf(acc_mag - 1.0f);
    const float gyro_mag = sqrtf(ema_gx_ * ema_gx_ + ema_gy_ * ema_gy_ + ema_gz_ * ema_gz_);
    const float score = (acc_perturb / 2.0f) * 0.6f + (gyro_mag / 100.0f) * 0.4f;
    return fminf(1.0f, fmaxf(0.0f, score));
}

PowerStats SecurityPowerMonitor::read_power(float shunt_voltage_mv, float bus_voltage_v) {
    const float current = shunt_voltage_mv * 10.0f; // R_SHUNT = 0.100 ohm -> I(mA) = V_shunt(mV) * 10
    const float power = bus_voltage_v * current;
    stall_detected_ = (current > 1800.0f);
    return PowerStats{bus_voltage_v, current, power, stall_detected_};
}

ATSStatus SecurityPowerMonitor::evaluate_ats(float v_mains_v, float v_batt_v, bool estop_pin_high) const {
    ATSStatus st{};
    st.v_mains_v = v_mains_v;
    st.v_batt_v  = v_batt_v;
    st.estop_active = estop_pin_high;
    st.on_ups_batt = (v_mains_v < 8.5f && v_batt_v >= 6.4f);
    const float v_bus = fmaxf(v_mains_v, v_batt_v);
    st.voltage_abnormal = (v_mains_v > 14.2f) || (v_bus < 6.2f);
    return st;
}

float SecurityPowerMonitor::raw_angle_to_bolt_mm(uint16_t raw_12bit, float mm_per_deg) const {
    const float deg = (static_cast<float>(raw_12bit & 0x0FFFU) * 360.0f) / 4096.0f;
    return fmaxf(0.0f, (deg - zero_deg_offset_) * mm_per_deg);
}

#if !defined(AEGIS_NATIVE_TEST)
bool SecurityPowerMonitor::init_hardware_i2c() {
    // 1. MPU6050 (0x68): Acorda PLL_X, DLPF=44Hz, 100Hz, +-500dps, +-8G
    Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x19); Wire.write(9);    Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

    // 2. INA219 (0x40): Config 0x399F, Cal = 4096 (0x1000) p/ R=0.1R -> 100uA/bit
    Wire.beginTransmission(0x40); Wire.write(0x00); Wire.write(0x39); Wire.write(0x9F); Wire.endTransmission();
    Wire.beginTransmission(0x40); Wire.write(0x05); Wire.write(0x10); Wire.write(0x00); Wire.endTransmission();
    return true;
}

bool SecurityPowerMonitor::read_mpu6050(IMURawData& out_raw) {
    Wire.beginTransmission(0x68); Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x68, 14) < 14) return false;
    auto rd16 = []() -> int16_t { return static_cast<int16_t>((Wire.read() << 8) | Wire.read()); };
    out_raw.ax_g = rd16() / 4096.0f;
    out_raw.ay_g = rd16() / 4096.0f;
    out_raw.az_g = rd16() / 4096.0f;
    (void)rd16(); // temperatura interna MPU
    out_raw.gx_dps = rd16() / 65.5f;
    out_raw.gy_dps = rd16() / 65.5f;
    out_raw.gz_dps = rd16() / 65.5f;
    return true;
}

bool SecurityPowerMonitor::read_ina219(PowerStats& out_stats) {
    Wire.beginTransmission(0x40); Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x40, 2) < 2) return false;
    const uint16_t raw_v = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
    const float v_bus = static_cast<float>(raw_v >> 3) * 0.004f;

    Wire.beginTransmission(0x40); Wire.write(0x04);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x40, 2) < 2) return false;
    const int16_t raw_i = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    const float cur_ma = static_cast<float>(raw_i) * 0.1f; // 100uA/bit
    out_stats = PowerStats{v_bus, cur_ma, v_bus * cur_ma, cur_ma > 1800.0f};
    return true;
}

bool SecurityPowerMonitor::read_as5600_bolt_mm(float& out_mm) {
    Wire.beginTransmission(0x36); Wire.write(0x0B); // STATUS: bit5 MD = magnet detected
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x36, 1) < 1) return false;
    const uint8_t status = Wire.read();
    if ((status & 0x20U) == 0) return false;

    Wire.beginTransmission(0x36); Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(0x36, 2) < 2) return false;
    const uint16_t raw = ((static_cast<uint16_t>(Wire.read()) & 0x0FU) << 8) | Wire.read();
    out_mm = raw_angle_to_bolt_mm(raw);
    return true;
}
#endif

} // namespace aegis::sensors
