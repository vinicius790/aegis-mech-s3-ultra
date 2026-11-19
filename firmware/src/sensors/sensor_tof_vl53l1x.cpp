#include "sensor_tof_vl53l1x.h"

#if !defined(AEGIS_NATIVE_TEST)
#include <Arduino.h>
#include <Wire.h>
#endif

namespace aegis::sensors {

void i2c_bus_recovery(int sda_pin, int scl_pin) {
#if !defined(AEGIS_NATIVE_TEST)
    Wire.end();
    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, OUTPUT);
    for (int i = 0; i < 9; ++i) {
        if (digitalRead(sda_pin) == HIGH) break;
        digitalWrite(scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_pin, HIGH);
        delayMicroseconds(5);
    }
    pinMode(sda_pin, OUTPUT);
    digitalWrite(sda_pin, LOW);  delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH); delayMicroseconds(5);
    digitalWrite(sda_pin, HIGH); delayMicroseconds(5);
    Wire.begin(sda_pin, scl_pin, 400000);
#else
    (void)sda_pin;
    (void)scl_pin;
#endif
}

void ToFVL53L1X::set_xshut(bool active_high) {
#if !defined(AEGIS_NATIVE_TEST)
    pinMode(xshut_, OUTPUT);
    digitalWrite(xshut_, active_high ? HIGH : LOW);
#else
    (void)active_high;
#endif
}

bool ToFVL53L1X::init_and_readdress() {
    set_xshut(true);
#if !defined(AEGIS_NATIVE_TEST)
    delay(5);
    // Troca registrador I2C_SLAVE__DEVICE_ADDRESS (0x0001) de 0x29 -> addr_ (7-bit)
    Wire.beginTransmission(0x29);
    Wire.write(0x00);
    Wire.write(0x01);
    Wire.write(addr_ & 0x7F);
    if (Wire.endTransmission() != 0) {
        // Verifica se ja responde no endereco de destino
        Wire.beginTransmission(addr_);
        initialized_ = (Wire.endTransmission() == 0);
        return initialized_;
    }
    initialized_ = true;
    return true;
#else
    initialized_ = true;
    return true;
#endif
}

float ToFVL53L1X::get_distance_mm() {
    if (!initialized_) return NAN;
#if !defined(AEGIS_NATIVE_TEST)
    // Le resultado nao-bloqueante (REG_RESULT__RANGE_STATUS = 0x0089, dist = +13/+14)
    Wire.beginTransmission(addr_);
    Wire.write(0x00);
    Wire.write(0x89);
    if (Wire.endTransmission(false) != 0) return NAN;
    if (Wire.requestFrom(static_cast<int>(addr_), 17) < 17) return NAN;
    uint8_t buf[17];
    for (int i = 0; i < 17; ++i) buf[i] = Wire.read();
    const uint8_t range_status = buf[0] & 0x1F;
    const uint16_t dist = (static_cast<uint16_t>(buf[13]) << 8) | buf[14];
    if (range_status != 9 || dist < 40 || dist > 1300) {
        last_dist_mm_ = NAN;
    } else {
        last_dist_mm_ = static_cast<float>(dist);
    }
#endif
    return last_dist_mm_;
}

} // namespace aegis::sensors
