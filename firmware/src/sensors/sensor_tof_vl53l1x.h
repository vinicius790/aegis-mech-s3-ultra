#pragma once
#include <stdint.h>
#include <math.h>

namespace aegis::sensors {

// Recuperacao fisica de barramento I2C travado com SDA=LOW (9 pulsos de clock + STOP)
void i2c_bus_recovery(int sda_pin, int scl_pin);

// Sensor ToF Laser VL53L1X com sequenciamento de boot XSHUT (0x29 -> 0x30 / 0x31)
class ToFVL53L1X {
public:
    ToFVL53L1X(int xshut_pin, uint8_t target_i2c_addr)
        : xshut_(xshut_pin), addr_(target_i2c_addr), initialized_(false), last_dist_mm_(NAN) {}

    void set_xshut(bool active_high);
    bool init_and_readdress();
    float get_distance_mm();
    bool is_ready() const { return initialized_; }
    uint8_t address() const { return addr_; }

private:
    int xshut_;
    uint8_t addr_;
    bool initialized_;
    float last_dist_mm_;
};

} // namespace aegis::sensors
