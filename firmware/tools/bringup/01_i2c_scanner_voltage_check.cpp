// Bringup 01 — Scanner I2C + Sequenciamento XSHUT (0x30/0x31) + Checagem ATS ADC e E-Stop
#include <Arduino.h>
#include <Wire.h>
#include "../../src/board_pins.h"
#include "../../src/sensors/sensor_tof_vl53l1x.h"

void setup() {
    Serial.begin(921600);
    delay(400);
    pinMode(AegisPins::ESTOP_SENSE_IN, INPUT_PULLUP);
    analogReadResolution(12);
    analogSetPinAttenuation(AegisPins::V_MAINS_SENSE, ADC_11db);
    analogSetPinAttenuation(AegisPins::V_UPS_BATT_SENSE, ADC_11db);

    aegis::sensors::i2c_bus_recovery(AegisPins::I2C_SDA, AegisPins::I2C_SCL);

    aegis::sensors::ToFVL53L1X tof_x(AegisPins::TOF_X_XSHUT, 0x30);
    aegis::sensors::ToFVL53L1X tof_y(AegisPins::TOF_Y_XSHUT, 0x31);
    tof_x.set_xshut(false);
    tof_y.set_xshut(false);
    delay(15);
    const bool ok_x = tof_x.init_and_readdress();
    const bool ok_y = tof_y.init_and_readdress();
    Serial.printf("BRINGUP 01 | ToF X(0x30)=%d | ToF Y(0x31)=%d\n", ok_x ? 1 : 0, ok_y ? 1 : 0);
}

void loop() {
    const float v_mains = (analogReadMilliVolts(AegisPins::V_MAINS_SENSE) / 1000.0f) * 4.90f;
    const float v_batt  = (analogReadMilliVolts(AegisPins::V_UPS_BATT_SENSE) / 1000.0f) * 3.20f;
    const bool estop    = (digitalRead(AegisPins::ESTOP_SENSE_IN) == HIGH);

    Serial.printf("ADC: V_MAINS=%.2fV | V_BATT=%.2fV | ESTOP=%s | I2C ACK:",
                  v_mains, v_batt, estop ? "ACTIVE(HIGH)" : "OK(LOW)");
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf(" 0x%02X", addr);
        }
    }
    Serial.println();
    delay(1500);
}
