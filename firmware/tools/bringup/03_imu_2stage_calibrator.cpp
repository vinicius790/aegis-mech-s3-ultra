// Bringup 03 — Calibracao Dual 500 Amostras 3 Eixos IMU MPU6050 + Persistência NVS
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "../../src/board_pins.h"
#include "../../src/sensors/sensor_security_power.h"

static aegis::sensors::SecurityPowerMonitor sec;
static aegis::sensors::IMURawData samples[500];
static Preferences prefs;

void setup() {
    Serial.begin(921600);
    Wire.begin(AegisPins::I2C_SDA, AegisPins::I2C_SCL, 400000);
    prefs.begin("aegis_cal", false);
    sec.init_hardware_i2c();

    for (int i = 0; i < 500; ++i) {
        sec.read_mpu6050(samples[i]);
        delay(2);
    }
    if (sec.calibrate_gyro_500(samples)) {
        Serial.println("BRINGUP 03: Calibracao 3-eixos APROVADA (var <= 2.0).");
    } else {
        Serial.println("BRINGUP 03: REJEITADA por vibracao em X/Y/Z! Carregando fallback NVS.");
        sec.set_gyro_bias_fallback(prefs.getFloat("gx", 0.0f), prefs.getFloat("gy", 0.0f), prefs.getFloat("gz", 0.0f));
    }
}

void loop() {
    aegis::sensors::IMURawData raw{};
    if (sec.read_mpu6050(raw)) {
        sec.update_imu(raw, 0.82f);
        Serial.printf("TamperScore=%.3f | A=(%.2f,%.2f,%.2f)g | G=(%.1f,%.1f,%.1f)dps\n",
                      sec.compute_tamper_score(), raw.ax_g, raw.ay_g, raw.az_g, raw.gx_dps, raw.gy_dps, raw.gz_dps);
    }
    delay(20);
}
