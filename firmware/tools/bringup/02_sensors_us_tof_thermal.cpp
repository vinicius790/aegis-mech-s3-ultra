// Bringup 02 — Teste Acustico Ping-Pong RMT (60ms/eixo, 38ms timeout) + ToF + DS18B20
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "../../src/board_pins.h"
#include "../../src/sensors/sensor_ultrasonic_rmt.h"
#include "../../src/sensors/sensor_tof_vl53l1x.h"

static aegis::sensors::UltrasonicRMT us_x(AegisPins::US_X_TRIG, AegisPins::US_X_ECHO, 0, 4);
static aegis::sensors::UltrasonicRMT us_y(AegisPins::US_Y_TRIG, AegisPins::US_Y_ECHO, 1, 5);
static aegis::sensors::ToFVL53L1X    tof_x(AegisPins::TOF_X_XSHUT, 0x30);
static aegis::sensors::ToFVL53L1X    tof_y(AegisPins::TOF_Y_XSHUT, 0x31);
static OneWire one_wire(AegisPins::DS18B20_DQ);
static DallasTemperature ds18(&one_wire);

void setup() {
    Serial.begin(921600);
    Wire.begin(AegisPins::I2C_SDA, AegisPins::I2C_SCL, 400000);
    ds18.begin();
    ds18.setWaitForConversion(false);
    ds18.requestTemperatures();
    tof_x.set_xshut(false); tof_y.set_xshut(false); delay(10);
    tof_x.init_and_readdress();
    tof_y.init_and_readdress();
    us_x.init();
    us_y.init();
    Serial.println("BRINGUP 02: Sensores Acustico-Opticos Ping-Pong Inicializados");
}

void loop() {
    const float t_c = ds18.getTempCByIndex(0);
    const float temp = (t_c > -20.0f && t_c < 85.0f) ? t_c : 25.0f;
    ds18.requestTemperatures();

    us_x.trigger_pulse(micros());
    delay(30); // Intervalo estrito 30 ms anti-diafonia antes de Y!
    const float ux = us_x.read_distance_mm(temp, 50.0f);

    us_y.trigger_pulse(micros());
    delay(30);
    const float uy = us_y.read_distance_mm(temp, 50.0f);

    const float tx = tof_x.get_distance_mm();
    const float ty = tof_y.get_distance_mm();
    Serial.printf("T=%.2fC | US=(%.1f, %.1f) mm | ToF=(%.1f, %.1f) mm\n", temp, ux, uy, tx, ty);
}
