// Bringup 04 — Varredura Segura Cremalheira M1.25 + AS5600 + INA219 -> Log ActuatorNet
#include <Arduino.h>
#include <Wire.h>
#include "../../src/board_pins.h"
#include "../../src/sensors/sensor_security_power.h"
#include "../../src/mechatronics/lock_controller.h"

static aegis::sensors::SecurityPowerMonitor sec;
static aegis::control::LockController ctrl;

void setup() {
    Serial.begin(921600);
    Wire.begin(AegisPins::I2C_SDA, AegisPins::I2C_SCL, 400000);
    sec.init_hardware_i2c();
    ledcSetup(0, 20000, 10); ledcAttachPin(AegisPins::MOTOR_IN1, 0);
    ledcSetup(1, 20000, 10); ledcAttachPin(AegisPins::MOTOR_IN2, 1);
    Serial.println("seq_id,t_ms,pwm_permille,pos_mm,current_ma,v_bus");
    ctrl.set_target(28.0f);
}

void loop() {
    static uint32_t t0 = millis();
    aegis::sensors::PowerStats pwr{};
    float pos_mm = 0.0f;
    sec.read_ina219(pwr);
    sec.read_as5600_bolt_mm(pos_mm);
    const int16_t u = (pwr.current_ma > 1200.0f) ? 0 : ctrl.compute(pos_mm, micros(), 0.01f, pwr.current_ma);

    const int duty = min(1023, max(0, static_cast<int>(abs(u) * 1023L / 1000L)));
    if (u >= 0) { ledcWrite(0, 1023); ledcWrite(1, 1023 - duty); }
    else        { ledcWrite(0, 1023 - duty); ledcWrite(1, 1023); }

    Serial.printf("1,%lu,%d,%.2f,%.1f,%.2f\n",
                  static_cast<unsigned long>(millis() - t0), u, pos_mm, pwr.current_ma, pwr.bus_voltage_v);
    if (millis() - t0 > 1800) {
        ctrl.set_target(0.0f);
    }
    delay(10);
}
