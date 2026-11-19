/**
 * @file comet_air_mouse_wand_c3.ino
 * @brief Firmware da Varinha Token 2FA & Air Mouse BLE HID (ESP32-C3 + MPU6050)
 *        Integrado de `COMET-AIR-MOUSE-main.zip` (Z05) para o Ecossistema AEGIS-MECH S3 v3.0.
 *
 * Pinout Hardware ESP32-C3:
 *   - MPU6050 SDA  -> GPIO 5
 *   - MPU6050 SCL  -> GPIO 4
 *   - Left Button  -> GPIO 3 (Short press = Left Click | Hold >= 180 ms = Scroll Down)
 *   - Right Button -> GPIO 2 (Short press = Right Click | Hold >= 180 ms = Scroll Up)
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

static constexpr int PIN_C3_SDA       = 5;
static constexpr int PIN_C3_SCL       = 4;
static constexpr int PIN_BTN_LEFT     = 3;
static constexpr int PIN_BTN_RIGHT    = 2;
static constexpr uint8_t MPU6050_ADDR = 0x68;

static constexpr float EMA_SMOOTHING  = 0.82f;
static constexpr float SENSITIVITY    = 0.20f;
static constexpr float DEADZONE_DPS   = 1.0f;
static constexpr uint32_t SCROLL_HOLD_MS = 180;

static float gBiasGx = 0.0f;
static float gBiasGy = 0.0f;
static float gBiasGz = 0.0f;
static float gSmoothDx = 0.0f;
static float gSmoothDy = 0.0f;

static void readMpu6050GyroDps(float& gx, float& gy, float& gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<int>(MPU6050_ADDR), 6, true);
    if (Wire.available() >= 6) {
        const int16_t rx = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        const int16_t ry = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        const int16_t rz = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        gx = static_cast<float>(rx) / 131.0f;
        gy = static_cast<float>(ry) / 131.0f;
        gz = static_cast<float>(rz) / 131.0f;
    } else {
        gx = 0.0f; gy = 0.0f; gz = 0.0f;
    }
}

static bool calibrateGyro500ThreeAxis() {
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    double sumSqX = 0.0, sumSqY = 0.0, sumSqZ = 0.0;

    for (int i = 0; i < 500; ++i) {
        float gx, gy, gz;
        readMpu6050GyroDps(gx, gy, gz);
        sumX += gx; sumY += gy; sumZ += gz;
        sumSqX += gx * gx; sumSqY += gy * gy; sumSqZ += gz * gz;
        delay(2);
    }
    gBiasGx = static_cast<float>(sumX / 500.0);
    gBiasGy = static_cast<float>(sumY / 500.0);
    gBiasGz = static_cast<float>(sumZ / 500.0);

    const float varX = static_cast<float>(sumSqX / 500.0) - gBiasGx * gBiasGx;
    const float varY = static_cast<float>(sumSqY / 500.0) - gBiasGy * gBiasGy;
    const float varZ = static_cast<float>(sumSqZ / 500.0) - gBiasGz * gBiasGz;
    const float maxVar = fmaxf(varX, fmaxf(varY, varZ));
    return maxVar <= 2.0f;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_C3_SDA, PIN_C3_SCL, 400000);
    pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
    pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

    // Wake MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    const bool calibOk = calibrateGyro500ThreeAxis();
    Serial.printf("{\"device\":\"COMET_AIRMOUSE_C3\",\"calib_3axis_ok\":%s}\n", calibOk ? "true" : "false");
}

void loop() {
    float gx, gy, gz;
    readMpu6050GyroDps(gx, gy, gz);

    float adjY = gy - gBiasGy;
    float adjZ = -(gz - gBiasGz); // Gyro Z -> Mouse X invertido
    if (fabsf(adjY) < DEADZONE_DPS) adjY = 0.0f;
    if (fabsf(adjZ) < DEADZONE_DPS) adjZ = 0.0f;

    gSmoothDx = EMA_SMOOTHING * gSmoothDx + (1.0f - EMA_SMOOTHING) * (adjZ * SENSITIVITY);
    gSmoothDy = EMA_SMOOTHING * gSmoothDy + (1.0f - EMA_SMOOTHING) * (adjY * SENSITIVITY);

    const bool leftDown  = (digitalRead(PIN_BTN_LEFT) == LOW);
    const bool rightDown = (digitalRead(PIN_BTN_RIGHT) == LOW);

    Serial.printf(
        "{\"dx\":%d,\"dy\":%d,\"left\":%d,\"right\":%d}\n",
        static_cast<int>(roundf(gSmoothDx)),
        static_cast<int>(roundf(gSmoothDy)),
        leftDown ? 1 : 0,
        rightDown ? 1 : 0
    );
    delay(10);
}
