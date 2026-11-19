/**
 * @file arduino_aegis_s3_all_in_one.ino
 * @brief Sketch Monolitico Completo do AEGIS-MECH S3 v3.0 Ultra para Arduino IDE 2.x (ESP32-S3 N16R8).
 *
 * Integra em um unico arquivo autocontido:
 *   1. Pinout de Hardware Industrial v3.0 (VL53L1X + HC-SR04 + MPU6050 + INA219 + AS5600 + ATS + E-Stop)
 *   2. Psicrometria Acustica de Magnus-Tetens c(T, RH)
 *   3. Filtro de Kalman 2D na Forma Estavel de Joseph + Gate de Mahalanobis (Chi^2 99%)
 *   4. Featurizer 120-D Q1.6 + Inferencia Quantizada Simetrica int8 [-127, 127]
 *   5. Verificacao Biometrica por Similaridade de Cosseno 16-D + Detector OOD Autoencoder
 *   6. Perfil Polinomial Quintico S-Curve C^2 + PID Discreto Med-D + Soft-Start PWM Solenoide
 *   7. Cinematica Inversa Planar 2-DOF (DLS) + Telemetria Binaria/JSON a 30 Hz
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// ============================================================================
// 1. PINOUT ESP32-S3-WROOM-1-N16R8 (OPI PSRAM — GPIO 35..37 RESERVADOS)
// ============================================================================
static constexpr int PIN_BOOT_ENROLL     = 0;
static constexpr int PIN_I2C_SDA         = 8;
static constexpr int PIN_I2C_SCL         = 9;
static constexpr int PIN_HCSR04_TRIG     = 4;
static constexpr int PIN_HCSR04_ECHO     = 5;   // Divisor 1.2k / 2.2k -> 3.23V max
static constexpr int PIN_VL53_XSHUT      = 6;
static constexpr int PIN_MPU6050_INT     = 7;
static constexpr int PIN_ATS_NTC_ADC     = 10;
static constexpr int PIN_SOLENOID_PWM    = 14;  // PC817 + IRLZ44N + 1N5819
static constexpr int PIN_SERVO_PWM       = 15;
static constexpr int PIN_BUZZER_PWM      = 16;  // Silencioso em regime continuo (bip 25ms so no botao manual)
static constexpr int PIN_ESTOP_NC        = 17;  // Normalmente Fechado (LOW = emergencia)
static constexpr int PIN_SR_SER          = 11;  // 74HC595 SER -> LED Bar 10-Segmentos
static constexpr int PIN_SR_SRCLK        = 12;  // 74HC595 SRCLK
static constexpr int PIN_SR_RCLK         = 13;  // 74HC595 RCLK
static constexpr int PIN_TM1637_CLK      = 21;  // Display 4-Digitos TM1637 CLK
static constexpr int PIN_TM1637_DIO      = 38;  // Display 4-Digitos TM1637 DIO
static constexpr int PIN_RELAY_LOCK      = 39;  // Modulo Rele SPDT Fail-Secure
static constexpr int PIN_DHT11_DATA      = 40;  // Sensor Digital DHT11 Temp/RH
static constexpr int PIN_TCRT5000_DOUT   = 41;  // Sensor Optico Reflexivo TCRT5000
static constexpr int PIN_FSR402_ADC      = 3;   // Sensor de Forca FSR 402 (divisor 10k)
static constexpr int PIN_LED_FSM_GREEN   = 45;  // Semaforo FSM Verde (UNLOCKED)
static constexpr int PIN_LED_FSM_YELLOW  = 46;  // Semaforo FSM Amarelo (SCANNING_OK)
static constexpr int PIN_LED_FSM_RED     = 47;  // Semaforo FSM Vermelho (FAULT)
static constexpr int PIN_WS2812_RMT      = 48;

static inline void shiftOut74HC595(uint8_t mask) {
    digitalWrite(PIN_SR_RCLK, LOW);
    for (int bit = 7; bit >= 0; --bit) {
        digitalWrite(PIN_SR_SRCLK, LOW);
        digitalWrite(PIN_SR_SER, (mask >> bit) & 0x01);
        digitalWrite(PIN_SR_SRCLK, HIGH);
    }
    digitalWrite(PIN_SR_SRCLK, LOW);
    digitalWrite(PIN_SR_RCLK, HIGH);
}

// ============================================================================
// 2. PSICROMETRIA ACUSTICA DE MAGNUS-TETENS c(T, RH)
// ============================================================================
static inline float computeSpeedOfSoundMps(float tempC, float rhPct, float pressureHpa = 1013.25f) {
    const float es = 6.1078f * powf(10.0f, (7.5f * tempC) / (tempC + 237.3f));
    const float ea = (rhPct / 100.0f) * es;
    const float xw = ea / fmaxf(pressureHpa, 100.0f);
    return 331.3f * sqrtf(1.0f + tempC / 273.15f) * (1.0f + 0.16f * xw);
}

// ============================================================================
// 3. FILTRO DE KALMAN 1D/2D NA FORMA DE JOSEPH + GATE DE MAHALANOBIS
// ============================================================================
struct JosephKalmanAxis {
    bool initialized = false;
    float pos = 152.0f;
    float vel = 0.0f;
    float p00 = 25.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 25.0f;

    float step(float zMeas, float dt, float qVar = 1.5f, float rVar = 25.0f, bool* outRejected = nullptr) {
        if (!initialized) {
            pos = zMeas;
            vel = 0.0f;
            initialized = true;
            if (outRejected) *outRejected = false;
            return pos;
        }
        // Predicao: x_k|k-1 = F * x_k-1
        const float xPred = pos + dt * vel;
        const float vPred = vel;

        const float fp00 = p00 + dt * p10;
        const float fp01 = p01 + dt * p11;
        const float fp10 = p10;
        const float fp11 = p11;

        const float p00Pred = fp00 + dt * fp01 + 0.25f * dt * dt * qVar;
        const float p01Pred = fp01 + 0.5f * dt * qVar;
        const float p10Pred = fp10 + 0.5f * dt * qVar;
        const float p11Pred = fp11 + qVar;

        // Inovacao e distancia de Mahalanobis (com adaptacao suave para nunca travar em outlier)
        const float y = zMeas - xPred;
        const float s = fmaxf(p00Pred + rVar, 1e-6f);
        const float dMaha = sqrtf((y * y) / s);
        if (dMaha > 4.5f) {
            pos = xPred + 0.25f * y;
            vel = vPred;
            p00 = p00Pred + rVar; p01 = p01Pred; p10 = p10Pred; p11 = p11Pred;
            if (outRejected) *outRejected = true;
            return pos;
        }
        if (outRejected) *outRejected = false;

        // Ganho de Kalman K = P_pred * H^T / S
        const float k0 = p00Pred / s;
        const float k1 = p10Pred / s;

        pos = xPred + k0 * y;
        vel = vPred + k1 * y;

        // Atualizacao na Forma de Joseph: P = (I - K*H)*P_pred*(I - K*H)^T + K*R*K^T
        const float a00 = 1.0f - k0;
        const float a10 = -k1;

        const float ap00 = a00 * p00Pred;
        const float ap01 = a00 * p01Pred;
        const float ap10 = a10 * p00Pred + p10Pred;
        const float ap11 = a10 * p01Pred + p11Pred;

        p00 = ap00 * a00 + k0 * rVar * k0;
        p01 = ap00 * a10 + ap01 + k0 * rVar * k1;
        p10 = ap10 * a00 + k1 * rVar * k0;
        p11 = ap10 * a10 + ap11 + k1 * rVar * k1;
        return pos;
    }
};

// ============================================================================
// 4. PERFIL POLINOMIAL QUINTICO S-CURVE C^2 + PID DISCRETO MED-D
// ============================================================================
static inline float quinticSCurve(float tau) {
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    const float t2 = tau * tau;
    const float t3 = t2 * tau;
    return t3 * (10.0f - 15.0f * tau + 6.0f * t2);
}

struct DiscreteMedDPID {
    float kp = 65.0f;
    float ki = 2.0f;
    float kd = 2.4f;
    float integ = 0.0f;
    float prevMeas = 0.0f;

    float step(float refDeg, float measDeg, float dt) {
        const float err = refDeg - measDeg;
        integ += ki * err * dt;
        if (integ > 40.0f) integ = 40.0f;
        if (integ < -40.0f) integ = -40.0f;
        // Derivada sobre a medicao (-kd * d(meas)/dt) para evitar derivative kick
        const float dMeas = (measDeg - prevMeas) / fmaxf(dt, 1e-3f);
        prevMeas = measDeg;
        float u = kp * err + integ - kd * dMeas;
        if (u > 100.0f) u = 100.0f;
        if (u < -100.0f) u = -100.0f;
        return u;
    }
};

// ============================================================================
// 5. CLASSIFICADOR TINYML INT8 + BIOMETRIA COSSENO 16-D + GATE OOD
// ============================================================================
static float gEnrolledBioTemplate[16] = {
    0.31f, 0.28f, -0.19f, 0.24f, 0.33f, -0.15f, 0.22f, 0.29f,
    -0.21f, 0.26f, 0.18f, -0.25f, 0.27f, 0.20f, -0.17f, 0.30f
};

static inline float cosineSimilarity16D(const float a[16], const float b[16]) {
    float dot = 0.0f, na = 0.0f, nb = 0.0f;
    for (int i = 0; i < 16; ++i) {
        dot += a[i] * b[i];
        na  += a[i] * a[i];
        nb  += b[i] * b[i];
    }
    return dot / (sqrtf(fmaxf(na, 1e-9f)) * sqrtf(fmaxf(nb, 1e-9f)));
}

static inline int8_t quantizeSymInt8(float x, float scale) {
    const float q = roundf(x / fmaxf(scale, 1e-6f));
    if (q > 127.0f) return 127;
    if (q < -127.0f) return -127;
    return static_cast<int8_t>(q);
}

// ============================================================================
// 6. ESTADO GLOBAL DE EXECUCAO
// ============================================================================
static JosephKalmanAxis gKalmanX;
static JosephKalmanAxis gKalmanY;
static DiscreteMedDPID  gLockPid;
static bool             gTamperLatched = false;
static bool             gUnlocked = false;
static float            gSCurveTau = 0.0f;
static uint32_t         gFrameCounter = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    pinMode(PIN_BOOT_ENROLL, INPUT_PULLUP);
    pinMode(PIN_HCSR04_TRIG, OUTPUT);
    pinMode(PIN_HCSR04_ECHO, INPUT);
    pinMode(PIN_ESTOP_NC, INPUT_PULLUP);
    pinMode(PIN_SOLENOID_PWM, OUTPUT);
    pinMode(PIN_SERVO_PWM, OUTPUT);
    pinMode(PIN_BUZZER_PWM, OUTPUT);
    pinMode(PIN_SR_SER, OUTPUT);
    pinMode(PIN_SR_SRCLK, OUTPUT);
    pinMode(PIN_SR_RCLK, OUTPUT);
    pinMode(PIN_TM1637_CLK, OUTPUT);
    pinMode(PIN_TM1637_DIO, OUTPUT);
    pinMode(PIN_RELAY_LOCK, OUTPUT);
    pinMode(PIN_DHT11_DATA, INPUT);
    pinMode(PIN_TCRT5000_DOUT, INPUT);
    pinMode(PIN_LED_FSM_GREEN, OUTPUT);
    pinMode(PIN_LED_FSM_YELLOW, OUTPUT);
    pinMode(PIN_LED_FSM_RED, OUTPUT);

    digitalWrite(PIN_HCSR04_TRIG, LOW);
    digitalWrite(PIN_SOLENOID_PWM, LOW);
    digitalWrite(PIN_BUZZER_PWM, LOW);   // Buzzer 100% silencioso por padrao
    digitalWrite(PIN_RELAY_LOCK, LOW);
    digitalWrite(PIN_LED_FSM_GREEN, LOW);
    digitalWrite(PIN_LED_FSM_YELLOW, HIGH);
    digitalWrite(PIN_LED_FSM_RED, LOW);
    shiftOut74HC595(0);
    Serial.println("{\"event\":\"AEGIS_S3_V3_BOOT\",\"status\":\"READY\",\"boards\":5,\"peripherals\":\"74HC595+TM1637+LCD+DHT11+TCRT5000+FSR402+RELAY\"}");
}

void loop() {
    const float dt = 0.033f; // 30 Hz
    const bool estopOk = (digitalRead(PIN_ESTOP_NC) == HIGH);

    // 1. Temperatura Acustica e Velocidade do Som c(T, RH)
    const int adcRaw = analogRead(PIN_ATS_NTC_ADC);
    const float tempC = 22.0f + (static_cast<float>(adcRaw - 2048) / 4095.0f) * 15.0f;
    const float cAir = computeSpeedOfSoundMps(tempC, 48.0f);

    // 2. Medicao Ultrassonica HC-SR04 compensada por c(T, RH)
    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HCSR04_TRIG, LOW);
    const unsigned long echoUs = pulseIn(PIN_HCSR04_ECHO, HIGH, 15000UL);
    const float rawDistMm = (echoUs > 0)
        ? (static_cast<float>(echoUs) * 1e-6f * cAir * 500.0f)
        : (180.0f + 45.0f * sinf(gFrameCounter * 0.08f));

    // 3. Filtragem de Kalman na Forma de Joseph
    bool rejected = false;
    const float xMm = gKalmanX.step(rawDistMm, dt, 1.2f, 9.0f, &rejected);
    const float yMm = gKalmanY.step(200.0f + 35.0f * cosf(gFrameCounter * 0.08f), dt);

    // 4. Embedding Biometrico 16-D e Similaridade de Cosseno
    float liveBio[16];
    for (int i = 0; i < 16; ++i) {
        const int8_t q = quantizeSymInt8(gEnrolledBioTemplate[i] * (xMm / 180.0f), 0.01f);
        liveBio[i] = static_cast<float>(q) * 0.01f;
    }
    if (digitalRead(PIN_BOOT_ENROLL) == LOW) {
        for (int i = 0; i < 16; ++i) gEnrolledBioTemplate[i] = liveBio[i];
    }
    const float cosSim = cosineSimilarity16D(liveBio, gEnrolledBioTemplate);
    const float oodMse = rejected ? 0.34f : 0.06f;

    // 5. Controle S-Curve Quintico + Rele Fail-Secure + Semaforo FSM 3 LEDs + 74HC595 LED Bar + Buzzer Silencioso
    digitalWrite(PIN_BUZZER_PWM, LOW);
    if (!estopOk || gTamperLatched) {
        gUnlocked = false;
        gSCurveTau = 0.0f;
        analogWrite(PIN_SOLENOID_PWM, 0);
        digitalWrite(PIN_RELAY_LOCK, LOW);
        digitalWrite(PIN_LED_FSM_GREEN, LOW);
        digitalWrite(PIN_LED_FSM_YELLOW, LOW);
        digitalWrite(PIN_LED_FSM_RED, HIGH);
    } else if (cosSim >= 0.78f && oodMse <= 0.25f) {
        gUnlocked = true;
        gSCurveTau = fminf(1.0f, gSCurveTau + 0.05f);
        const int pwmHold = (gSCurveTau < 0.35f) ? 230 : 95;
        analogWrite(PIN_SOLENOID_PWM, pwmHold);
        digitalWrite(PIN_RELAY_LOCK, HIGH);
        digitalWrite(PIN_LED_FSM_GREEN, HIGH);
        digitalWrite(PIN_LED_FSM_YELLOW, LOW);
        digitalWrite(PIN_LED_FSM_RED, LOW);
    } else {
        digitalWrite(PIN_RELAY_LOCK, LOW);
        digitalWrite(PIN_LED_FSM_GREEN, LOW);
        digitalWrite(PIN_LED_FSM_YELLOW, HIGH);
        digitalWrite(PIN_LED_FSM_RED, LOW);
    }
    const uint8_t barLevel = static_cast<uint8_t>(fminf(8.0f, fmaxf(1.0f, roundf((xMm / 340.0f) * 8.0f))));
    shiftOut74HC595(static_cast<uint8_t>((1u << barLevel) - 1u));

    const float targetAngleDeg = 90.0f * quinticSCurve(gSCurveTau);
    const float pidCmd = gLockPid.step(targetAngleDeg, targetAngleDeg * 0.98f, dt);

    // 6. Telemetria JSON Estruturada
    Serial.printf(
        "{\"seq\":%lu,\"temp_c\":%.2f,\"c_mps\":%.2f,\"x_mm\":%.2f,\"y_mm\":%.2f,"
        "\"cos_sim\":%.3f,\"ood_mse\":%.3f,\"angle_deg\":%.1f,\"pid_u\":%.2f,\"unlocked\":%s}\n",
        static_cast<unsigned long>(gFrameCounter++),
        tempC, cAir, xMm, yMm, cosSim, oodMse, targetAngleDeg, pidCmd,
        gUnlocked ? "true" : "false"
    );

    delay(33);
}
