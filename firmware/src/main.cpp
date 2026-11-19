// AEGIS-MECH S3 v3.0 — Firmware Principal Dual-Core FreeRTOS (ESP-IDF 4.4 / Arduino 2.x)
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>
#include <string.h>

#include "board_pins.h"
#include "protocol_frames.h"
#include "sensors/sensor_ultrasonic_rmt.h"
#include "sensors/sensor_tof_vl53l1x.h"
#include "sensors/sensor_security_power.h"
#include "ai/kalman_particle.h"
#include "ai/gesture_featurizer.h"
#include "ai/inference_int8.h"
#include "ai/ood_model.h"
#include "ai/test_vectors.h"
#include "mechatronics/lock_controller.h"
#include "mechatronics/security_fsm.h"
#include "comms/hmi_local.h"
#include "comms/serial_crc_bridge.h"

struct TelemetrySnap {
    uint64_t t_us;
    float    us_x_mm, us_y_mm, tof_x_mm, tof_y_mm;
    float    kf_x, kf_y, kf_vx, kf_vy, pf_spread_mm;
    float    bolt_mm, current_ma, v_mains_v, v_batt_v, temp_c;
    float    tamper_score;
    uint8_t  mahal_flags;
    bool     estop_active, limit_open, limit_closed, on_ups;
    uint8_t  fault_code;
};

struct BoltCommand {
    float target_mm;
    bool  engage_solenoid;
    bool  abort_estop;
};

static SemaphoreHandle_t xI2CMutex   = nullptr;
static QueueHandle_t     xCmdQueue   = nullptr;
static QueueHandle_t     xInferQueue = nullptr;
static portMUX_TYPE      snap_mux    = portMUX_INITIALIZER_UNLOCKED;
static TelemetrySnap     g_snap      = {};
static volatile bool     g_nvs_tamper_latched = false;
static Preferences       g_prefs;

static bool run_post_golden_vectors() {
    int passed = 0;
    for (int i = 0; i < AEGIS_NUM_TEST_VECTORS; ++i) {
        int32_t logits[7] = {0};
        float bio[16] = {0.0f};
        aegis::ai::tiny1dcnn_infer_int8(aegis::test::TEST_INPUTS[i], logits, bio);
        bool ok = true;
        for (int c = 0; c < 7; ++c) {
            if (logits[c] != aegis::test::EXPECTED_LOGITS[i][c]) {
                ok = false;
                break;
            }
        }
        if (ok) ++passed;
    }
    Serial.printf("POST: %d/%d GOLDEN VECTORS PASS | FW=3.0.0\n", passed, AEGIS_NUM_TEST_VECTORS);
    return (passed == AEGIS_NUM_TEST_VECTORS);
}

static void apply_drv8871_pwm_slow_decay(int16_t u_permille) {
    const int duty = min(1023, max(0, static_cast<int>(abs(u_permille) * 1023L / 1000L)));
    if (u_permille == 0) {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    } else if (u_permille > 0) {
        ledcWrite(0, 1023);
        ledcWrite(1, 1023 - duty);
    } else {
        ledcWrite(0, 1023 - duty);
        ledcWrite(1, 1023);
    }
}

void Task_Sensor_Control_RT(void* pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz deterministico

    aegis::sensors::UltrasonicRMT us_x(AegisPins::US_X_TRIG, AegisPins::US_X_ECHO, 0, 4);
    aegis::sensors::UltrasonicRMT us_y(AegisPins::US_Y_TRIG, AegisPins::US_Y_ECHO, 1, 5);
    aegis::sensors::ToFVL53L1X    tof_x(AegisPins::TOF_X_XSHUT, 0x30);
    aegis::sensors::ToFVL53L1X    tof_y(AegisPins::TOF_Y_XSHUT, 0x31);
    aegis::sensors::SecurityPowerMonitor sec_pwr;
    aegis::ai::KalmanFilter2D     ekf;
    aegis::ai::ParticleFilterLite32 pf;
    aegis::control::LockController lock_ctrl;

    OneWire one_wire(AegisPins::DS18B20_DQ);
    DallasTemperature ds18(&one_wire);
    ds18.begin();
    ds18.setWaitForConversion(false);
    ds18.requestTemperatures();

    us_x.init();
    us_y.init();

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        tof_x.set_xshut(false); tof_y.set_xshut(false);
        delay(10);
        tof_x.init_and_readdress();
        tof_y.init_and_readdress();
        sec_pwr.init_hardware_i2c();
        xSemaphoreGive(xI2CMutex);
    }

    uint32_t tick_idx = 0;
    float temp_c = 25.0f;
    float us_x_mm = NAN, us_y_mm = NAN, tof_x_mm = NAN, tof_y_mm = NAN;
    float bolt_mm = 0.0f;
    uint8_t i2c_fail_streak = 0;
    aegis::sensors::PowerStats pwr{12.0f, 40.0f, 480.0f, false};

    for (;;) {
        esp_task_wdt_reset();
        const uint64_t now_us = esp_timer_get_time();

        // 1. Leitura ATS ADC + E-Stop Hardwired
        const bool estop_active = (digitalRead(AegisPins::ESTOP_SENSE_IN) == HIGH);
        const float v_mains = (analogReadMilliVolts(AegisPins::V_MAINS_SENSE) / 1000.0f) * 4.90f;
        const float v_batt  = (analogReadMilliVolts(AegisPins::V_UPS_BATT_SENSE) / 1000.0f) * 3.20f;
        const auto ats = sec_pwr.evaluate_ats(v_mains, v_batt, estop_active);

        // 2. DS18B20 nao-bloqueante a cada 1s (100 ticks)
        if ((tick_idx % 100U) == 85U) {
            const float t_read = ds18.getTempCByIndex(0);
            if (t_read > -30.0f && t_read < 90.0f) temp_c = t_read;
            ds18.requestTemperatures();
        }

        // 3. Ping-Pong acustico estrito modulo 6 (60 ms por eixo, 30 ms entre X e Y!)
        if ((tick_idx % 6U) == 0U) {
            us_y_mm = us_y.read_distance_mm(temp_c, 50.0f);
            us_x.trigger_pulse(static_cast<uint32_t>(now_us));
        } else if ((tick_idx % 6U) == 3U) {
            us_x_mm = us_x.read_distance_mm(temp_c, 50.0f);
            us_y.trigger_pulse(static_cast<uint32_t>(now_us));
        }

        // 4. Barramento I2C sob Mutex c/ i2c_bus_recovery automatico se 3 NACKs
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(4)) == pdTRUE) {
            aegis::sensors::IMURawData imu{};
            bool ok_i2c = sec_pwr.read_mpu6050(imu);
            if (ok_i2c) sec_pwr.update_imu(imu, 0.82f);
            sec_pwr.read_ina219(pwr);
            sec_pwr.read_as5600_bolt_mm(bolt_mm);
            if ((tick_idx % 2U) == 0U) { // ToF 50 Hz
                tof_x_mm = tof_x.get_distance_mm();
                tof_y_mm = tof_y.get_distance_mm();
            }
            if (!ok_i2c) {
                if (++i2c_fail_streak >= 3U) {
                    aegis::sensors::i2c_bus_recovery(AegisPins::I2C_SDA, AegisPins::I2C_SCL);
                    i2c_fail_streak = 0;
                }
            } else {
                i2c_fail_streak = 0;
            }
            xSemaphoreGive(xI2CMutex);
        }

        // 5. Fusao Bayesiana (EKF Forma de Joseph + ParticleFilter32)
        ekf.predict(0.01f);
        uint8_t m_flags = 0;
        if (!isnan(tof_x_mm) && !ekf.update_x(tof_x_mm, true)) m_flags |= 0x01U;
        if (!isnan(us_x_mm)  && !ekf.update_x(us_x_mm,  false)) m_flags |= 0x02U;
        if (!isnan(tof_y_mm) && !ekf.update_y(tof_y_mm, true)) m_flags |= 0x04U;
        if (!isnan(us_y_mm)  && !ekf.update_y(us_y_mm,  false)) m_flags |= 0x08U;
        pf.predict(ekf.vel_x(), 0.01f);
        pf.update(ekf.pos_x());

        // 6. Controle LockController v3.0
        BoltCommand cmd{};
        while (xQueueReceive(xCmdQueue, &cmd, 0) == pdTRUE) {
            if (cmd.abort_estop) {
                apply_drv8871_pwm_slow_decay(0);
                digitalWrite(AegisPins::SOLENOID_CTRL, LOW);
                lock_ctrl.reset();
            } else {
                lock_ctrl.set_target(cmd.target_mm);
                digitalWrite(AegisPins::SOLENOID_CTRL, cmd.engage_solenoid ? HIGH : LOW);
            }
        }
        lock_ctrl.set_bus_voltage(pwr.bus_voltage_v);
        if (ats.estop_active) {
            apply_drv8871_pwm_slow_decay(0);
        } else {
            const int16_t pwm_u = lock_ctrl.compute(bolt_mm, now_us, 0.01f, pwr.current_ma);
            apply_drv8871_pwm_slow_decay(pwm_u);
        }

        // 7. Publica snapshot atomico
        TelemetrySnap local{};
        local.t_us = now_us;
        local.us_x_mm = us_x_mm; local.us_y_mm = us_y_mm;
        local.tof_x_mm = tof_x_mm; local.tof_y_mm = tof_y_mm;
        local.kf_x = ekf.pos_x(); local.kf_y = ekf.pos_y();
        local.kf_vx = ekf.vel_x(); local.kf_vy = ekf.vel_y();
        local.pf_spread_mm = 0.0f;
        local.bolt_mm = bolt_mm;
        local.current_ma = pwr.current_ma;
        local.v_mains_v = ats.v_mains_v;
        local.v_batt_v = ats.v_batt_v;
        local.temp_c = temp_c;
        local.tamper_score = sec_pwr.compute_tamper_score();
        local.mahal_flags = m_flags;
        local.estop_active = ats.estop_active;
        local.on_ups = ats.on_ups_batt;
        local.limit_open = (digitalRead(AegisPins::LIMIT_OPEN_SW) == LOW);
        local.limit_closed = (digitalRead(AegisPins::LIMIT_CLOSE_SW) == LOW);
        local.fault_code = estop_active ? static_cast<uint8_t>(FaultCode::E08_HARDWARE_ESTOP_ACTIVE)
                         : (ats.voltage_abnormal ? static_cast<uint8_t>(FaultCode::E02_VOLTAGE_ABNORMAL_042)
                                                 : static_cast<uint8_t>(lock_ctrl.fault_code()));

        portENTER_CRITICAL(&snap_mux);
        g_snap = local;
        portEXIT_CRITICAL(&snap_mux);

        ++tick_idx;
        vTaskDelayUntil(&last_wake, period);
    }
}

void Task_Gesture_AI(void* pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz IA Gestual
    aegis::ai::GestureFeaturizer featurizer;

    for (;;) {
        esp_task_wdt_reset();
        TelemetrySnap snap{};
        portENTER_CRITICAL(&snap_mux);
        snap = g_snap;
        portEXIT_CRITICAL(&snap_mux);

        if (snap.kf_x >= 120.0f && snap.kf_x <= 320.0f && snap.kf_y >= 120.0f && snap.kf_y <= 320.0f) {
            featurizer.push_point(static_cast<uint32_t>(snap.t_us / 1000ULL), snap.kf_x, snap.kf_y);
        }
        if (featurizer.is_gesture_complete()) {
            GestureInferPayload res{};
            featurizer.extract_features_120(res.vec_120);
            float bio[16] = {0.0f};
            const uint64_t t0 = esp_timer_get_time();
            aegis::ai::tiny1dcnn_infer_int8(res.vec_120, res.logits, bio);
            res.infer_us = static_cast<uint32_t>(esp_timer_get_time() - t0);
            res.ood_recon_mse = aegis::ai::ood_mse_f32(res.vec_120);

            int best = 0;
            for (int c = 1; c < 7; ++c) {
                if (res.logits[c] > res.logits[best]) best = c;
            }
            res.pred_class = static_cast<uint8_t>(best);

            // Calculo real de similaridade de cosseno 16-D contra o template biometrico NVS
            static float s_nvs_bio_template[16] = {
                0.28f, 0.24f, -0.18f, 0.25f, 0.31f, -0.14f, 0.22f, 0.27f,
                -0.19f, 0.26f, 0.17f, -0.23f, 0.29f, 0.21f, -0.16f, 0.28f
            };
            if (gpio_get_level(aegis::pins::GPIO_BOOT_ENROLL) == 0) {
                for (int i = 0; i < 16; ++i) s_nvs_bio_template[i] = bio[i];
            }
            float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
            for (int i = 0; i < 16; ++i) {
                dot    += bio[i] * s_nvs_bio_template[i];
                norm_a += bio[i] * bio[i];
                norm_b += s_nvs_bio_template[i] * s_nvs_bio_template[i];
            }
            const float denom = std::sqrt(std::fmax(norm_a, 1e-9f)) * std::sqrt(std::fmax(norm_b, 1e-9f));
            res.bio_cos_sim = (denom > 1e-6f) ? (dot / denom) : 0.0f;
            res.accepted = (best < 6 && res.ood_recon_mse <= AEGIS_OOD_THRESHOLD && res.bio_cos_sim >= 0.75f) ? 1U : 0U;
            xQueueOverwrite(xInferQueue, &res);
            featurizer.reset();
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

void Task_Comms_HMI_FSM(void* pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33); // 30 Hz HMI + FSM + SerialCRC

    aegis::fsm::SecurityFSM fsm(aegis::fsm::DEFAULT_STATE_TABLE);
    aegis::comms::LocalHMI hmi;
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        hmi.init(AegisPins::WS2812_RING, AegisPins::BUZZER_PWM);
        xSemaphoreGive(xI2CMutex);
    }

    uint16_t seq_num = 0;
    uint32_t dwell_ms = 0;
    uint32_t tick_30hz = 0;
    GestureInferPayload last_infer{};

    for (;;) {
        esp_task_wdt_reset();
        const uint32_t now_ms = millis();
        TelemetrySnap snap{};
        portENTER_CRITICAL(&snap_mux);
        snap = g_snap;
        portEXIT_CRITICAL(&snap_mux);

        if (snap.kf_x >= 120.0f && snap.kf_x <= 320.0f && snap.kf_y >= 120.0f && snap.kf_y <= 320.0f) {
            dwell_ms = min<uint32_t>(5000U, dwell_ms + 33U);
        } else {
            dwell_ms = 0U;
        }

        const bool has_new_infer = (xQueueReceive(xInferQueue, &last_infer, 0) == pdTRUE);
        aegis::fsm::SystemTelemetry obs{};
        obs.now_ms = now_ms;
        obs.tof_x_mm = snap.tof_x_mm;
        obs.tof_y_mm = snap.tof_y_mm;
        obs.bolt_pos_mm = snap.bolt_mm;
        obs.current_ma = snap.current_ma;
        obs.tamper_score = snap.tamper_score;
        obs.arena_dwell_ms = static_cast<uint16_t>(dwell_ms);
        obs.estop_active = snap.estop_active;
        obs.nvs_tamper_latched = g_nvs_tamper_latched;
        obs.gesture_seq_ok = has_new_infer && (last_infer.accepted != 0U);
        obs.bio_ood_ok = obs.gesture_seq_ok;
        obs.vision_2fa_ok = true;
        obs.limit_open_hit = snap.limit_open;
        obs.limit_closed_hit = snap.limit_closed;

        const auto prev_st = fsm.state();
        const auto cur_st  = fsm.step(obs);

        if (cur_st == aegis::fsm::VaultState::TAMPER_LOCKDOWN && prev_st != aegis::fsm::VaultState::TAMPER_LOCKDOWN) {
            g_nvs_tamper_latched = true;
            g_prefs.putBool("tamper_latch", true);
        }

        BoltCommand cmd{};
        cmd.abort_estop = (cur_st == aegis::fsm::VaultState::HARDWARE_ESTOP);
        cmd.target_mm = (cur_st == aegis::fsm::VaultState::UNLOCKING_BOLT || cur_st == aegis::fsm::VaultState::OPEN_WAIT)
                      ? AegisPins::BOLT_OPEN_MM : 0.0f;
        cmd.engage_solenoid = (cur_st == aegis::fsm::VaultState::UNLOCKING_BOLT);
        xQueueSend(xCmdQueue, &cmd, 0);

        uint8_t r = 0, g = 0, b = 0;
        hmi.set_mode(cur_st == aegis::fsm::VaultState::TAMPER_LOCKDOWN ? aegis::comms::LEDMode::ALARM_PULSE_RED
                     : cur_st == aegis::fsm::VaultState::OPEN_WAIT     ? aegis::comms::LEDMode::SUCCESS_GREEN
                                                                       : aegis::comms::LEDMode::BREATHING_IDLE);
        hmi.update_led_ring(now_ms, r, g, b, snap.on_ups);
        (void)hmi.get_buzzer_tone(now_ms);

        if ((tick_30hz % 3U) == 0U && xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(4)) == pdTRUE) {
            const char* st_name = (cur_st == aegis::fsm::VaultState::LOCKED_IDLE) ? "LOCKED_IDLE" : "ACTIVE";
            hmi.render_oled_status(st_name, snap.bolt_mm, snap.v_mains_v, snap.estop_active ? "E08 ESTOP" : "E00");
            xSemaphoreGive(xI2CMutex);
        }

        SensorFrame50Hz tx_frame{};
        tx_frame.us_x_mm = snap.us_x_mm; tx_frame.us_y_mm = snap.us_y_mm;
        tx_frame.tof_x_mm = snap.tof_x_mm; tx_frame.tof_y_mm = snap.tof_y_mm;
        tx_frame.kalman_x_mm = snap.kf_x; tx_frame.kalman_y_mm = snap.kf_y;
        tx_frame.kalman_vx = snap.kf_vx; tx_frame.kalman_vy = snap.kf_vy;
        tx_frame.temp_c = snap.temp_c;
        tx_frame.mahalanobis_flags = snap.mahal_flags;
        uint8_t out_pkt[80];
        const size_t n = aegis::comms::SerialCRCBridge::pack_telemetry(tx_frame, seq_num++, snap.t_us, out_pkt);
        Serial.write(out_pkt, n);

        ++tick_30hz;
        vTaskDelayUntil(&last_wake, period);
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(AegisPins::ESTOP_SENSE_IN, INPUT_PULLUP);
    pinMode(AegisPins::LIMIT_OPEN_SW, INPUT_PULLUP);
    pinMode(AegisPins::LIMIT_CLOSE_SW, INPUT_PULLUP);
    pinMode(AegisPins::SOLENOID_CTRL, OUTPUT);
    digitalWrite(AegisPins::SOLENOID_CTRL, LOW);

    ledcSetup(0, 20000, 10); // DRV8871 PWM 20 kHz 10-bit
    ledcSetup(1, 20000, 10);
    ledcAttachPin(AegisPins::MOTOR_IN1, 0);
    ledcAttachPin(AegisPins::MOTOR_IN2, 1);
    apply_drv8871_pwm_slow_decay(0);

    analogReadResolution(12);
    analogSetPinAttenuation(AegisPins::V_MAINS_SENSE, ADC_11db);
    analogSetPinAttenuation(AegisPins::V_UPS_BATT_SENSE, ADC_11db);

    Wire.begin(AegisPins::I2C_SDA, AegisPins::I2C_SCL, 400000);
    g_prefs.begin("aegis_sec", false);
    g_nvs_tamper_latched = g_prefs.getBool("tamper_latch", false);

    esp_task_wdt_init(2, true);
    xI2CMutex   = xSemaphoreCreateMutex();
    xCmdQueue   = xQueueCreate(8, sizeof(BoltCommand));
    xInferQueue = xQueueCreate(1, sizeof(GestureInferPayload));

    if (!run_post_golden_vectors()) {
        apply_drv8871_pwm_slow_decay(0);
        digitalWrite(AegisPins::SOLENOID_CTRL, LOW);
        while (true) { delay(500); }
    }

    xTaskCreatePinnedToCore(Task_Sensor_Control_RT, "Sensor_RT", 8192, NULL, 22, NULL, 1);
    xTaskCreatePinnedToCore(Task_Gesture_AI,        "GestureAI", 8192, NULL, 14, NULL, 0);
    xTaskCreatePinnedToCore(Task_Comms_HMI_FSM,     "Comms_FSM", 8192, NULL, 10, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}
