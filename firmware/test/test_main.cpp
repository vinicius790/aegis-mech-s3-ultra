#include <unity.h>
#include "../src/board_pins.h"
#include "../src/protocol_frames.h"
#include "../src/mechatronics/lock_controller.h"
#include "../src/mechatronics/security_fsm.h"
#include "../src/ai/gesture_featurizer.h"
#include "../src/ai/kalman_particle.h"
#include "../src/ai/inference_int8.h"
#include "../src/ai/test_vectors.h"
#include "../src/sensors/sensor_security_power.h"

void test_pin_contracts() {
    TEST_ASSERT_EQUAL(4, AegisPins::US_X_TRIG);
    TEST_ASSERT_EQUAL(5, AegisPins::US_X_ECHO);
    TEST_ASSERT_EQUAL(42, AegisPins::ESTOP_SENSE_IN);
    TEST_ASSERT_NOT_EQUAL(AegisPins::MOTOR_IN1, AegisPins::MOTOR_IN2);
}

void test_round_shift_symmetric() {
    TEST_ASSERT_EQUAL(2, aegis::ai::round_shift_clamp_i8(128, 6, false));
    TEST_ASSERT_EQUAL(0, aegis::ai::round_shift_clamp_i8(-10, 6, true));
    TEST_ASSERT_EQUAL(-127, aegis::ai::round_shift_clamp_i8(-50000, 4, false));
}

void test_golden_vectors_post() {
    for (int i = 0; i < AEGIS_NUM_TEST_VECTORS; ++i) {
        int32_t logits[7] = {0};
        float bio[16] = {0.0f};
        aegis::ai::tiny1dcnn_infer_int8(aegis::test::TEST_INPUTS[i], logits, bio);
        for (int c = 0; c < 7; ++c) {
            TEST_ASSERT_EQUAL_INT32(aegis::test::EXPECTED_LOGITS[i][c], logits[c]);
        }
    }
}

void test_featurizer_computes_nonzero_velocities() {
    aegis::ai::GestureFeaturizer feat;
    for (int i = 0; i < 26; ++i) {
        feat.push_point(static_cast<uint32_t>(i * 20), 150.0f + i * 4.0f, 160.0f + i * 3.0f);
    }
    TEST_ASSERT_TRUE(feat.is_gesture_complete());
    int8_t vec[120] = {0};
    feat.extract_features_120(vec);
    TEST_ASSERT_NOT_EQUAL(0, vec[2]); // vx nao e zero!
    TEST_ASSERT_NOT_EQUAL(0, vec[3]); // vy nao e zero!
}

void test_particle_filter_resamples_without_collapse() {
    aegis::ai::ParticleFilterLite32 pf;
    pf.reset(190.0f);
    for (int i = 0; i < 12; ++i) {
        pf.predict(0.0f, 0.02f);
        pf.update(215.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 215.0f, pf.estimate());
}

void test_lock_controller_measurement_d() {
    aegis::control::LockController ctrl;
    ctrl.set_target(28.0f);
    const int16_t u = ctrl.compute(0.0f, 10000ULL, 0.01f, 100.0f);
    TEST_ASSERT_TRUE(u > 0);
}

void test_security_fsm_latch_and_estop() {
    aegis::fsm::SecurityFSM fsm(aegis::fsm::DEFAULT_STATE_TABLE);
    aegis::fsm::SystemTelemetry obs{};
    obs.now_ms = 1000;
    obs.tamper_score = 0.95f;
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(aegis::fsm::VaultState::TAMPER_LOCKDOWN),
                      static_cast<uint8_t>(fsm.step(obs)));
    obs.now_ms = 40000; // Apos 39s SEM auth: deve continuar travado!
    obs.tamper_score = 0.1f;
    obs.nvs_tamper_latched = true;
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(aegis::fsm::VaultState::TAMPER_LOCKDOWN),
                      static_cast<uint8_t>(fsm.step(obs)));

    obs.estop_active = true;
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(aegis::fsm::VaultState::HARDWARE_ESTOP),
                      static_cast<uint8_t>(fsm.step(obs)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pin_contracts);
    RUN_TEST(test_round_shift_symmetric);
    RUN_TEST(test_golden_vectors_post);
    RUN_TEST(test_featurizer_computes_nonzero_velocities);
    RUN_TEST(test_particle_filter_resamples_without_collapse);
    RUN_TEST(test_lock_controller_measurement_d);
    RUN_TEST(test_security_fsm_latch_and_estop);
    return UNITY_END();
}
