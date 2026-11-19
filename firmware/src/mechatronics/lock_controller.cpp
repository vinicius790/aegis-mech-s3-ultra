#include "lock_controller.h"
#include "../ai/actuator_model.h"

namespace aegis::control {

void SCurveProfile::reset(float current_pos_mm) {
    pos_start = current_pos_mm;
    pos_target = current_pos_mm;
    ref_pos = current_pos_mm;
    ref_vel = 0.0f;
    t_elapsed = 0.0f;
}

float SCurveProfile::step_reference(float target_mm, float dt_s) {
    if (fabsf(target_mm - pos_target) > 0.05f) {
        pos_target = target_mm;
        pos_start = ref_pos;
        t_elapsed = 0.0f;
    }
    t_elapsed += dt_s;
    const float dist = pos_target - pos_start;
    const float t_move = fmaxf(0.35f, fabsf(dist) / fmaxf(v_max * 0.65f, 1.0f)); // >= 350 ms anti-inrush
    const float s = fminf(1.0f, fmaxf(0.0f, t_elapsed / t_move));
    // Polinomio quintico minimo-jerk C^2: 10s^3 - 15s^4 + 6s^5
    const float s2 = s * s;
    const float s3 = s2 * s;
    const float poly = s3 * (10.0f + s * (-15.0f + 6.0f * s));
    ref_pos = pos_start + dist * poly;
    ref_vel = dist * (30.0f * s2 - 60.0f * s3 + 30.0f * s2 * s2) / t_move;
    return ref_pos;
}

void LockController::set_target(float target_mm) {
    pos_target_mm_ = target_mm;
}

void LockController::reset() {
    scurve_.reset(pos_current_mm_);
    integral_error_ = 0.0f;
    stall_timer_s_ = 0.0f;
    last_feedback_us_ = 0;
    fault_code_ = FaultCode::E00_NOMINAL;
    is_stalled_ = false;
}

float LockController::evaluate_actuatornet_ff(float error, float vel, float v_bus) {
    float feats[10] = {};
    feats[0] = fmaxf(-1.0f, fminf(1.0f, error / 28.0f));
    feats[1] = vel / 100.0f;
    feats[2] = (v_bus - 12.0f) / 6.0f;
    feats[3] = 0.0f;
    return aegis::actuator::evaluate_mlp(feats);
}

float LockController::fuzzy_infer_kp_kd(float error, float abs_vel, float& out_kd) {
    const float abs_e = fabsf(error);
    float kp = 45.0f;
    if (abs_e > 10.0f) {
        kp = 65.0f;
    } else if (abs_e < 1.0f) {
        kp = 30.0f;
    }

    out_kd = 1.2f;
    if (abs_vel > 20.0f) {
        out_kd = 2.4f;
    } else if (abs_vel < 2.0f) {
        out_kd = 0.8f;
    }
    return kp;
}

int16_t LockController::compute(float pos_meas_mm, uint64_t now_us, float dt_s, float current_ma) {
    // Invariante MAX_AGE = 150 ms
    if (last_feedback_us_ > 0 && (now_us - last_feedback_us_) > 150000ULL) {
        fault_code_ = FaultCode::E04_FEEDBACK_MAX_AGE_150MS;
        return 0;
    }
    last_feedback_us_ = now_us;

    const float pos_prev = pos_current_mm_;
    if (dt_s > 1e-4f) {
        vel_current_mm_s_ = (pos_meas_mm - pos_prev) / dt_s;
    }
    pos_current_mm_ = pos_meas_mm;

    // Blanking de partida 150 ms (evita falso stall em inrush) + deteccao sustentada > 120 ms
    if (scurve_.elapsed_s() > 0.15f && current_ma > 1800.0f && fabsf(vel_current_mm_s_) < 1.0f) {
        stall_timer_s_ += dt_s;
        if (stall_timer_s_ >= 0.12f) {
            is_stalled_ = true;
            fault_code_ = FaultCode::E03_MOTOR_STALL_OVERCURRENT;
            return 0;
        }
    } else {
        stall_timer_s_ = 0.0f;
    }

    // Zona morta final no alvo estatico
    if (fabsf(pos_target_mm_ - pos_current_mm_) < 0.3f && fabsf(vel_current_mm_s_) < 1.5f) {
        return 0;
    }

    const float ref_mm = scurve_.step_reference(pos_target_mm_, dt_s);
    const float error  = ref_mm - pos_current_mm_;

    // Derivada ESTRITAMENTE na medicao (elimina derivative kick em degraus)
    const float d_meas = (dt_s > 1e-4f) ? -(pos_current_mm_ - pos_prev) / dt_s : 0.0f;

    float kd = 1.2f;
    const float kp = fuzzy_infer_kp_kd(error, fabsf(vel_current_mm_s_), kd);
    const float ki = 2.0f;
    const float u_ff = evaluate_actuatornet_ff(error, vel_current_mm_s_, v_bus_v_);

    const float u_unsat = kp * error + kd * d_meas + ki * integral_error_ + u_ff;
    const float u_sat = fmaxf(-1000.0f, fminf(1000.0f, u_unsat));

    // Anti-windup por back-calculation (Kb = 0.05) + clamp
    integral_error_ += (error * dt_s) + 0.05f * (u_sat - u_unsat) * dt_s;
    integral_error_ = fmaxf(-5.0f, fminf(5.0f, integral_error_));

    return static_cast<int16_t>(u_sat);
}

} // namespace aegis::control
