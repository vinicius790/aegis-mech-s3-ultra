#pragma once
#include <stdint.h>
#include <math.h>
#include "../protocol_frames.h"

namespace aegis::control {

struct SCurveProfile {
    float pos_start = 0.0f;
    float pos_target = 0.0f;
    float ref_pos = 0.0f;
    float ref_vel = 0.0f;
    float v_max = 35.0f;
    float t_elapsed = 0.0f;

    void reset(float current_pos_mm);
    float step_reference(float target_mm, float dt_s);
    float elapsed_s() const { return t_elapsed; }
};

class LockController {
public:
    LockController()
        : pos_current_mm_(0.0f), pos_target_mm_(0.0f),
          vel_current_mm_s_(0.0f), integral_error_(0.0f),
          stall_timer_s_(0.0f), v_bus_v_(12.0f),
          last_feedback_us_(0), fault_code_(FaultCode::E00_NOMINAL),
          is_stalled_(false) {}

    void set_target(float target_mm);
    void set_bus_voltage(float v_bus_v) { v_bus_v_ = v_bus_v; }
    int16_t compute(float pos_meas_mm, uint64_t now_us, float dt_s, float current_ma = 0.0f);
    void reset();

    float current_position() const { return pos_current_mm_; }
    float target_position() const { return pos_target_mm_; }
    float reference_position() const { return scurve_.ref_pos; }
    bool is_stalled() const { return is_stalled_; }
    FaultCode fault_code() const { return fault_code_; }

private:
    float fuzzy_infer_kp_kd(float error, float abs_vel, float& out_kd);
    float evaluate_actuatornet_ff(float error, float vel, float v_bus);

    SCurveProfile scurve_;
    float pos_current_mm_;
    float pos_target_mm_;
    float vel_current_mm_s_;
    float integral_error_;
    float stall_timer_s_;
    float v_bus_v_;
    uint64_t last_feedback_us_;
    FaultCode fault_code_;
    bool is_stalled_;
};

} // namespace aegis::control
