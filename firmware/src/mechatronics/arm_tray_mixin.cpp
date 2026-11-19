#include "arm_tray_mixin.h"

namespace aegis::mechatronics {

float ArmTrayMixin::step(uint32_t now_ms, float max_rate_deg_s) {
    if (now_ms <= last_step_ms_) return current_deg_;
    const float dt_s = static_cast<float>(now_ms - last_step_ms_) / 1000.0f;
    last_step_ms_ = now_ms;
    const float max_delta = max_rate_deg_s * fminf(dt_s, 0.1f);
    const float err = target_deg_ - current_deg_;
    if (fabsf(err) <= max_delta) {
        current_deg_ = target_deg_;
    } else {
        current_deg_ += (err > 0.0f ? max_delta : -max_delta);
    }
    return current_deg_;
}

} // namespace aegis::mechatronics
