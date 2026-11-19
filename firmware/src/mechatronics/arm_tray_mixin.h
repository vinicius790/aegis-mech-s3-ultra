#pragma once
#include <stdint.h>
#include <math.h>

namespace aegis::mechatronics {

// Controlador nao-bloqueante p/ gaveta 2-DOF (substitui delay()/time.sleep())
class ArmTrayMixin {
public:
    ArmTrayMixin()
        : tray_deployed_(false), current_deg_(0.0f), target_deg_(0.0f), last_step_ms_(0) {}

    bool is_deployed() const { return tray_deployed_; }
    float current_angle_deg() const { return current_deg_; }

    void deploy_async(uint32_t now_ms) {
        tray_deployed_ = true;
        target_deg_ = 90.0f;
        last_step_ms_ = now_ms;
    }

    void retract_async(uint32_t now_ms) {
        tray_deployed_ = false;
        target_deg_ = 0.0f;
        last_step_ms_ = now_ms;
    }

    float step(uint32_t now_ms, float max_rate_deg_s = 120.0f);

private:
    bool     tray_deployed_;
    float    current_deg_;
    float    target_deg_;
    uint32_t last_step_ms_;
};

} // namespace aegis::mechatronics
