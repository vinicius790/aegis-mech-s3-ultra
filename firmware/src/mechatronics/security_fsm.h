#pragma once
#include <stdint.h>
#include <array>

namespace aegis::fsm {

enum class VaultState : uint8_t {
    LOCKED_IDLE = 0,
    SENSING_GESTURE,
    INFERENCE_BIO,
    AUTH_VISION_2FA,
    UNLOCKING_BOLT,
    OPEN_WAIT,
    LOCKING_BOLT,
    TAMPER_LOCKDOWN,
    AUTH_PAUSED_RECOVER,
    HARDWARE_ESTOP,
    COUNT
};

struct SystemTelemetry {
    uint32_t now_ms;
    uint32_t encoder_age_us;
    float    tof_x_mm;
    float    tof_y_mm;
    float    bolt_pos_mm;        // Posicao metrica real do ferrolho (deg * 0.17453 mm/deg)
    float    current_ma;
    float    tamper_score;
    uint16_t arena_dwell_ms;     // Permanencia continua na arena >= 120 ms (anti-ruido)
    bool     estop_active;       // GPIO 42 HIGH = corte fisico de potencia
    bool     nvs_tamper_latched; // Flag persistente NVS pós-ataque
    bool     tamper_clear_auth_ok;
    bool     gesture_seq_ok;
    bool     bio_ood_ok;
    bool     vision_2fa_ok;
    bool     limit_open_hit;
    bool     limit_closed_hit;
};

using StateHandlerFn = VaultState (*)(const SystemTelemetry& obs, uint32_t elapsed_ms);

class SecurityFSM {
public:
    explicit SecurityFSM(const std::array<StateHandlerFn, static_cast<size_t>(VaultState::COUNT)>& table)
        : dispatch_(table), current_(VaultState::LOCKED_IDLE), resume_state_(VaultState::LOCKED_IDLE), entered_ms_(0) {}

    VaultState step(const SystemTelemetry& obs) {
        // 1. Guarda global prioritaria: E-Stop fisico (GPIO 42 HIGH)
        if (obs.estop_active) {
            if (current_ != VaultState::HARDWARE_ESTOP) {
                return transition_to(VaultState::HARDWARE_ESTOP, obs.now_ms);
            }
            return current_;
        }
        if (current_ == VaultState::HARDWARE_ESTOP && !obs.estop_active) {
            return transition_to(VaultState::LOCKED_IDLE, obs.now_ms);
        }

        // 2. Guarda Tamper: viola ou NVS travado
        if ((obs.nvs_tamper_latched || obs.tamper_score >= 0.85f) &&
            current_ != VaultState::OPEN_WAIT &&
            current_ != VaultState::TAMPER_LOCKDOWN) {
            return transition_to(VaultState::TAMPER_LOCKDOWN, obs.now_ms);
        }

        // 3. Pilha simples suspend/resume durante perda momentanea de sensor gestual
        if (current_ == VaultState::SENSING_GESTURE && (obs.tof_x_mm <= 0.0f || obs.tof_y_mm <= 0.0f)) {
            resume_state_ = VaultState::SENSING_GESTURE;
            return transition_to(VaultState::AUTH_PAUSED_RECOVER, obs.now_ms);
        }
        if (current_ == VaultState::AUTH_PAUSED_RECOVER) {
            if (obs.tof_x_mm >= 120.0f && obs.tof_x_mm <= 320.0f) {
                return transition_to(resume_state_, obs.now_ms);
            }
            if ((obs.now_ms - entered_ms_) > 250) {
                return transition_to(VaultState::LOCKED_IDLE, obs.now_ms);
            }
            return current_;
        }

        const uint8_t idx = static_cast<uint8_t>(current_);
        const VaultState next = dispatch_[idx](obs, obs.now_ms - entered_ms_);
        if (next != current_) {
            return transition_to(next, obs.now_ms);
        }
        return current_;
    }

    VaultState state() const { return current_; }

private:
    VaultState transition_to(VaultState next, uint32_t now_ms) {
        current_ = next;
        entered_ms_ = now_ms;
        return current_;
    }

    std::array<StateHandlerFn, static_cast<size_t>(VaultState::COUNT)> dispatch_;
    VaultState current_;
    VaultState resume_state_;
    uint32_t entered_ms_;
};

extern const std::array<StateHandlerFn, static_cast<size_t>(VaultState::COUNT)> DEFAULT_STATE_TABLE;

} // namespace aegis::fsm
