#include "security_fsm.h"

namespace aegis::fsm {

static VaultState handle_locked_idle(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)elapsed_ms;
    // Exige mao na arena [120..320] mm com permanencia >= 120 ms (anti-disparo falso)
    if (obs.tof_x_mm >= 120.0f && obs.tof_x_mm <= 320.0f && obs.arena_dwell_ms >= 120) {
        return VaultState::SENSING_GESTURE;
    }
    return VaultState::LOCKED_IDLE;
}

static VaultState handle_sensing_gesture(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    if (obs.gesture_seq_ok) {
        return VaultState::INFERENCE_BIO;
    }
    if (elapsed_ms > 4000) {
        return VaultState::LOCKED_IDLE;
    }
    return VaultState::SENSING_GESTURE;
}

static VaultState handle_inference_bio(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)elapsed_ms;
    return obs.bio_ood_ok ? VaultState::AUTH_VISION_2FA : VaultState::LOCKED_IDLE;
}

static VaultState handle_auth_vision_2fa(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    if (obs.vision_2fa_ok) {
        return VaultState::UNLOCKING_BOLT;
    }
    if (elapsed_ms > 5000) {
        return VaultState::LOCKED_IDLE;
    }
    return VaultState::AUTH_VISION_2FA;
}

static VaultState handle_unlocking_bolt(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    // Unidade metrica correta: bolt_pos_mm >= 28.0 mm (nunca graus brutos do AS5600!)
    if (obs.limit_open_hit || obs.bolt_pos_mm >= 28.0f) {
        return VaultState::OPEN_WAIT;
    }
    if (elapsed_ms > 2500) {
        return VaultState::LOCKED_IDLE;
    }
    return VaultState::UNLOCKING_BOLT;
}

static VaultState handle_open_wait(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)obs;
    if (elapsed_ms > 10000) {
        return VaultState::LOCKING_BOLT;
    }
    return VaultState::OPEN_WAIT;
}

static VaultState handle_locking_bolt(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    if (obs.limit_closed_hit || obs.bolt_pos_mm <= 0.5f) {
        return VaultState::LOCKED_IDLE;
    }
    if (elapsed_ms > 2500) {
        return VaultState::LOCKED_IDLE;
    }
    return VaultState::LOCKING_BOLT;
}

static VaultState handle_tamper_lockdown(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)elapsed_ms;
    // Bug G09 corrigido: NUNCA destranca sozinho após 30s; exige autenticacao explicita!
    if (obs.tamper_clear_auth_ok && !obs.nvs_tamper_latched) {
        return VaultState::LOCKED_IDLE;
    }
    return VaultState::TAMPER_LOCKDOWN;
}

static VaultState handle_paused_recover(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)obs;
    (void)elapsed_ms;
    return VaultState::AUTH_PAUSED_RECOVER;
}

static VaultState handle_hardware_estop(const SystemTelemetry& obs, uint32_t elapsed_ms) {
    (void)obs;
    (void)elapsed_ms;
    return VaultState::HARDWARE_ESTOP;
}

const std::array<StateHandlerFn, static_cast<size_t>(VaultState::COUNT)> DEFAULT_STATE_TABLE = {
    handle_locked_idle,
    handle_sensing_gesture,
    handle_inference_bio,
    handle_auth_vision_2fa,
    handle_unlocking_bolt,
    handle_open_wait,
    handle_locking_bolt,
    handle_tamper_lockdown,
    handle_paused_recover,
    handle_hardware_estop
};

} // namespace aegis::fsm
