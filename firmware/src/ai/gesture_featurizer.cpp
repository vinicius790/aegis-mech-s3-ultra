#include "gesture_featurizer.h"
#include <math.h>

namespace aegis::ai {

void GestureFeaturizer::reset() {
    count_ = 0;
    in_gesture_ = false;
}

void GestureFeaturizer::push_point(uint32_t t_ms, float x_mm, float y_mm) {
    if (isnan(x_mm) || isnan(y_mm)) return;
    if (count_ < MAX_RAW_POINTS) {
        t_buf_[count_] = t_ms;
        x_buf_[count_] = x_mm;
        y_buf_[count_] = y_mm;
        count_++;
        if (count_ >= 10) in_gesture_ = true;
    }
}

bool GestureFeaturizer::is_gesture_complete() const {
    return (in_gesture_ && count_ >= 25);
}

void GestureFeaturizer::extract_features_120(int8_t out_vec[120]) {
    if (count_ < 2) {
        for (size_t i = 0; i < FEATURE_DIM; ++i) out_vec[i] = 0;
        return;
    }

    float x_interp[TARGET_POINTS];
    float y_interp[TARGET_POINTS];
    const uint32_t t_start = t_buf_[0];
    const uint32_t t_end   = t_buf_[count_ - 1];
    const float span       = (t_end > t_start) ? static_cast<float>(t_end - t_start) : 1.0f;

    // 1. Reamostragem uniforme identica a numpy.interp
    for (size_t i = 0; i < TARGET_POINTS; ++i) {
        const float alpha = static_cast<float>(i) / static_cast<float>(TARGET_POINTS - 1);
        const float t_target = static_cast<float>(t_start) + alpha * span;
        float x = x_buf_[count_ - 1];
        float y = y_buf_[count_ - 1];
        for (size_t j = 0; j < count_ - 1; ++j) {
            if (t_target >= static_cast<float>(t_buf_[j]) && t_target <= static_cast<float>(t_buf_[j + 1])) {
                const float ds = static_cast<float>(t_buf_[j + 1] - t_buf_[j]);
                const float f  = (ds > 0.0f) ? (t_target - static_cast<float>(t_buf_[j])) / ds : 0.0f;
                x = x_buf_[j] + f * (x_buf_[j + 1] - x_buf_[j]);
                y = y_buf_[j] + f * (y_buf_[j + 1] - y_buf_[j]);
                break;
            }
        }
        x_interp[i] = x;
        y_interp[i] = y;
    }

    // 2. Derivadas finitas exatas (identicas a numpy.gradient com dt = 2 / 23) + clamp simetrico [-127, 127]
    const float dt = 2.0f / static_cast<float>(TARGET_POINTS - 1);
    auto to_q16 = [](float v) -> int8_t {
        const int q = static_cast<int>(roundf(v * 64.0f));
        return static_cast<int8_t>(q > 127 ? 127 : (q < -127 ? -127 : q));
    };
    auto clip1 = [](float v) -> float {
        return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
    };

    for (size_t i = 0; i < TARGET_POINTS; ++i) {
        const float vx = (i == 0) ? (x_interp[1] - x_interp[0]) / dt
                       : (i == TARGET_POINTS - 1) ? (x_interp[i] - x_interp[i - 1]) / dt
                       : (x_interp[i + 1] - x_interp[i - 1]) / (2.0f * dt);
        const float vy = (i == 0) ? (y_interp[1] - y_interp[0]) / dt
                       : (i == TARGET_POINTS - 1) ? (y_interp[i] - y_interp[i - 1]) / dt
                       : (y_interp[i + 1] - y_interp[i - 1]) / (2.0f * dt);

        const float x_norm = 2.0f * (x_interp[i] - 120.0f) / 200.0f - 1.0f;
        const float y_norm = 2.0f * (y_interp[i] - 120.0f) / 200.0f - 1.0f;
        const float t_rel  = 2.0f * static_cast<float>(i) / static_cast<float>(TARGET_POINTS - 1) - 1.0f;

        out_vec[i * 5 + 0] = to_q16(x_norm);
        out_vec[i * 5 + 1] = to_q16(y_norm);
        out_vec[i * 5 + 2] = to_q16(clip1(vx / 500.0f));
        out_vec[i * 5 + 3] = to_q16(clip1(vy / 500.0f));
        out_vec[i * 5 + 4] = to_q16(t_rel);
    }
}

} // namespace aegis::ai
