#pragma once
#include <stdint.h>
#include <stddef.h>

namespace aegis::ai {

// Featurizer embutido que recebe pontos em streaming (50 Hz),
// detecta inicio e fim de gesto e produz vetor canonico de 120 bytes em int8 Q1.6
class GestureFeaturizer {
public:
    static constexpr size_t MAX_RAW_POINTS = 64;
    static constexpr size_t TARGET_POINTS = 24;
    static constexpr size_t NUM_CHANNELS = 5; // x, y, vx, vy, t_rel
    static constexpr size_t FEATURE_DIM = TARGET_POINTS * NUM_CHANNELS; // 120

    GestureFeaturizer() : count_(0), in_gesture_(false) {}

    void reset();
    void push_point(uint32_t t_ms, float x_mm, float y_mm);
    bool is_gesture_complete() const;
    void extract_features_120(int8_t out_vec[120]);

private:
    uint32_t t_buf_[MAX_RAW_POINTS];
    float x_buf_[MAX_RAW_POINTS];
    float y_buf_[MAX_RAW_POINTS];
    size_t count_;
    bool in_gesture_;
};

} // namespace aegis::ai
