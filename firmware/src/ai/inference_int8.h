#pragma once
#include <stdint.h>
#include <stddef.h>

namespace aegis::ai {

inline int8_t round_shift_clamp_i8(int32_t acc, uint8_t shift_s, bool apply_relu) {
    int32_t rounded = 0;
    if (shift_s == 0) {
        rounded = acc;
    } else {
        const int32_t half = 1 << (shift_s - 1);
        rounded = (acc >= 0) ? ((acc + half) >> shift_s)
                             : (-(((-acc) + half) >> shift_s));
    }
    if (apply_relu && rounded < 0) return 0;
    if (rounded > 127)  return 127;
    if (rounded < -127) return -127; // Clamp simetrico [-127, 127]
    return static_cast<int8_t>(rounded);
}

inline int32_t dot_i8_acc32(const int8_t* __restrict weights,
                            const int8_t* __restrict input,
                            size_t len,
                            int32_t bias_q32) {
    int32_t acc = bias_q32;
    for (size_t i = 0; i < len; ++i) {
        acc += static_cast<int32_t>(weights[i]) * static_cast<int32_t>(input[i]);
    }
    return acc;
}

void conv1d_i8(const int8_t* in, int c_in, int l_in,
               const int8_t* w, const int32_t* bias,
               int c_out, int k_size, int stride, int pad,
               uint8_t shift_s, bool relu, int8_t* out);

void depthwise_conv1d_k3_s2_i8(const int8_t* in, int channels, int l_in,
                               const int8_t* w, const int32_t* bias,
                               uint8_t shift_s, bool relu, int8_t* out);

void gap_len12_to_i8(const int8_t* in, int channels, int8_t* out);

void tiny1dcnn_infer_int8(const int8_t in_120[120], int32_t logits_out[7], float bio_out[16]);

float ood_mse_f32(const int8_t in_120[120]);

uint8_t tamper_infer_f32(const float in_12[12], float out_probs[5]);

} // namespace aegis::ai
