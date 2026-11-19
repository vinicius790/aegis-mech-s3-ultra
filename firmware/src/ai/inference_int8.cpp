#include "inference_int8.h"
#include "model_cnn.h"
#include "ood_model.h"
#include "tamper_model.h"
#include <math.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(AEGIS_NATIVE_TEST) && defined(AEGIS_USE_PIE_ASM)
extern "C" int32_t aegis_pie_dot_s8(const int8_t* w, const int8_t* x, int len);
#endif

namespace aegis::ai {

void conv1d_i8(const int8_t* in, int c_in, int l_in,
               const int8_t* w, const int32_t* bias,
               int c_out, int k_size, int stride, int pad,
               uint8_t shift_s, bool relu, int8_t* out) {
    const int l_out = (l_in + 2 * pad - k_size) / stride + 1;
    for (int oc = 0; oc < c_out; ++oc) {
        for (int t = 0; t < l_out; ++t) {
            int32_t acc = bias[oc];
            for (int ic = 0; ic < c_in; ++ic) {
                for (int k = 0; k < k_size; ++k) {
                    const int ti = t * stride - pad + k;
                    if (ti >= 0 && ti < l_in) {
                        const int w_idx = (oc * c_in + ic) * k_size + k;
                        acc += static_cast<int32_t>(w[w_idx]) * static_cast<int32_t>(in[ic * l_in + ti]);
                    }
                }
            }
            out[oc * l_out + t] = round_shift_clamp_i8(acc, shift_s, relu);
        }
    }
}

void depthwise_conv1d_k3_s2_i8(const int8_t* in, int channels, int l_in,
                               const int8_t* w, const int32_t* bias,
                               uint8_t shift_s, bool relu, int8_t* out) {
    const int l_out = (l_in + 2 - 3) / 2 + 1; // 12 passos p/ l_in=24
    for (int c = 0; c < channels; ++c) {
        for (int t = 0; t < l_out; ++t) {
            int32_t acc = bias[c];
            for (int k = 0; k < 3; ++k) {
                const int ti = t * 2 - 1 + k;
                if (ti >= 0 && ti < l_in) {
                    acc += static_cast<int32_t>(w[c * 3 + k]) * static_cast<int32_t>(in[c * l_in + ti]);
                }
            }
            out[c * l_out + t] = round_shift_clamp_i8(acc, shift_s, relu);
        }
    }
}

void gap_len12_to_i8(const int8_t* in, int channels, int8_t* out) {
    for (int c = 0; c < channels; ++c) {
        int32_t acc = 0;
        for (int t = 0; t < 12; ++t) acc += static_cast<int32_t>(in[c * 12 + t]);
        const int32_t q = (acc >= 0) ? ((acc + 6) / 12) : -(((-acc) + 6) / 12);
        out[c] = static_cast<int8_t>(q > 127 ? 127 : (q < -127 ? -127 : q));
    }
}

void tiny1dcnn_infer_int8(const int8_t in_120[120], int32_t logits_out[7], float bio_out[16]) {
    // Transpoe (24, 5) para layout channel-first (5, 24) identico ao PyTorch/Python
    int8_t x_seq[5 * 24];
    for (int t = 0; t < 24; ++t) {
        for (int c = 0; c < 5; ++c) {
            x_seq[c * 24 + t] = in_120[t * 5 + c];
        }
    }

    int8_t h1[16 * 24];
    int8_t hdw[16 * 12];
    int8_t hpw[24 * 12];
    int8_t h3[32 * 12];
    int8_t hgap[32];
    int8_t hfc[32];

    conv1d_i8(x_seq, 5, 24, aegis::cnn::CONV1_W, aegis::cnn::CONV1_B, 16, 3, 1, 1, AEGIS_S1_SHIFT, true, h1);
    depthwise_conv1d_k3_s2_i8(h1, 16, 24, aegis::cnn::DW_W, aegis::cnn::DW_B, AEGIS_SDW_SHIFT, false, hdw);
    conv1d_i8(hdw, 16, 12, aegis::cnn::PW_W, aegis::cnn::PW_B, 24, 1, 1, 0, AEGIS_SPW_SHIFT, true, hpw);
    conv1d_i8(hpw, 24, 12, aegis::cnn::CONV3_W, aegis::cnn::CONV3_B, 32, 3, 1, 1, AEGIS_S3_SHIFT, true, h3);
    gap_len12_to_i8(h3, 32, hgap);

    for (int i = 0; i < 32; ++i) {
        const int32_t acc = dot_i8_acc32(&aegis::cnn::FC_W[i * 32], hgap, 32, aegis::cnn::FC_B[i]);
        hfc[i] = round_shift_clamp_i8(acc, AEGIS_SFC_SHIFT, true);
    }

    for (int c = 0; c < 7; ++c) {
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(AEGIS_NATIVE_TEST) && defined(AEGIS_USE_PIE_ASM)
        logits_out[c] = aegis::cnn::CLS_B[c] + aegis_pie_dot_s8(&aegis::cnn::CLS_W[c * 32], hfc, 32);
#else
        logits_out[c] = dot_i8_acc32(&aegis::cnn::CLS_W[c * 32], hfc, 32, aegis::cnn::CLS_B[c]);
#endif
    }

    float norm_sq = 0.0f;
    for (int b = 0; b < 16; ++b) {
        const int32_t b_acc = dot_i8_acc32(&aegis::cnn::BIO_W[b * 32], hfc, 32, aegis::cnn::BIO_B[b]);
        bio_out[b] = static_cast<float>(b_acc);
        norm_sq += bio_out[b] * bio_out[b];
    }
    const float inv_norm = (norm_sq > 1e-6f) ? (1.0f / sqrtf(norm_sq)) : 0.0f;
    for (int b = 0; b < 16; ++b) bio_out[b] *= inv_norm;
}

float ood_mse_f32(const int8_t in_120[120]) {
    float x[120];
    for (int i = 0; i < 120; ++i) x[i] = static_cast<float>(in_120[i]) / 64.0f;

    float h1[48], z[16], d1[48];
    for (int i = 0; i < 48; ++i) {
        float acc = aegis::ood::E1_B[i];
        for (int j = 0; j < 120; ++j) acc += aegis::ood::E1_W[i * 120 + j] * x[j];
        h1[i] = (acc > 0.0f) ? acc : 0.0f;
    }
    for (int i = 0; i < 16; ++i) {
        float acc = aegis::ood::E2_B[i];
        for (int j = 0; j < 48; ++j) acc += aegis::ood::E2_W[i * 48 + j] * h1[j];
        z[i] = (acc > 0.0f) ? acc : 0.0f;
    }
    for (int i = 0; i < 48; ++i) {
        float acc = aegis::ood::D1_B[i];
        for (int j = 0; j < 16; ++j) acc += aegis::ood::D1_W[i * 16 + j] * z[j];
        d1[i] = (acc > 0.0f) ? acc : 0.0f;
    }
    float mse = 0.0f;
    for (int i = 0; i < 120; ++i) {
        float acc = aegis::ood::D2_B[i];
        for (int j = 0; j < 48; ++j) acc += aegis::ood::D2_W[i * 48 + j] * d1[j];
        const float diff = acc - x[i];
        mse += diff * diff;
    }
    return mse / 120.0f;
}

uint8_t tamper_infer_f32(const float in_12[12], float out_probs[5]) {
    auto relu6 = [](float v) { return v < 0.0f ? 0.0f : (v > 6.0f ? 6.0f : v); };
    float h1[24], h2[16], logits[5];
    for (int i = 0; i < 24; ++i) {
        float acc = aegis::tamper::B1[i];
        for (int j = 0; j < 12; ++j) acc += aegis::tamper::W1[i * 12 + j] * in_12[j];
        h1[i] = relu6(acc);
    }
    for (int i = 0; i < 16; ++i) {
        float acc = aegis::tamper::B2[i];
        for (int j = 0; j < 24; ++j) acc += aegis::tamper::W2[i * 24 + j] * h1[j];
        h2[i] = relu6(acc);
    }
    float max_l = -1e9f;
    for (int i = 0; i < 5; ++i) {
        float acc = aegis::tamper::B3[i];
        for (int j = 0; j < 16; ++j) acc += aegis::tamper::W3[i * 16 + j] * h2[j];
        logits[i] = acc;
        if (acc > max_l) max_l = acc;
    }
    float sum_e = 0.0f;
    uint8_t best = 0;
    for (int i = 0; i < 5; ++i) {
        out_probs[i] = expf(logits[i] - max_l);
        sum_e += out_probs[i];
        if (logits[i] > logits[best]) best = static_cast<uint8_t>(i);
    }
    for (int i = 0; i < 5; ++i) out_probs[i] /= fmaxf(sum_e, 1e-6f);
    return best;
}

} // namespace aegis::ai
