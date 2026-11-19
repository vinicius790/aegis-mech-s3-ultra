#include "kalman_particle.h"

namespace aegis::ai {

void KalmanFilter2D::reset(float init_x, float init_y) {
    x_[0] = init_x; x_[1] = 0.0f;
    y_[0] = init_y; y_[1] = 0.0f;
    P_x_[0][0] = 10.0f; P_x_[0][1] = 0.0f; P_x_[1][0] = 0.0f; P_x_[1][1] = 10.0f;
    P_y_[0][0] = 10.0f; P_y_[0][1] = 0.0f; P_y_[1][0] = 0.0f; P_y_[1][1] = 10.0f;
}

void KalmanFilter2D::predict(float dt_s) {
    if (dt_s <= 0.0f) dt_s = 0.02f;
    x_[0] += x_[1] * dt_s;
    y_[0] += y_[1] * dt_s;

    const float dt2 = dt_s * dt_s;
    const float dt3 = dt2 * dt_s;
    const float dt4 = dt2 * dt2;
    const float sa2 = sigma_a_ * sigma_a_;
    const float q00 = 0.25f * dt4 * sa2;
    const float q01 = 0.5f  * dt3 * sa2;
    const float q11 = dt2   * sa2;

    P_x_[0][0] += dt_s * (P_x_[1][0] + P_x_[0][1] + dt_s * P_x_[1][1]) + q00;
    P_x_[0][1] += dt_s * P_x_[1][1] + q01;
    P_x_[1][0] = P_x_[0][1];
    P_x_[1][1] += q11;

    P_y_[0][0] += dt_s * (P_y_[1][0] + P_y_[0][1] + dt_s * P_y_[1][1]) + q00;
    P_y_[0][1] += dt_s * P_y_[1][1] + q01;
    P_y_[1][0] = P_y_[0][1];
    P_y_[1][1] += q11;
}

bool KalmanFilter2D::update_axis(float state[2], float p[2][2], float z_meas, bool is_tof) {
    if (isnan(z_meas) || z_meas <= 0.0f || z_meas > 1500.0f) return false;
    const float r = is_tof ? r_tof_ : r_us_;
    const float innov = z_meas - state[0];
    const float s = p[0][0] + r;
    if (s <= 1e-6f) return false;

    const float d_m_sq = (innov * innov) / s;
    if (d_m_sq > 6.63f) return false; // Gating chi2 99% contra multipath

    const float k0 = p[0][0] / s;
    const float k1 = p[1][0] / s;
    state[0] += k0 * innov;
    state[1] += k1 * innov;

    // Forma de Joseph (I - KH) P (I - KH)^T + K R K^T para estabilidade em float32
    const float a = 1.0f - k0;
    const float c = -k1;
    const float m00 = a * p[0][0];
    const float m01 = a * p[0][1];
    const float m10 = c * p[0][0] + p[1][0];
    const float m11 = c * p[0][1] + p[1][1];

    p[0][0] = m00 * a + k0 * r * k0;
    p[0][1] = m00 * c + m01 + k0 * r * k1;
    p[1][0] = p[0][1];
    p[1][1] = m10 * c + m11 + k1 * r * k1;
    return true;
}

bool KalmanFilter2D::update_x(float z_meas, bool is_tof) {
    return update_axis(x_, P_x_, z_meas, is_tof);
}

bool KalmanFilter2D::update_y(float z_meas, bool is_tof) {
    return update_axis(y_, P_y_, z_meas, is_tof);
}

float ParticleFilterLite32::irwin_hall_normal(float sigma) {
    float acc = 0.0f;
    for (int k = 0; k < 4; ++k) {
        rng_state_ ^= rng_state_ << 13;
        rng_state_ ^= rng_state_ >> 17;
        rng_state_ ^= rng_state_ << 5;
        const float u = static_cast<float>(rng_state_ & 0xFFFFu) / 65536.0f - 0.5f;
        acc += u;
    }
    return acc * 1.73205f * sigma;
}

void ParticleFilterLite32::reset(float init_pos) {
    for (int i = 0; i < N; ++i) {
        particles_[i] = init_pos + (static_cast<float>(i - (N / 2)) * 1.2f);
        weights_[i] = 1.0f / static_cast<float>(N);
    }
}

void ParticleFilterLite32::predict(float vel, float dt_s) {
    const float dt_eff = (dt_s > 0.0f) ? dt_s : 0.02f;
    const float noise_std = fmaxf(1.5f, 5.0f * sqrtf(fmaxf(dt_eff, 0.005f) / 0.02f));
    for (int i = 0; i < N; ++i) {
        particles_[i] += vel * dt_eff + irwin_hall_normal(noise_std);
    }
}

void ParticleFilterLite32::systematic_resample() {
    float new_p[N];
    float cdf = weights_[0];
    int j = 0;
    rng_state_ ^= rng_state_ << 13;
    rng_state_ ^= rng_state_ >> 17;
    rng_state_ ^= rng_state_ << 5;
    const float u0 = (static_cast<float>(rng_state_ & 0xFFFFu) / 65536.0f) / static_cast<float>(N);
    for (int i = 0; i < N; ++i) {
        const float u = u0 + static_cast<float>(i) / static_cast<float>(N);
        while (u > cdf && j < N - 1) {
            ++j;
            cdf += weights_[j];
        }
        new_p[i] = particles_[j] + irwin_hall_normal(0.5f); // Roughening anti-degenerescencia
    }
    for (int i = 0; i < N; ++i) {
        particles_[i] = new_p[i];
        weights_[i] = 1.0f / static_cast<float>(N);
    }
}

void ParticleFilterLite32::update(float z_meas) {
    if (isnan(z_meas) || z_meas <= 0.0f) return;
    float sum = 0.0f;
    for (int i = 0; i < N; ++i) {
        const float err = z_meas - particles_[i];
        weights_[i] *= expf(-0.5f * (err * err) / 64.0f);
        sum += weights_[i];
    }
    if (sum <= 1e-6f) {
        reset(z_meas);
        return;
    }
    for (int i = 0; i < N; ++i) weights_[i] /= sum;

    if (effective_sample_size() < 0.5f * static_cast<float>(N)) {
        systematic_resample();
    }
}

float ParticleFilterLite32::effective_sample_size() const {
    float sum_sq = 0.0f;
    for (int i = 0; i < N; ++i) sum_sq += weights_[i] * weights_[i];
    return 1.0f / fmaxf(sum_sq, 1e-9f);
}

float ParticleFilterLite32::estimate() const {
    float mean = 0.0f;
    for (int i = 0; i < N; ++i) mean += particles_[i] * weights_[i];
    return mean;
}

} // namespace aegis::ai
