#pragma once
#include <stdint.h>
#include <math.h>

namespace aegis::ai {

class KalmanFilter2D {
public:
    KalmanFilter2D(float sigma_a = 800.0f, float r_us = 25.0f, float r_tof = 4.0f)
        : sigma_a_(sigma_a), r_us_(r_us), r_tof_(r_tof) {
        reset(200.0f, 200.0f);
    }

    void reset(float init_x, float init_y);
    void predict(float dt_s);
    bool update_x(float z_meas, bool is_tof);
    bool update_y(float z_meas, bool is_tof);

    float pos_x() const { return x_[0]; }
    float vel_x() const { return x_[1]; }
    float pos_y() const { return y_[0]; }
    float vel_y() const { return y_[1]; }

private:
    bool update_axis(float state[2], float p[2][2], float z_meas, bool is_tof);

    float sigma_a_;
    float r_us_;
    float r_tof_;
    float x_[2];
    float P_x_[2][2];
    float y_[2];
    float P_y_[2][2];
};

class ParticleFilterLite32 {
public:
    static constexpr int N = 32;

    ParticleFilterLite32() : rng_state_(0xAE915321u) { reset(200.0f); }
    void reset(float init_pos);
    void predict(float vel, float dt_s);
    void update(float z_meas);
    float estimate() const;
    float effective_sample_size() const;

private:
    float irwin_hall_normal(float sigma);
    void systematic_resample();

    uint32_t rng_state_;
    float particles_[N];
    float weights_[N];
};

} // namespace aegis::ai
