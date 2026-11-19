#pragma once
/**
 * @file kinematics.h
 * @brief Cinematica Direta/Inversa 2-DOF Planar, Jacobiano Analitico (DLS) e Plataforma de Stewart 6-DOF.
 *
 * Implementa a matematica robótica do Plano v3.0 Ultra para controle espacial de efetuadores
 * e bancadas mecatrônicas acopladas ao AEGIS-MECH S3.
 */

#include <cmath>
#include <cstdint>

namespace aegis::mechatronics {

struct Vec2f {
    float x;
    float y;
};

struct Vec3f {
    float x;
    float y;
    float z;
};

class Kinematics2DOF {
public:
    Kinematics2DOF(float l1_mm = 110.0f, float l2_mm = 95.0f)
        : l1_(l1_mm), l2_(l2_mm) {}

    /**
     * @brief Cinematica Direta Planar 2-DOF:
     * x = L1*cos(t1) + L2*cos(t1 + t2)
     * y = L1*sin(t1) + L2*sin(t1 + t2)
     */
    Vec2f forward(float theta1_rad, float theta2_rad) const {
        const float t12 = theta1_rad + theta2_rad;
        return {
            l1_ * std::cos(theta1_rad) + l2_ * std::cos(t12),
            l1_ * std::sin(theta1_rad) + l2_ * std::sin(t12)
        };
    }

    /**
     * @brief Cinematica Inversa Geometrica 2-DOF com clamp de alcancabilidade no anel [|L1-L2|, L1+L2].
     */
    bool inverse(float x_mm, float y_mm, bool elbow_up, float& out_theta1_rad, float& out_theta2_rad) const {
        const float r2 = x_mm * x_mm + y_mm * y_mm;
        const float r = std::sqrt(r2);
        const float max_r = l1_ + l2_;
        const float min_r = std::fabs(l1_ - l2_);
        const bool within_workspace = (r <= max_r && r >= min_r);

        float c2 = (r2 - l1_ * l1_ - l2_ * l2_) / (2.0f * l1_ * l2_);
        if (c2 > 1.0f) c2 = 1.0f;
        if (c2 < -1.0f) c2 = -1.0f;

        const float s2_abs = std::sqrt(std::fmax(0.0f, 1.0f - c2 * c2));
        const float s2 = elbow_up ? -s2_abs : s2_abs;
        out_theta2_rad = std::atan2(s2, c2);

        const float k1 = l1_ + l2_ * c2;
        const float k2 = l2_ * s2;
        out_theta1_rad = std::atan2(y_mm, x_mm) - std::atan2(k2, k1);
        return within_workspace;
    }

    /**
     * @brief Matriz Jacobiana 2x2 J(theta) em ordem row-major [J00, J01, J10, J11].
     */
    void jacobian(float theta1_rad, float theta2_rad, float J_out[4]) const {
        const float s1 = std::sin(theta1_rad);
        const float c1 = std::cos(theta1_rad);
        const float s12 = std::sin(theta1_rad + theta2_rad);
        const float c12 = std::cos(theta1_rad + theta2_rad);

        J_out[0] = -l1_ * s1 - l2_ * s12;
        J_out[1] = -l2_ * s12;
        J_out[2] =  l1_ * c1 + l2_ * c12;
        J_out[3] =  l2_ * c12;
    }

    /**
     * @brief Passo diferencial via Damped Least Squares (Levenberg-Marquardt):
     * dtheta = J^T * (J * J^T + lambda^2 * I)^(-1) * dx
     */
    void damped_least_squares_step(
        float theta1_rad,
        float theta2_rad,
        float dx_mm,
        float dy_mm,
        float lambda,
        float& out_dtheta1,
        float& out_dtheta2
    ) const {
        float J[4];
        jacobian(theta1_rad, theta2_rad, J);

        const float l2_reg = lambda * lambda;
        const float a00 = J[0] * J[0] + J[1] * J[1] + l2_reg;
        const float a01 = J[0] * J[2] + J[1] * J[3];
        const float a11 = J[2] * J[2] + J[3] * J[3] + l2_reg;

        const float det = a00 * a11 - a01 * a01;
        if (std::fabs(det) < 1e-9f) {
            out_dtheta1 = 0.0f;
            out_dtheta2 = 0.0f;
            return;
        }
        const float inv_det = 1.0f / det;
        const float y0 = ( a11 * dx_mm - a01 * dy_mm) * inv_det;
        const float y1 = (-a01 * dx_mm + a00 * dy_mm) * inv_det;

        out_dtheta1 = J[0] * y0 + J[2] * y1;
        out_dtheta2 = J[1] * y0 + J[3] * y1;
    }

private:
    float l1_;
    float l2_;
};

class StewartPlatform6DOF {
public:
    StewartPlatform6DOF(float base_radius_mm = 80.0f, float platform_radius_mm = 60.0f) {
        static constexpr float kBaseAnglesDeg[6] = { -15.0f, 15.0f, 105.0f, 135.0f, 225.0f, 255.0f };
        static constexpr float kPlatAnglesDeg[6] = { -45.0f, 45.0f,  75.0f, 165.0f, 195.0f, 285.0f };
        static constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;

        for (int i = 0; i < 6; ++i) {
            const float ba = kBaseAnglesDeg[i] * kDeg2Rad;
            const float pa = kPlatAnglesDeg[i] * kDeg2Rad;
            base_joints_[i] = { base_radius_mm * std::cos(ba), base_radius_mm * std::sin(ba), 0.0f };
            plat_joints_[i] = { platform_radius_mm * std::cos(pa), platform_radius_mm * std::sin(pa), 0.0f };
        }
    }

    /**
     * @brief Calcula os 6 comprimentos de perna L_i = || R(roll, pitch, yaw) * b_i + p - a_i ||_2.
     */
    void compute_leg_lengths(
        float roll_rad,
        float pitch_rad,
        float yaw_rad,
        float tx_mm,
        float ty_mm,
        float tz_mm,
        float out_lengths_mm[6]
    ) const {
        const float cr = std::cos(roll_rad),  sr = std::sin(roll_rad);
        const float cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);
        const float cy = std::cos(yaw_rad),   sy = std::sin(yaw_rad);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        const float r00 = cy * cp;
        const float r01 = cy * sp * sr - sy * cr;
        const float r02 = cy * sp * cr + sy * sr;
        const float r10 = sy * cp;
        const float r11 = sy * sp * sr + cy * cr;
        const float r12 = sy * sp * cr - cy * sr;
        const float r20 = -sp;
        const float r21 = cp * sr;
        const float r22 = cp * cr;

        for (int i = 0; i < 6; ++i) {
            const Vec3f& b = plat_joints_[i];
            const Vec3f& a = base_joints_[i];
            const float wx = r00 * b.x + r01 * b.y + r02 * b.z + tx_mm - a.x;
            const float wy = r10 * b.x + r11 * b.y + r12 * b.z + ty_mm - a.y;
            const float wz = r20 * b.x + r21 * b.y + r22 * b.z + tz_mm - a.z;
            out_lengths_mm[i] = std::sqrt(wx * wx + wy * wy + wz * wz);
        }
    }

private:
    Vec3f base_joints_[6]{};
    Vec3f plat_joints_[6]{};
};

} // namespace aegis::mechatronics
