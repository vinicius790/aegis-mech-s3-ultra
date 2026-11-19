# -*- coding: utf-8 -*-
"""Dual Estimation: Joseph-Form Kalman Filter + 32-Particle AMCL Filter."""

import math
import numpy as np
from typing import Tuple
from .config import MAHALANOBIS_THRESHOLD_SQ
from .psychrometry import speed_of_sound_mps


class KalmanFilter1D:
    """Filtro de Kalman linear para 1 eixo com intervalo dt variavel e forma de Joseph."""

    def __init__(self, sigma_a: float = 800.0, r_us: float = 25.0, r_tof: float = 4.0):
        self.sigma_a = sigma_a
        self.r_us = r_us
        self.r_tof = r_tof
        self.x = np.array([200.0, 0.0], dtype=np.float64)
        self.P = np.eye(2, dtype=np.float64) * 10.0
        self.rejections = 0

    def predict(self, dt: float):
        if dt <= 0.0:
            dt = 0.02
        f_mat = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        q_mat = (self.sigma_a ** 2) * np.array(
            [[0.25 * dt ** 4, 0.5 * dt ** 3], [0.5 * dt ** 3, dt ** 2]],
            dtype=np.float64,
        )
        self.x = f_mat @ self.x
        self.P = f_mat @ self.P @ f_mat.T + q_mat

    def update(self, z: float, is_tof: bool) -> bool:
        """Atualiza na Forma de Joseph; retorna True se aceita ou False se outlier/NaN."""
        if math.isnan(z) or z <= 0.0 or z > 1500.0:
            return False

        h_mat = np.array([[1.0, 0.0]], dtype=np.float64)
        r_mat = np.array([[self.r_tof if is_tof else self.r_us]], dtype=np.float64)
        innov = z - float((h_mat @ self.x)[0])
        s_val = float((h_mat @ self.P @ h_mat.T + r_mat)[0, 0])
        if s_val <= 1e-9:
            return False

        d_m_sq = (innov ** 2) / s_val
        if d_m_sq > MAHALANOBIS_THRESHOLD_SQ:
            self.rejections += 1
            return False

        k_gain = (self.P @ h_mat.T) / s_val  # (2, 1)
        self.x = self.x + k_gain.flatten() * innov
        ikh = np.eye(2, dtype=np.float64) - k_gain @ h_mat
        # Atualizacao Forma de Joseph: P <- (I-KH) P (I-KH)^T + K R K^T (preserva simetria PSD)
        self.P = ikh @ self.P @ ikh.T + k_gain @ r_mat @ k_gain.T
        self.P = 0.5 * (self.P + self.P.T)
        return True


class KalmanFilter2D:
    """Fusão assincrona 2D (X, Y) espelhando o kernel embarcado C++."""

    def __init__(self, sigma_a: float = 800.0, r_us: float = 25.0, r_tof: float = 4.0):
        self.kf_x = KalmanFilter1D(sigma_a=sigma_a, r_us=r_us, r_tof=r_tof)
        self.kf_y = KalmanFilter1D(sigma_a=sigma_a, r_us=r_us, r_tof=r_tof)

    def predict(self, dt: float):
        self.kf_x.predict(dt)
        self.kf_y.predict(dt)

    def update_x(self, z: float, is_tof: bool) -> bool:
        return self.kf_x.update(z, is_tof)

    def update_y(self, z: float, is_tof: bool) -> bool:
        return self.kf_y.update(z, is_tof)

    def state(self) -> Tuple[float, float, float, float]:
        return (
            float(self.kf_x.x[0]),
            float(self.kf_y.x[0]),
            float(self.kf_x.x[1]),
            float(self.kf_y.x[1]),
        )


class ParticleFilter1D:
    """Filtro de 32 particulas para robustez a reflexos acusticos multiplos."""

    def __init__(self, num_particles: int = 32, pos_init: float = 200.0, seed: int = 42):
        self.N = num_particles
        self.rng = np.random.RandomState(seed)
        self.particles = self.rng.normal(pos_init, 8.0, self.N)
        self.weights = np.ones(self.N) / self.N

    def predict(self, v: float, dt: float, noise_std: float = 5.0):
        sigma = max(1.5, noise_std * math.sqrt(max(dt, 0.005) / 0.02))
        self.particles += v * dt + self.rng.normal(0.0, sigma, self.N)

    def update(self, z: float, sigma: float = 8.0):
        if math.isnan(z) or z <= 0.0:
            return
        errors = z - self.particles
        self.weights *= np.exp(-0.5 * (errors / sigma) ** 2)
        total = float(np.sum(self.weights))
        if total > 1e-9:
            self.weights /= total
        else:
            self.particles = self.rng.normal(z, 4.0, self.N)
            self.weights = np.ones(self.N) / self.N
            return

        n_eff = 1.0 / float(np.sum(self.weights ** 2))
        if n_eff < self.N / 2.0:
            self.resample()

    def resample(self):
        positions = (np.arange(self.N) + self.rng.uniform(0.0, 1.0)) / self.N
        indexes = np.zeros(self.N, dtype=int)
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        i, j = 0, 0
        while i < self.N:
            if positions[i] <= cumulative_sum[j] or j == self.N - 1:
                indexes[i] = j
                i += 1
            else:
                j += 1
        # Roughening leve previne empobrecimento de particulas identicas
        self.particles = self.particles[indexes] + self.rng.normal(0.0, 0.5, self.N)
        self.weights = np.ones(self.N) / self.N

    def estimate(self) -> Tuple[float, float]:
        mean = float(np.sum(self.particles * self.weights))
        var = float(np.sum(self.weights * (self.particles - mean) ** 2))
        return mean, math.sqrt(max(0.0, var))


__all__ = ["KalmanFilter1D", "KalmanFilter2D", "ParticleFilter1D", "speed_of_sound_mps"]
