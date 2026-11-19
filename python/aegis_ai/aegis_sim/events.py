# -*- coding: utf-8 -*-
"""Domain Randomization Events, Additive Bias Noise & SO(3) Quaternion Perturbations."""

import math
from typing import Dict
import numpy as np
import torch
from ..config import NAN_DROPOUT
from ..psychrometry import speed_of_sound_mps


def quat_mul(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """Multiplicacao de quaternions [w, x, y, z] preservando variedade SO(3)."""
    w1, x1, y1, z1 = q1.unbind(-1)
    w2, x2, y2, z2 = q2.unbind(-1)
    return torch.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dim=-1,
    )


class NoiseModelWithAdditiveBias:
    """Ruido branco passo-a-passo + vies termico persistente amostrado no reset(env_mask)."""

    def __init__(self, noise_std: float = 1.5, bias_std: float = 0.8):
        self.noise_std = noise_std
        self.bias_std = bias_std
        self.bias: torch.Tensor | None = None

    def reset(self, num_envs: int, dim: int, env_mask: torch.Tensor | None = None, device: str = "cpu"):
        if self.bias is None or self.bias.shape != (num_envs, dim):
            self.bias = torch.randn((num_envs, dim), device=device) * self.bias_std
        elif env_mask is not None:
            self.bias[env_mask] = torch.randn((int(env_mask.sum()), dim), device=device) * self.bias_std

    def apply(self, signal: torch.Tensor) -> torch.Tensor:
        if self.bias is None or self.bias.shape != signal.shape:
            self.reset(signal.size(0), signal.size(-1), device=str(signal.device))
        return signal + self.bias + torch.randn_like(signal) * self.noise_std


class ResetSampledQuaternionNoiseModel:
    """Aplica erro de alinhamento de montagem SO(3) por multiplicacao geometrica q' = q_pert * q."""

    def __init__(self, max_angle_rad: float = math.radians(3.0)):
        self.max_angle_rad = float(max_angle_rad)

    def perturb(self, q: torch.Tensor) -> torch.Tensor:
        batch = q.shape[:-1]
        axis = torch.nn.functional.normalize(torch.randn(*batch, 3, device=q.device), dim=-1)
        half_ang = (torch.rand(*batch, 1, device=q.device) - 0.5) * self.max_angle_rad
        q_pert = torch.cat([torch.cos(half_ang), axis * torch.sin(half_ang)], dim=-1)
        out = quat_mul(q_pert, q)
        return torch.nn.functional.normalize(out, dim=-1)


class VectorizedEventManager:
    """Gerenciador vetorizado em 4 fases (prestartup, startup, reset, interval) zero-host-sync."""

    def __init__(self, num_envs: int = 16, device: str = "cpu", seed: int = 42):
        self.num_envs = int(num_envs)
        self.device = device
        torch.manual_seed(seed)
        self.c_t = torch.full((self.num_envs,), 343.2, device=device)
        self.friction_scale = torch.ones((self.num_envs,), device=device)
        self.sensor_bias = torch.zeros((self.num_envs, 2), device=device)
        self.noise_model = NoiseModelWithAdditiveBias(noise_std=1.5, bias_std=1.0)

    def prestartup(self, temp_c: float = 25.0, rh_pct: float = 50.0):
        c_val = speed_of_sound_mps(temp_c, rh_pct=rh_pct)
        self.c_t.fill_(c_val)

    def startup(self):
        self.friction_scale = 0.7 + 0.8 * torch.rand((self.num_envs,), device=self.device)

    def reset(self, env_mask: torch.Tensor | None = None):
        self.noise_model.reset(self.num_envs, 2, env_mask=env_mask, device=self.device)
        if env_mask is None:
            self.sensor_bias = (torch.rand((self.num_envs, 2), device=self.device) - 0.5) * 4.0
        else:
            self.sensor_bias[env_mask] = (torch.rand((int(env_mask.sum()), 2), device=self.device) - 0.5) * 4.0

    def apply_sensor_interval(self, clean_xy: torch.Tensor, prob_dropout: float = 0.03) -> torch.Tensor:
        noisy = self.noise_model.apply(clean_xy) + self.sensor_bias
        drop = torch.rand_like(noisy) < prob_dropout
        return torch.where(drop, torch.full_like(noisy, float("nan")), noisy)


class EventManager:
    """Wrapper escalar compativel com v2.0 (usa engine termohigrometrica v3.0)."""

    def __init__(self, seed: int = 42):
        self.rng = np.random.RandomState(seed)
        self.c_t = 343.2
        self.friction_coeff = 0.25
        self.sensor_noise_std = 1.5

    def prestartup(self, temp_c: float = 25.0, rh_pct: float = 50.0):
        self.c_t = speed_of_sound_mps(temp_c, rh_pct=rh_pct)

    def startup(self):
        self.friction_coeff = float(self.rng.uniform(0.1, 0.4))
        self.sensor_noise_std = float(self.rng.uniform(1.0, 2.5))

    def reset(self) -> Dict[str, float]:
        drift = float(self.rng.uniform(-2.0, 2.0))
        return {"drift_mm": drift}

    def interval_noise(self, val: float, prob_dropout: float = 0.03) -> float:
        if self.rng.uniform(0.0, 1.0) < prob_dropout:
            return NAN_DROPOUT
        noise = self.rng.normal(0.0, self.sensor_noise_std)
        return float(val + noise)
