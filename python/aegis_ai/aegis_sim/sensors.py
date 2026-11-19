# -*- coding: utf-8 -*-
"""Rigid-Body BaseImu Transport & ContactSensor (IsaacLab Sensors Port)."""

from typing import Dict
import torch


class BaseImu:
    """Simula aceleracao transportada do CoM ate a montagem offset na caixa + ataques tamper."""

    def __init__(self, offset_body: tuple[float, float, float] = (0.09, -0.06, -0.04), device: str = "cpu"):
        self.r_offset = torch.tensor(offset_body, dtype=torch.float32, device=device)
        self.g_world = torch.tensor([0.0, 0.0, 9.80665], dtype=torch.float32, device=device)
        self.device = device

    def compute(
        self,
        a_com_w: torch.Tensor,    # (B, 3) m/s^2
        omega_b: torch.Tensor,    # (B, 3) rad/s
        alpha_b: torch.Tensor,    # (B, 3) rad/s^2
        impact_g: float = 0.0,
    ) -> Dict[str, torch.Tensor]:
        b_sz = a_com_w.size(0)
        r = self.r_offset.unsqueeze(0).expand(b_sz, 3)
        centripetal = torch.cross(omega_b, torch.cross(omega_b, r, dim=-1), dim=-1)
        euler_term = torch.cross(alpha_b, r, dim=-1)
        total_ms2 = a_com_w + euler_term + centripetal + self.g_world.unsqueeze(0)
        accel_g = total_ms2 / 9.80665
        if impact_g > 0.0:
            accel_g = accel_g + torch.tensor([impact_g, 0.5 * impact_g, 0.0], device=self.device)
        return {"accel_g": accel_g, "gyro_dps": torch.rad2deg(omega_b)}


class ContactSensor:
    """Monitor vetorial de forcas normais, atrito tangencial e tempo de contato."""

    def __init__(self, num_envs: int, mu_s: float = 0.35, device: str = "cpu"):
        self.num_envs = int(num_envs)
        self.mu_s = float(mu_s)
        self.contact_time_s = torch.zeros((self.num_envs,), device=device)
        self.air_time_s = torch.zeros((self.num_envs,), device=device)
        self.device = device

    def step(self, normal_force_n: torch.Tensor, dt_s: float = 0.01) -> Dict[str, torch.Tensor]:
        in_contact = normal_force_n > 0.05
        self.contact_time_s = torch.where(in_contact, self.contact_time_s + dt_s, torch.zeros_like(self.contact_time_s))
        self.air_time_s = torch.where(~in_contact, self.air_time_s + dt_s, torch.zeros_like(self.air_time_s))
        friction_max = self.mu_s * torch.clamp(normal_force_n, min=0.0)
        return {"in_contact": in_contact, "friction_limit_n": friction_max, "contact_time_s": self.contact_time_s}
