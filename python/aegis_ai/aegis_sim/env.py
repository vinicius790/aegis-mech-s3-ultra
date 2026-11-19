# -*- coding: utf-8 -*-
"""VectorizedAegisEnv: IsaacLab 3.0 Style Parallel Arena + Bolt Dynamics."""

from typing import Dict, Any, Tuple
import torch
from .buffers import DelayBuffer, CircularBuffer
from .events import VectorizedEventManager
from .raycaster import VectorizedDualConeRayCaster
from .sensors import BaseImu, ContactSensor
from .actuators import DriveBamActuator


class VectorizedAegisEnv:
    """Ambiente vetorizado (B envs) separando estritamente terminated vs time_out."""

    def __init__(self, num_envs: int = 32, max_steps: int = 100, device: str = "cpu", seed: int = 42):
        self.num_envs = int(num_envs)
        self.max_steps = int(max_steps)
        self.device = device
        self.events = VectorizedEventManager(num_envs=self.num_envs, device=device, seed=seed)
        self.raycaster = VectorizedDualConeRayCaster(num_envs=self.num_envs, device=device)
        self.imu = BaseImu(device=device)
        self.contact = ContactSensor(num_envs=self.num_envs, device=device)
        self.actuator = DriveBamActuator()
        self.delay_buf = DelayBuffer(history_len=3, num_envs=self.num_envs, dim=2, hold_prob=0.02, device=device)
        self.circ_buf = CircularBuffer(history_len=5, num_envs=self.num_envs, dim=2, device=device)

        self.step_count = torch.zeros((self.num_envs,), dtype=torch.long, device=device)
        self.hand_xy = torch.full((self.num_envs, 2), 220.0, device=device)
        self.bolt_mm = torch.zeros((self.num_envs,), device=device)
        self.bolt_vel = torch.zeros((self.num_envs,), device=device)

    def reset(self, env_mask: torch.Tensor | None = None) -> torch.Tensor:
        self.events.reset(env_mask=env_mask)
        self.delay_buf.reset(env_mask=env_mask)
        self.circ_buf.reset(env_mask=env_mask)
        if env_mask is None:
            self.step_count.zero_()
            self.hand_xy.fill_(220.0)
            self.bolt_mm.zero_()
            self.bolt_vel.zero_()
        else:
            self.step_count[env_mask] = 0
            self.hand_xy[env_mask] = 220.0
            self.bolt_mm[env_mask] = 0.0
            self.bolt_vel[env_mask] = 0.0
        return self.circ_buf.append(self.hand_xy)

    def step(
        self, pwm_actions: torch.Tensor, hand_vel_xy: torch.Tensor | None = None
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, Dict[str, Any]]:
        dt_s = 0.01
        self.step_count += 1
        if hand_vel_xy is not None:
            self.hand_xy = torch.clamp(self.hand_xy + hand_vel_xy * dt_s, 120.0, 320.0)

        # Raycasting multi-raio vetorizado (16 ToF + 13 US)
        rays = self.raycaster.cast_batch(self.hand_xy, ping_interval_ms=30.0)
        tof_xy = torch.stack([rays["tof_x"], rays["tof_y"]], dim=-1)
        delayed_xy = self.delay_buf.compute(torch.nan_to_num(tof_xy, nan=220.0))
        obs_hist = self.circ_buf.append(delayed_xy)

        # Dinamica mecanica do ferrolho via DriveBam
        tau, v_eff, current_a = self.actuator.step(pwm_actions.flatten(), self.bolt_vel, torch.zeros(self.num_envs, device=self.device))
        self.bolt_vel = torch.clamp(self.bolt_vel + tau * 60.0 * dt_s, -60.0, 60.0)
        self.bolt_mm = torch.clamp(self.bolt_mm + self.bolt_vel * dt_s, 0.0, 28.0)

        stall = (current_a.abs() > 2.0) & (self.bolt_vel.abs() < 0.5)
        terminated = stall
        time_out = self.step_count >= self.max_steps
        rewards = -torch.abs(28.0 - self.bolt_mm) * 0.05
        return obs_hist, rewards, terminated, time_out, {"v_eff": v_eff, "bolt_mm": self.bolt_mm, "rays": rays}
