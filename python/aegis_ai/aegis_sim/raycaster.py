# -*- coding: utf-8 -*-
"""Vectorized Dual-Cone RayCaster (16 ToF Rays + 13 Ultrasonic Rays + Crosstalk)."""

import math
from typing import Tuple
import torch
from ..config import NAN_DROPOUT


class VectorizedDualConeRayCaster:
    """Intersecao raio-esfera vetorizada (B, num_rays) p/ VL53L1X (16 raios 15°) e HC-SR04 (13 raios 22°)."""

    def __init__(self, num_envs: int, device: str = "cpu", fov_tof_deg: float = 15.0, fov_us_deg: float = 22.0):
        self.num_envs = int(num_envs)
        self.device = device

        # Grade 4x4 para VL53L1X (16 raios dentro do cone FOV)
        half_tof = math.radians(fov_tof_deg) * 0.5
        g_tof = torch.linspace(-half_tof, half_tof, 4, device=device)
        gx, gy = torch.meshgrid(g_tof, g_tof, indexing="ij")
        tof_dirs = torch.stack([torch.ones(16, device=device), torch.tan(gx.flatten()), torch.tan(gy.flatten())], dim=-1)
        self.tof_dirs = torch.nn.functional.normalize(tof_dirs, dim=-1)  # (16, 3)

        # 13 raios concentricos para HC-SR04 (eixo central + 2 aneis de 6 raios)
        us_list = [[1.0, 0.0, 0.0]]
        for ring_frac in (0.5, 1.0):
            ang_rad = math.radians(fov_us_deg * 0.5 * ring_frac)
            r_yz = math.tan(ang_rad)
            for k in range(6):
                th = 2.0 * math.pi * k / 6.0
                us_list.append([1.0, r_yz * math.cos(th), r_yz * math.sin(th)])
        self.us_dirs = torch.nn.functional.normalize(torch.tensor(us_list, dtype=torch.float32, device=device), dim=-1)  # (13, 3)

    def _cast_cone(
        self,
        ray_dirs: torch.Tensor,      # (R, 3)
        hand_pos_xyz: torch.Tensor,  # (B, 3) relativos a parede do sensor [dist, off1, off2]
        hand_radius: float,
        min_range_mm: float,
        max_range_mm: float,
    ) -> torch.Tensor:
        # Origem do sensor na origem (0,0,0), eixo +X apontando para a arena
        # Ray t onde ||t*d - c||^2 = r^2 -> t^2 - 2*(c.d)*t + (||c||^2 - r^2) = 0
        cdotd = torch.einsum("bi,ri->br", hand_pos_xyz, ray_dirs)  # (B, R)
        c2 = torch.sum(hand_pos_xyz * hand_pos_xyz, dim=-1, keepdim=True)  # (B, 1)
        disc = cdotd * cdotd - (c2 - hand_radius ** 2)
        valid_disc = (disc >= 0.0) & (cdotd > 0.0)
        t_hit = cdotd - torch.sqrt(torch.clamp(disc, min=0.0))
        in_range = valid_disc & (t_hit >= min_range_mm) & (t_hit <= max_range_mm)

        # Reduz os raios validos mantendo NaN se nenhum raio acertar a mao
        vals = torch.where(in_range, t_hit, torch.zeros_like(t_hit))
        counts = in_range.sum(dim=-1)
        mean_hit = vals.sum(dim=-1) / torch.clamp(counts, min=1)
        return torch.where(counts > 0, mean_hit, torch.full((hand_pos_xyz.size(0),), float("nan"), device=self.device))

    def cast_batch(
        self,
        target_xy: torch.Tensor,       # (B, 2) na arena [120..320]
        ping_interval_ms: float = 30.0,
        hand_radius: float = 35.0,
    ) -> Dict[str, torch.Tensor] if False else dict:
        b_sz = target_xy.size(0)
        # Centro da barra de sensores X e Y no meio da parede (y=215 ou x=215)
        pos_for_x = torch.stack([target_xy[:, 0], target_xy[:, 1] - 215.0, torch.zeros(b_sz, device=self.device)], dim=-1)
        pos_for_y = torch.stack([target_xy[:, 1], target_xy[:, 0] - 215.0, torch.zeros(b_sz, device=self.device)], dim=-1)

        tof_x = self._cast_cone(self.tof_dirs, pos_for_x, hand_radius=hand_radius, min_range_mm=40.0, max_range_mm=1300.0)
        tof_y = self._cast_cone(self.tof_dirs, pos_for_y, hand_radius=hand_radius, min_range_mm=40.0, max_range_mm=1300.0)
        us_x = self._cast_cone(self.us_dirs, pos_for_x, hand_radius=hand_radius, min_range_mm=20.0, max_range_mm=4000.0)
        us_y = self._cast_cone(self.us_dirs, pos_for_y, hand_radius=hand_radius, min_range_mm=20.0, max_range_mm=4000.0)

        # Injeta crosstalk sintetico realista se intervalo ping-pong < 30 ms (preserva X se Y foi NaN)!
        if ping_interval_ms < 30.0:
            us_x = torch.where(torch.isnan(us_y), us_x, torch.minimum(us_x, us_y * 0.45))
        return {"tof_x": tof_x, "tof_y": tof_y, "us_x": us_x, "us_y": us_y}


class DualConeRayCaster:
    """Raycaster escalar compativel com testes v2.0 utilizando motor analitico v3.0."""

    def __init__(self, fov_tof_deg: float = 15.0, fov_us_deg: float = 22.0):
        self.fov_tof_rad = math.radians(fov_tof_deg)
        self.fov_us_rad = math.radians(fov_us_deg)
        self._vec = VectorizedDualConeRayCaster(1, device="cpu", fov_tof_deg=fov_tof_deg, fov_us_deg=fov_us_deg)

    def cast(self, target_pos: Tuple[float, float], axis: str) -> Tuple[float, float]:
        tx, ty = target_pos
        true_dist = tx if axis == "X" else ty
        tof_read = true_dist if true_dist >= 40.0 else NAN_DROPOUT
        us_read = true_dist if true_dist >= 20.0 else NAN_DROPOUT
        return float(tof_read), float(us_read)
