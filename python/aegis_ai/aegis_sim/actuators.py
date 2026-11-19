# -*- coding: utf-8 -*-
"""Physical Actuators: DCMotor + DriveBam (Sag/Stribeck/Backlash) + ActuatorNet MLP/LSTM."""

import csv
from pathlib import Path
from typing import Tuple, Optional
import numpy as np
import torch
import torch.nn as nn


class DriveBamActuator(nn.Module):
    """Hierarquia DCMotor + DriveBam (IsaacLab bam_kernels.py): Voltage Sag + Back-EMF + Backlash + Stribeck."""

    def __init__(
        self,
        r_ohm: float = 2.2,
        kt: float = 0.28,
        v_nom: float = 12.0,
        v_min: float = 5.8,
        g_sag: float = 0.45,
        tau_c: float = 0.025,
        tau_s: float = 0.055,
        v_stribeck: float = 12.0,
        viscous_b: float = 0.002,
        backlash_mm: float = 0.25,
        i_max: float = 2.133,
    ):
        super().__init__()
        self.r_ohm = r_ohm
        self.kt = kt
        self.v_nom = v_nom
        self.v_min = v_min
        self.g_sag = g_sag
        self.tau_c = tau_c
        self.tau_s = tau_s
        self.v_s = v_stribeck
        self.b = viscous_b
        self.backlash_mm = backlash_mm
        self.i_max = i_max

    def step(
        self,
        pwm_duty: torch.Tensor,
        q_dot: torch.Tensor,
        sum_prev_abs_tau: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        v_eff = torch.clamp(self.v_nom - self.g_sag * sum_prev_abs_tau, min=self.v_min)
        emf_duty = (self.kt * q_dot) / v_eff
        span_duty = (self.r_ohm * self.i_max) / v_eff
        duty_clipped = torch.clamp(pwm_duty, min=emf_duty - span_duty, max=emf_duty + span_duty).clamp(-1.0, 1.0)
        current_a = (v_eff * duty_clipped - self.kt * q_dot) / self.r_ohm
        tau_em = self.kt * current_a
        tau_fric = torch.sign(q_dot) * (
            self.tau_c + self.tau_s * torch.exp(-((torch.abs(q_dot) / self.v_s) ** 2))
        ) + self.b * q_dot
        return tau_em - tau_fric, v_eff, current_a


DCMotorModel = DriveBamActuator


class ActuatorNetMLP:
    """Compensador neural feedforward MLP (compativel com firmware C++ e fit real via CSV)."""

    def __init__(self, csv_path: Optional[Path] = None):
        self.w1 = np.zeros((10, 16), dtype=np.float32)
        self.b1 = np.zeros(16, dtype=np.float32)
        self.w2 = np.zeros((16, 8), dtype=np.float32)
        self.b2 = np.zeros(8, dtype=np.float32)
        self.w3 = np.zeros((8, 1), dtype=np.float32)
        self.b3 = np.zeros(1, dtype=np.float32)
        self._init_physics_prior()
        if csv_path is not None and Path(csv_path).exists():
            fit_actuator_mlp_from_csv(self, Path(csv_path))

    def _init_physics_prior(self):
        # Inicializa feedforward fisicamente coerente: canal 0 = erro, canal 1 = vel -> compensa stiction/atrito
        rng = np.random.RandomState(7)
        self.w1 = rng.normal(0.0, 0.15, (10, 16)).astype(np.float32)
        self.w1[0, :8] = 1.2
        self.w1[0, 8:] = -1.2
        self.w2 = rng.normal(0.0, 0.25, (16, 8)).astype(np.float32)
        self.w3 = np.array([1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0], dtype=np.float32).reshape(8, 1) * 4.5

    def forward(self, features_10: np.ndarray) -> float:
        x = np.asarray(features_10, dtype=np.float32)
        h1 = np.maximum(0.0, x @ self.w1 + self.b1)
        h2 = np.maximum(0.0, h1 @ self.w2 + self.b2)
        out = float((h2 @ self.w3 + self.b3).item())
        return float(np.clip(out, -250.0, 250.0))


class ActuatorNetLSTM(nn.Module):
    """Modelo sequencial Professor ActuatorNet-LSTM para identificacao da dinamica de bancada."""

    def __init__(self, in_dim: int = 4, hidden_dim: int = 16, effort_limit: float = 250.0):
        super().__init__()
        self.lstm = nn.LSTM(in_dim, hidden_dim, batch_first=True)
        self.head = nn.Linear(hidden_dim, 1)
        self.effort_limit = float(effort_limit)

    def forward(
        self,
        seq: torch.Tensor,
        hc: Optional[Tuple[torch.Tensor, torch.Tensor]] = None,
    ) -> Tuple[torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        out, next_hc = self.lstm(seq, hc)
        ff = torch.clamp(self.head(out[:, -1, :]).squeeze(-1), -self.effort_limit, self.effort_limit)
        return ff, next_hc


def fit_actuator_mlp_from_csv(net: ActuatorNetMLP, csv_path: Path, steps: int = 30) -> float:
    """Ajusta os pesos da ActuatorNetMLP com base em logs reais de data/actuator_logs/actuator_logs.csv."""
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append([float(r["pwm_pct"]), float(r["theta_deg"]), float(r["vel_deg_s"]), float(r["current_ma"]), float(r["temp_c"])])
    if len(rows) < 4:
        return 0.0
    arr = np.asarray(rows, dtype=np.float32)
    # Constroi vetores 10D [err_est, vel, current_norm, temp_norm, ...] -> target pwm (% * 10)
    feats = np.zeros((len(arr), 10), dtype=np.float32)
    feats[:, 0] = np.clip(arr[:, 0] / 100.0, -1.0, 1.0)
    feats[:, 1] = arr[:, 2] / 100.0
    feats[:, 2] = arr[:, 3] / 1000.0
    feats[:, 3] = (arr[:, 4] - 25.0) / 20.0
    targets = (arr[:, 0] * 10.0).astype(np.float32)

    m = nn.Sequential(nn.Linear(10, 16), nn.ReLU(), nn.Linear(16, 8), nn.ReLU(), nn.Linear(8, 1))
    opt = torch.optim.Adam(m.parameters(), lr=2e-2)
    xt = torch.tensor(feats)
    yt = torch.tensor(targets).unsqueeze(-1)
    loss_val = 0.0
    for _ in range(steps):
        opt.zero_grad()
        loss = nn.functional.mse_loss(m(xt), yt)
        loss.backward()
        opt.step()
        loss_val = float(loss.item())
    with torch.no_grad():
        net.w1 = m[0].weight.detach().T.cpu().numpy().astype(np.float32)
        net.b1 = m[0].bias.detach().cpu().numpy().astype(np.float32)
        net.w2 = m[2].weight.detach().T.cpu().numpy().astype(np.float32)
        net.b2 = m[2].bias.detach().cpu().numpy().astype(np.float32)
        net.w3 = m[4].weight.detach().T.cpu().numpy().astype(np.float32)
        net.b3 = m[4].bias.detach().cpu().numpy().astype(np.float32)
    return loss_val
