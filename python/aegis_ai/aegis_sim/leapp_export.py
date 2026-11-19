# -*- coding: utf-8 -*-
"""LEAPP Pure Functional Edge Exporter & 7-Axis OOD Stress Evaluator."""

from typing import Dict, Any, Tuple
import torch
from .actuators import DriveBamActuator


class LEAPPRunner:
    """Contrato funcional puro step(obs_t, prev_state) -> (u_cmd, next_state) estilo IsaacLab LEAPP."""

    def __init__(self, ema_alpha: float = 0.25, kp: float = 65.0, kd: float = 2.0):
        self.ema_alpha = float(ema_alpha)
        self.kp = float(kp)
        self.kd = float(kd)

    def init_state(self, num_envs: int, device: str = "cpu") -> Dict[str, torch.Tensor]:
        return {
            "ema_u": torch.zeros((num_envs,), device=device),
            "prev_pos": torch.zeros((num_envs,), device=device),
            "initialized": torch.zeros((num_envs,), dtype=torch.bool, device=device),
        }

    def step(
        self,
        target_mm: torch.Tensor,
        pos_meas_mm: torch.Tensor,
        prev_state: Dict[str, torch.Tensor],
        dt_s: float = 0.01,
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        prev_p = torch.where(prev_state["initialized"], prev_state["prev_pos"], pos_meas_mm)
        vel_meas = (pos_meas_mm - prev_p) / max(dt_s, 1e-4)
        err = target_mm - pos_meas_mm
        # Derivada exclusivamente na medicao
        u_raw = torch.clamp((self.kp * err - self.kd * vel_meas) / 1000.0, -1.0, 1.0)
        u_ema = (1.0 - self.ema_alpha) * prev_state["ema_u"] + self.ema_alpha * u_raw
        next_state = {
            "ema_u": u_ema,
            "prev_pos": pos_meas_mm.clone(),
            "initialized": torch.ones_like(prev_state["initialized"]),
        }
        return u_ema, next_state


def robust_eval_7_axes(num_envs: int = 64) -> Dict[str, Any]:
    """Avalia fechamento/abertura do ferrolho em 7 eixos de estresse OOD (meta >= 98%)."""
    axes = [
        ("1_nominal",          {"v_nom": 12.0, "tau_c": 0.025, "drop": 0.00}),
        ("2_temp_extremes",    {"v_nom": 11.5, "tau_c": 0.040, "drop": 0.01}),
        ("3_ups_sag_6V5",      {"v_nom": 6.5,  "tau_c": 0.025, "drop": 0.00}),
        ("4_solar_glare_tof",  {"v_nom": 12.0, "tau_c": 0.025, "drop": 0.05}),
        ("5_i2c_jitter_drop",  {"v_nom": 12.0, "tau_c": 0.030, "drop": 0.10}),
        ("6_friction_2x",      {"v_nom": 12.0, "tau_c": 0.055, "drop": 0.02}),
        ("7_combined_extreme", {"v_nom": 6.8,  "tau_c": 0.050, "drop": 0.08}),
    ]
    runner = LEAPPRunner(ema_alpha=0.35, kp=85.0, kd=2.4)
    report: Dict[str, float] = {}
    torch.manual_seed(42)

    for name, cfg in axes:
        act = DriveBamActuator(v_nom=cfg["v_nom"], tau_c=cfg["tau_c"])
        pos = torch.zeros(num_envs)
        vel = torch.zeros(num_envs)
        state = runner.init_state(num_envs)
        target = torch.full((num_envs,), 28.0)

        for _ in range(200):  # 2.0 s @ 100 Hz (< timeout 2.5s da SecurityFSM!)
            drop_mask = torch.rand(num_envs) < cfg["drop"]
            meas = torch.where(drop_mask, state["prev_pos"], pos)
            u_cmd, state = runner.step(target, meas, state, dt_s=0.01)
            # Compensacao feedforward de Voltage Sag na bateria UPS (12.0 / V_nom)
            u_comp = torch.clamp(u_cmd * (12.0 / max(cfg["v_nom"], 5.0)), -1.0, 1.0)
            tau, _, _ = act.step(u_comp, vel, torch.zeros(num_envs))
            vel = torch.clamp(vel + tau * 65.0 * 0.01, -60.0, 60.0)
            pos = torch.clamp(pos + vel * 0.01, 0.0, 28.5)

        success = float(((torch.abs(pos - 28.0) <= 1.2)).float().mean().item())
        report[name] = success

    min_rate = min(report.values())
    mean_rate = sum(report.values()) / len(report)
    return {"axes": report, "min_success_rate": min_rate, "mean_success_rate": mean_rate, "passed": min_rate >= 0.98}
