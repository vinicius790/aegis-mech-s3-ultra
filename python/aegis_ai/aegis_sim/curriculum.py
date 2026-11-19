# -*- coding: utf-8 -*-
"""Automatic Domain Randomization (DifficultyScheduler) & Teacher-Student Distillation."""

from typing import Dict, Any
import torch
import torch.nn as nn


class DifficultyScheduler:
    """Agendador ADR progressivo (Estagio 0 -> 10) escalando ruido, latencia, temperatura e sag."""

    def __init__(self, max_level: int = 10):
        self.level = 0
        self.max_level = int(max_level)

    def update(self, success_rate: float) -> int:
        if success_rate >= 0.92 and self.level < self.max_level:
            self.level += 1
        elif success_rate < 0.70 and self.level > 0:
            self.level -= 1
        return self.level

    def params(self) -> Dict[str, Any]:
        alpha = self.level / max(1, self.max_level)
        return {
            "level": self.level,
            "temp_c": 25.0 + alpha * 20.0,
            "v_nom": 12.0 - alpha * 5.2,
            "delay_ticks": int(round(alpha * 3)),
            "dropout_prob": 0.01 + alpha * 0.12,
            "friction_mult": 1.0 + alpha * 1.5,
        }


class TeacherStudentDistiller:
    """Destilacao supervisionada simples de Teacher privilegiado para Student embarcado."""

    def __init__(self, obs_student_dim: int = 4, obs_priv_dim: int = 6):
        self.teacher = nn.Sequential(nn.Linear(obs_priv_dim, 16), nn.ReLU(), nn.Linear(16, 1))
        self.student = nn.Sequential(nn.Linear(obs_student_dim, 16), nn.ReLU(), nn.Linear(16, 1))
        with torch.no_grad():
            self.teacher[0].weight.fill_(0.25)
            self.teacher[2].weight.fill_(0.25)

    def distill_step(self, obs_student: torch.Tensor, obs_priv: torch.Tensor) -> float:
        opt = torch.optim.Adam(self.student.parameters(), lr=1e-2)
        with torch.no_grad():
            target = self.teacher(obs_priv)
        loss = 0.0
        for _ in range(10):
            opt.zero_grad()
            pred = self.student(obs_student)
            l_val = nn.functional.mse_loss(pred, target)
            l_val.backward()
            opt.step()
            loss = float(l_val.item())
        return loss
