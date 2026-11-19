# -*- coding: utf-8 -*-
"""Expansion D: LeRobot / ACT Policy Bridge for 2-DOF Tray (with MAX_AGE & Singularity Clamp)."""

import sys
import time
from pathlib import Path
from typing import Dict, Any, Tuple
import numpy as np

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR / "python") not in sys.path:
    sys.path.insert(0, str(ROOT_DIR / "python"))

from aegis_ai.aegis_sim.kinematics import Kinematics2DOF
from aegis_ai.config import MAX_AGE_MS


class LeRobotBridge:
    def __init__(self, ema_alpha: float = 0.30, l1: float = 120.0, l2: float = 100.0):
        self.connected = True
        self.ema_alpha = float(ema_alpha)
        self.kin = Kinematics2DOF(l1=l1, l2=l2)
        self.last_angles_deg: Tuple[float, float] = (0.0, 0.0)
        self.last_cmd_ts = time.monotonic()

    def execute_policy(self, obs: Dict[str, Any] | np.ndarray, obs_age_ms: float = 0.0) -> Tuple[float, float]:
        """Valida frescor MAX_AGE <= 150ms, evita singularidade e suaviza EMA."""
        if obs_age_ms > MAX_AGE_MS:
            return self.last_angles_deg

        if isinstance(obs, dict):
            x = float(obs.get("target_x_mm", 150.0))
            z = float(obs.get("target_z_mm", 40.0))
        else:
            arr = np.asarray(obs, dtype=float).flatten()
            x = float(arr[0]) if arr.size >= 1 else 150.0
            z = float(arr[1]) if arr.size >= 2 else 40.0

        th1, th2 = self.kin.inverse_kinematics(x, z)
        a1 = (1.0 - self.ema_alpha) * self.last_angles_deg[0] + self.ema_alpha * th1
        a2 = (1.0 - self.ema_alpha) * self.last_angles_deg[1] + self.ema_alpha * th2
        self.last_angles_deg = (float(a1), float(a2))
        self.last_cmd_ts = time.monotonic()
        return self.last_angles_deg
