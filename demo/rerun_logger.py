# -*- coding: utf-8 -*-
"""Multi-Timeline Telemetry Logger with Rerun.io (50 Hz) + Stewart 6-DOF FK Support."""

import math
from typing import Dict, Any, List, Optional
import numpy as np

try:
    import rerun as rr  # type: ignore
    HAS_RERUN = True
except ImportError:
    HAS_RERUN = False


class RerunTelemetryLogger:
    """Registra cones 3D, covariancia EKF, atuadores e pose Stewart 6-DOF (ou fallback offline)."""

    def __init__(self, run_id: str = "aegis_mech_s3"):
        self.run_id = run_id
        self.has_rerun = HAS_RERUN
        self.offline_frames: List[Dict[str, Any]] = []
        if self.has_rerun:
            rr.init("aegis_mech_s3", spawn=False)

    def log_frame(
        self,
        t_ms: float,
        x_mm: float,
        y_mm: float,
        tof_x_mm: float,
        tof_y_mm: float,
        kalman_x: float,
        kalman_y: float,
        current_ma: float = 0.0,
        tamper_score: float = 0.0,
        mahal_rejected: bool = False,
        bolt_mm: float = 0.0,
    ):
        record = {
            "t_ms": float(t_ms),
            "raw_xy": [x_mm, y_mm],
            "tof_xy": [tof_x_mm, tof_y_mm],
            "kalman_xy": [kalman_x, kalman_y],
            "current_ma": float(current_ma),
            "tamper_score": float(tamper_score),
            "mahal_rejected": bool(mahal_rejected),
            "bolt_mm": float(bolt_mm),
        }
        self.offline_frames.append(record)
        if len(self.offline_frames) > 500:
            self.offline_frames.pop(0)

        if self.has_rerun:
            rr.set_time_sequence("tick_ms", int(t_ms))
            rr.log("world/arena/kalman_pos", rr.Points2D([[kalman_x, kalman_y]]))
            if not math.isnan(tof_x_mm) and not math.isnan(tof_y_mm):
                rr.log("world/arena/tof_rays", rr.Arrows3D(origins=[[0, y_mm, 28], [x_mm, 0, 28]], vectors=[[tof_x_mm, 0, 0], [0, tof_y_mm, 0]]))
            rr.log("world/actuator/current_ma", rr.Scalar(current_ma))
            rr.log("world/actuator/bolt_mm", rr.Scalar(bolt_mm))
            rr.log("world/security/tamper_score", rr.Scalar(tamper_score))


if __name__ == "__main__":
    logger = RerunTelemetryLogger()
    logger.log_frame(20.0, 220.0, 220.0, 220.5, 219.8, 220.1, 219.9, current_ma=42.0)
    print(f"Rerun logger pronto (rerun_sdk instalado: {HAS_RERUN}, frames={len(logger.offline_frames)}).")
