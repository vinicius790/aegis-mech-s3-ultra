# -*- coding: utf-8 -*-
"""Optional ROS 2 / Gazebo Harmonic Bridge (Soft-Import rclpy)."""

import math
from typing import Dict, Any, List
from .contratos import TelemetriaFrame

try:
    import rclpy  # type: ignore
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class Ros2TelemetryBridge:
    """Converte pacotes AEGIS-PROTO v1 nos topicos padrao ROS 2 (/scan, /odom, /poweron_yolo)."""

    def __init__(self, frame_id: str = "aegis_arena_link"):
        self.frame_id = frame_id
        self.has_ros2 = HAS_ROS2

    @staticmethod
    def _sanitize_range(val_mm: float, min_mm: float = 20.0, max_mm: float = 1500.0) -> float:
        if math.isnan(val_mm) or val_mm <= 0.0:
            return float("inf")
        return float(max(min_mm, min(max_mm, val_mm)) / 1000.0)

    def build_messages(self, frame: TelemetriaFrame) -> Dict[str, Any]:
        ranges: List[float] = [
            self._sanitize_range(frame.tof_x_mm, min_mm=40.0),
            self._sanitize_range(frame.us_x_mm, min_mm=20.0),
            self._sanitize_range(frame.tof_y_mm, min_mm=40.0),
            self._sanitize_range(frame.us_y_mm, min_mm=20.0),
        ]
        return {
            "/scan": {
                "header": {"frame_id": self.frame_id, "stamp_us": frame.t_cap_us},
                "range_min": 0.02,
                "range_max": 1.50,
                "ranges_m": ranges,
            },
            "/odom": {
                "x_m": frame.kalman_x_mm / 1000.0,
                "y_m": frame.kalman_y_mm / 1000.0,
                "vx_m_s": frame.kalman_vx / 1000.0,
                "vy_m_s": frame.kalman_vy / 1000.0,
            },
            "/poweron_yolo": {"data": bool(frame.mahalanobis_flags != 0)},
        }
