# -*- coding: utf-8 -*-
import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.vision_2fa.aruco_pose import rvec_to_quaternion, solve_aruco_pose_sim
from aegis_ai.vision_2fa.yolo_ondemand import YOLOOnDemandGuard

def test_quaternion_conversion():
    # Rotacao nula deve produzir quatérnio unitario [1, 0, 0, 0]
    q_ident = rvec_to_quaternion(np.array([0.0, 0.0, 0.0]))
    assert np.allclose(q_ident, [1.0, 0.0, 0.0, 0.0])

    # Rotacao arbitraria em torno de X
    rvec = np.array([np.pi / 2.0, 0.0, 0.0])
    q = rvec_to_quaternion(rvec)
    norm = np.linalg.norm(q)
    assert abs(norm - 1.0) < 1e-6
    # Para 90 graus em X: cos(45) = sqrt(2)/2, sin(45) = sqrt(2)/2
    assert abs(q[0] - np.cos(np.pi / 4.0)) < 1e-5
    assert abs(q[1] - np.sin(np.pi / 4.0)) < 1e-5

def test_solve_aruco_sim():
    pose = solve_aruco_pose_sim(marker_id=42, true_distance_mm=250.0)
    assert pose["id"] == 42
    assert pose["dist_mm"] == 250.0
    assert len(pose["quaternion"]) == 4

def test_yolo_ondemand_guard():
    guard = YOLOOnDemandGuard(timeout_s=0.2)
    assert guard.is_active() is False
    res = guard.process_frame()
    assert res["active"] is False

    guard.trigger(0.2)
    assert guard.is_active() is True
    res_active = guard.process_frame()
    assert res_active["active"] is True
    assert len(res_active["detections"]) > 0
