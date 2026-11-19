# -*- coding: utf-8 -*-
"""Testes Abrangentes v3.0 — Visao 2FA OpenCV Real (ArUco + HSV Anti-Spoof + DangerDetector + Tracker)."""

from pathlib import Path
import sys
import numpy as np
import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

from aegis_ai.vision_2fa import (
    CameraCalibrator,
    DogTracker,
    MediaPipeTeacher,
    YOLOOnDemandGuard,
    detect_aruco_and_verify_token,
    render_synthetic_aruco_badge,
    verify_bolt_alignment,
)


def test_camera_calibrator_beacons_roundtrip():
    cal = CameraCalibrator()
    assert cal.camera_matrix[0, 0] > 600.0
    f_px = cal.calibrate_from_beacons(d_known_mm=500.0, h_physical_mm=100.0, h_pixels=120.0)
    assert abs(f_px - 600.0) < 1e-6
    d_est = cal.estimate_metric_distance(focal_px=f_px, h_physical_mm=100.0, h_pixels=120.0)
    assert abs(d_est - 500.0) < 1e-6


def test_aruco_real_detection_and_bw_spoofing_rejection():
    cal = CameraCalibrator()
    K, D = cal.get_intrinsics()["K"], cal.get_intrinsics()["D"]

    # 1. Cracha vermelho autentico com ArUco DICT_6X6_1000 ID=42
    red_badge = render_synthetic_aruco_badge(marker_id=42, border_bgr=(15, 15, 225))
    res_ok = detect_aruco_and_verify_token(red_badge, K, D, marker_size_m=0.05, expected_id=42, expected_color="red")
    assert res_ok["detected"] is True
    assert res_ok["authorized"] is True
    assert res_ok["color_ratio"] >= 0.08
    assert 100.0 <= res_ok["dist_mm"] <= 900.0

    # 2. Ataque spoofing: impressao em P&B (cinza neutro) -> detecta ID mas BLOQUEIA!
    bw_badge = render_synthetic_aruco_badge(marker_id=42, border_bgr=(150, 150, 150))
    res_spoof = detect_aruco_and_verify_token(bw_badge, K, D, marker_size_m=0.05, expected_id=42, expected_color="red")
    assert res_spoof["detected"] is True
    assert res_spoof["authorized"] is False
    assert res_spoof["color_ratio"] < 0.02


def test_yolo_danger_detector_tailgating_and_tools():
    guard = YOLOOnDemandGuard(timeout_s=2.0)
    guard.trigger(2.0)

    safe_res = guard.process_frame(simulated_boxes=[{"class": "person", "confidence": 0.95, "box": (100, 100, 200, 300)}])
    assert safe_res["tamper_warning"] is False

    tailgating = guard.process_frame(
        simulated_boxes=[
            {"class": "person", "confidence": 0.92, "box": (80, 80, 180, 300)},
            {"class": "person", "confidence": 0.89, "box": (220, 90, 320, 300)},
        ]
    )
    assert tailgating["tailgating_alert"] is True
    assert tailgating["tamper_warning"] is True

    tool_intrusion = guard.process_frame(
        simulated_boxes=[
            {"class": "person", "confidence": 0.92, "box": (100, 100, 200, 300)},
            {"class": "scissors", "confidence": 0.80, "box": (220, 200, 290, 280)},
        ]
    )
    assert tool_intrusion["danger_iou"] > 0.0
    assert tool_intrusion["tamper_warning"] is True


def test_dog_tracker_hough_bolt_and_mediapipe_fallback():
    tr = DogTracker(max_dist_px=40.0)
    t1 = tr.update([(100.0, 100.0), (300.0, 300.0)])
    t2 = tr.update([(105.0, 104.0), (306.0, 298.0)])
    assert set(t1.keys()) == set(t2.keys())

    # Imagem de ferrolho retangular horizontal alinhado
    bolt_img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(bolt_img, (40, 100), (280, 140), (255, 255, 255), -1)
    align = verify_bolt_alignment(bolt_img, expected_angle_deg=0.0, tol_deg=5.0)
    assert align["aligned"] is True and align["line_count"] > 0

    # Patch tom de pele em quadro sintetico para calibrar Fallback do MediaPipeTeacher
    hand_frame = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(hand_frame, (160, 120), 35, (140, 175, 220), -1)  # BGR proximo de tom de pele
    pos = MediaPipeTeacher().estimate_arena_position_mm(hand_frame)
    assert pos is not None
    assert 120.0 <= pos[0] <= 320.0 and 120.0 <= pos[1] <= 320.0
