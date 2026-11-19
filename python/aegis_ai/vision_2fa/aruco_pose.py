# -*- coding: utf-8 -*-
"""Real ArUco 3D Pose (DICT_6X6_1000 + solvePnP) & HSV Anti-Spoofing Token Verifier."""

import math
from typing import Dict, Any, Tuple
import numpy as np
import cv2


def rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    """Converte vetor de Rodrigues rvec para Quaternio unitario q = [w, x, y, z] sem Gimbal Lock."""
    rvec = np.asarray(rvec, dtype=float).flatten()
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rvec / theta
    half = theta / 2.0
    s = math.sin(half)
    return np.array([math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def solve_aruco_pose_sim(
    marker_id: int,
    true_distance_mm: float = 300.0,
    tilt_deg: float = 0.0,
) -> Dict[str, Any]:
    """Referencia analitica deterministica para testes unitarios rapidos."""
    tilt_rad = math.radians(tilt_deg)
    rvec = np.array([tilt_rad, 0.0, 0.0])
    tvec = np.array([0.0, 0.0, true_distance_mm / 1000.0])
    q = rvec_to_quaternion(rvec)
    return {
        "id": marker_id,
        "rvec": rvec,
        "tvec": tvec,
        "quaternion": q,
        "dist_mm": true_distance_mm,
    }


def render_synthetic_aruco_badge(
    marker_id: int = 42,
    border_bgr: Tuple[int, int, int] = (20, 20, 220),
    marker_size_px: int = 160,
    canvas_size: Tuple[int, int] = (640, 480),
) -> np.ndarray:
    """Renderiza imagem BGR sintetica fiel de cracha fisico ArUco DICT_6X6_1000 + borda colorida."""
    w, h = canvas_size
    frame = np.full((h, w, 3), 240, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    marker_gray = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size_px)
    marker_bgr = cv2.cvtColor(marker_gray, cv2.COLOR_GRAY2BGR)

    cx, cy = w // 2, h // 2
    half = marker_size_px // 2
    pad = 24
    cv2.rectangle(frame, (cx - half - pad, cy - half - pad), (cx + half + pad, cy + half + pad), border_bgr, -1)
    cv2.rectangle(frame, (cx - half - 8, cy - half - 8), (cx + half + 8, cy + half + 8), (255, 255, 255), -1)
    frame[cy - half : cy - half + marker_size_px, cx - half : cx - half + marker_size_px] = marker_bgr
    return frame


def detect_aruco_and_verify_token(
    frame_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float = 0.05,
    expected_id: int = 42,
    expected_color: str = "red",
) -> Dict[str, Any]:
    """Deteccao real OpenCV DICT_6X6_1000 + solvePnP + checagem HSV anti-spoofing (rejeita P&B)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {"detected": False, "authorized": False, "reason": "no_marker"}

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if expected_color == "red":
        # Wrap-around vermelho HSV [172..180] | [0..8]
        m1 = cv2.inRange(hsv, np.array([170, 90, 70]), np.array([180, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255]))
        color_mask = cv2.bitwise_or(m1, m2)
    elif expected_color == "green":
        color_mask = cv2.inRange(hsv, np.array([38, 70, 70]), np.array([86, 255, 255]))
    else:
        color_mask = np.zeros_like(gray)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    half = marker_size_m / 2.0
    obj_pts = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )

    h_img, w_img = color_mask.shape
    for idx, mid in enumerate(ids.flatten()):
        if int(mid) != expected_id:
            continue
        c2d = corners[idx][0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, c2d, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        cx, cy = int(np.mean(c2d[:, 0])), int(np.mean(c2d[:, 1]))
        # Amostra cor na vizinhanca externa do marcador (evita centro P&B do ArUco)
        bw = int(max(40.0, np.ptp(c2d[:, 0]) * 0.75))
        bh = int(max(40.0, np.ptp(c2d[:, 1]) * 0.75))
        roi = color_mask[max(0, cy - bh) : min(h_img, cy + bh), max(0, cx - bw) : min(w_img, cx + bw)]
        color_ratio = float(np.count_nonzero(roi)) / max(1, roi.size)
        dist_mm = float(np.linalg.norm(tvec) * 1000.0)
        authorized = bool(color_ratio >= 0.08 and 100.0 <= dist_mm <= 900.0)
        return {
            "detected": True,
            "id": int(mid),
            "rvec": rvec.flatten(),
            "tvec": tvec.flatten(),
            "quaternion": rvec_to_quaternion(rvec),
            "dist_mm": dist_mm,
            "color_ratio": color_ratio,
            "authorized": authorized,
            "reason": "ok" if authorized else "color_or_distance_mismatch",
        }
    return {"detected": False, "authorized": False, "reason": "wrong_id"}
