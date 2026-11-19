# -*- coding: utf-8 -*-
"""Camera Calibration Engine (Pinhole 2-Beacons f=h*D/H, RealSense Defaults & Chessboard)."""

from typing import Dict, Any, List, Tuple
import numpy as np
import cv2


class CameraCalibrator:
    def __init__(self, board_size: Tuple[int, int] = (9, 6), square_size_mm: float = 24.0):
        self.board_size = board_size
        self.square_size = float(square_size_mm)
        self.camera_matrix = np.eye(3, dtype=np.float64)
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
        self.load_realsense_defaults()

    def load_realsense_defaults(self):
        """Intrinsics reais calibrados (portados de robotica-computacional/404/config)."""
        self.camera_matrix = np.array(
            [
                [609.35, 0.0, 323.96],
                [0.0, 611.12, 241.61],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.dist_coeffs = np.array([0.119211, -0.281377, -0.002453, 0.001087, 0.0], dtype=np.float64)

    @staticmethod
    def calibrate_from_beacons(d_known_mm: float, h_physical_mm: float, h_pixels: float) -> float:
        """Calibra distância focal f = (h_px * D) / H_mm."""
        if d_known_mm <= 0.0 or h_physical_mm <= 0.0 or h_pixels <= 0.0:
            raise ValueError("Parametros fisicos e em pixels devem ser estritamente positivos.")
        return float((h_pixels * d_known_mm) / h_physical_mm)

    @staticmethod
    def estimate_metric_distance(focal_px: float, h_physical_mm: float, h_pixels: float) -> float:
        """Estima distancia D = (f * H_mm) / h_px em milimetros."""
        if focal_px <= 0.0 or h_physical_mm <= 0.0 or h_pixels <= 0.0:
            raise ValueError("Parametros de estimativa devem ser positivos.")
        return float((focal_px * h_physical_mm) / h_pixels)

    def calibrate_chessboard(self, gray_frames: List[np.ndarray]) -> Dict[str, Any]:
        """Executa cv2.calibrateCamera usando grade xadrez real."""
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0 : self.board_size[0], 0 : self.board_size[1]].T.reshape(-1, 2) * self.square_size
        objpoints, imgpoints = [], []
        image_size = None
        for g in gray_frames:
            image_size = (g.shape[1], g.shape[0])
            ok, corners = cv2.findChessboardCorners(g, self.board_size, None)
            if ok and corners is not None:
                objpoints.append(objp)
                imgpoints.append(corners)
        if not objpoints or image_size is None:
            return {"calibrated": False, "views_used": 0}
        rms, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
        self.camera_matrix = mtx
        self.dist_coeffs = dist.flatten()
        return {"calibrated": True, "views_used": len(objpoints), "rms": float(rms)}

    def get_intrinsics(self) -> Dict[str, np.ndarray]:
        return {"K": self.camera_matrix.copy(), "D": self.dist_coeffs.copy()}
