# -*- coding: utf-8 -*-
"""Teacher-Student Ground Truth Generator via MediaPipe Hands (with OpenCV Optical Fallback)."""

from typing import List, Dict, Any, Tuple
import numpy as np
import cv2
from ..config import ARENA_X_MIN, ARENA_X_MAX, ARENA_Y_MIN, ARENA_Y_MAX

try:
    import mediapipe as mp  # type: ignore
    HAS_MEDIAPIPE = hasattr(mp, "solutions")
except ImportError:
    HAS_MEDIAPIPE = False


class MediaPipeTeacher:
    """Extrai 21 marcos da mao e projeta marco 9 (middle_mcp) no espaco metrico da arena."""

    def __init__(self):
        self.initialized = True
        self.has_mediapipe = HAS_MEDIAPIPE
        self._hands = mp.solutions.hands.Hands(max_num_hands=1) if HAS_MEDIAPIPE else None

    @staticmethod
    def project_uv_to_arena_mm(u_norm: float, v_norm: float) -> Tuple[float, float]:
        x_mm = ARENA_X_MIN + max(0.0, min(1.0, u_norm)) * (ARENA_X_MAX - ARENA_X_MIN)
        y_mm = ARENA_Y_MIN + max(0.0, min(1.0, v_norm)) * (ARENA_Y_MAX - ARENA_Y_MIN)
        return float(x_mm), float(y_mm)

    def get_hand_landmarks(self, frame_bgr: np.ndarray) -> List[Dict[str, float]]:
        if frame_bgr is None or not isinstance(frame_bgr, np.ndarray) or frame_bgr.size == 0:
            return []

        if self._hands is not None:
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            res = self._hands.process(rgb)
            if res.multi_hand_landmarks:
                lm = res.multi_hand_landmarks[0].landmark
                return [{"x": float(p.x), "y": float(p.y), "z": float(p.z)} for p in lm]

        # Fallback optico deterministico via YCrCb skin/foreground segmentation
        h, w = frame_bgr.shape[:2]
        ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
        mask = cv2.inRange(ycrcb, np.array([40, 133, 77]), np.array([255, 173, 127]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200.0:
            return []
        m = cv2.moments(largest)
        if m["m00"] <= 1e-6:
            return []
        cx = (m["m10"] / m["m00"]) / max(w, 1)
        cy = (m["m01"] / m["m00"]) / max(h, 1)
        # Gera estrutura canonica de 21 marcos centrada no MCP (indice 9)
        return [{"x": float(cx), "y": float(cy), "z": 0.0} for _ in range(21)]

    def estimate_arena_position_mm(self, frame_bgr: np.ndarray) -> Tuple[float, float] | None:
        lms = self.get_hand_landmarks(frame_bgr)
        if len(lms) <= 9:
            return None
        return self.project_uv_to_arena_mm(lms[9]["x"], lms[9]["y"])
