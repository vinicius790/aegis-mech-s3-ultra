# -*- coding: utf-8 -*-
"""Optical Tracker (DogTracker + auto_canny + HoughLinesP + NMS) para Inspecao Visual de Trava."""

import math
from typing import Any, Dict, List, Tuple
import numpy as np
import cv2


class DogTracker:
    """Rastreador Euclidiano quadro-a-quadro sem contagem dupla (portado de robotica-computacional/404)."""

    def __init__(self, max_dist_px: float = 65.0):
        self.max_dist_px = float(max_dist_px)
        self.next_id = 0
        self.objects: Dict[int, Tuple[float, float]] = {}

    def update(self, centroids: List[Tuple[float, float]]) -> Dict[int, Tuple[float, float]]:
        if not centroids:
            self.objects.clear()
            return {}
        if not self.objects:
            for c in centroids:
                self.objects[self.next_id] = c
                self.next_id += 1
            return dict(self.objects)

        matched_ids = set()
        matched_c = set()
        pairs: List[Tuple[float, int, int]] = []
        obj_ids = list(self.objects.keys())
        for oid in obj_ids:
            ox, oy = self.objects[oid]
            for ci, (cx, cy) in enumerate(centroids):
                dist = math.hypot(cx - ox, cy - oy)
                if dist <= self.max_dist_px:
                    pairs.append((dist, oid, ci))
        pairs.sort(key=lambda item: item[0])

        new_objects: Dict[int, Tuple[float, float]] = {}
        for _, oid, ci in pairs:
            if oid not in matched_ids and ci not in matched_c:
                matched_ids.add(oid)
                matched_c.add(ci)
                new_objects[oid] = centroids[ci]

        for ci, c in enumerate(centroids):
            if ci not in matched_c:
                new_objects[self.next_id] = c
                self.next_id += 1

        self.objects = new_objects
        return dict(self.objects)


def auto_canny(gray: np.ndarray, sigma: float = 0.33) -> np.ndarray:
    """Canny automatico portado de robotica-computacional/05-visao-p2/util."""
    v = float(np.median(gray))
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    return cv2.Canny(gray, lower, upper)


def verify_bolt_alignment(frame_bgr: np.ndarray, expected_angle_deg: float = 0.0, tol_deg: float = 12.0) -> Dict[str, Any]:
    """Detecta alinhamento visual do ferrolho mecanico usando auto_canny + HoughLinesP + NMS angular."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    edges = auto_canny(gray, sigma=0.33)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=40, minLineLength=40, maxLineGap=10)
    if lines is None:
        return {"aligned": False, "line_count": 0, "dominant_angle_deg": None}

    angles: List[float] = []
    lengths: List[float] = []
    for l in lines:
        vals = np.asarray(l).flatten()
        x1, y1, x2, y2 = float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
        # Normaliza angulo para [-90, 90]
        if ang > 90.0:
            ang -= 180.0
        elif ang < -90.0:
            ang += 180.0
        angles.append(ang)
        lengths.append(math.hypot(x2 - x1, y2 - y1))

    best_idx = int(np.argmax(lengths))
    dom_ang = angles[best_idx]
    diff = abs(dom_ang - expected_angle_deg)
    diff = min(diff, abs(180.0 - diff))
    return {
        "aligned": bool(diff <= tol_deg),
        "line_count": len(lines),
        "dominant_angle_deg": float(dom_ang),
    }
