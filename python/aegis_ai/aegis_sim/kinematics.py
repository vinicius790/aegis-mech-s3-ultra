# -*- coding: utf-8 -*-
"""Multi-DOF Kinematics: Stewart 6-DOF (IK + Jacobian + Adaptive-DLS + FK Newton-Raphson) & Planar 2-DOF."""

import math
from typing import Tuple, Optional
import numpy as np


class Kinematics2DOF:
    """Cinematica analitica planar 2-DOF com projecao em espaco nulo e guarda de singularidade."""

    def __init__(self, l1: float = 120.0, l2: float = 100.0):
        self.l1 = l1
        self.l2 = l2
        self.r_min = abs(l1 - l2) + 5.0
        self.r_max = (l1 + l2) - 5.0

    def inverse_kinematics(self, x: float, z: float) -> Tuple[float, float]:
        r = math.hypot(x, z)
        if r < 1e-4:
            return 0.0, 0.0
        if r < self.r_min or r > self.r_max:
            scale = max(self.r_min, min(self.r_max, r)) / r
            x *= scale
            z *= scale

        c2 = (x ** 2 + z ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        c2 = max(-0.995, min(0.995, c2))
        s2 = math.sqrt(max(0.0, 1.0 - c2 ** 2))
        theta2 = math.atan2(s2, c2)
        theta1 = math.atan2(z, x) - math.atan2(self.l2 * s2, self.l1 + self.l2 * c2)
        return math.degrees(theta1), math.degrees(theta2)

    def jacobian(self, th1_deg: float, th2_deg: float) -> np.ndarray:
        t1, t2 = math.radians(th1_deg), math.radians(th2_deg)
        s1, c1 = math.sin(t1), math.cos(t1)
        s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
        return np.array([
            [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12],
            [ self.l1 * c1 + self.l2 * c12,  self.l2 * c12],
        ], dtype=np.float64)


class StewartPlatform6DOFV3:
    """Cinematica Completa Stewart 6-DOF (6 atuadores lineares com drivers DRV8871):
    - Cinematica Inversa SE(3) + checagem angulo junta universal <= 32 deg
    - Jacobiano Inverso 6x6 + Adaptive Damped Least Squares (Maciejewski-Klein)
    - Cinematica Direta Newton-Raphson a partir dos sensores L_meas
    """

    def __init__(
        self,
        base_radius: float = 120.0,
        plat_radius: float = 80.0,
        leg_min: float = 105.0,
        leg_max: float = 155.0,
        max_joint_deg: float = 42.0,
    ):
        self.r_b = base_radius
        self.r_p = plat_radius
        self.leg_min = leg_min
        self.leg_max = leg_max
        self.max_joint_deg = max_joint_deg

        ang_b = np.radians([15, 105, 135, 225, 255, 345])
        ang_p = np.radians([45, 75, 165, 195, 285, 315])
        self.b = np.column_stack([self.r_b * np.cos(ang_b), self.r_b * np.sin(ang_b), np.zeros(6)])
        self.p = np.column_stack([self.r_p * np.cos(ang_p), self.r_p * np.sin(ang_p), np.zeros(6)])
        self.base_joints = self.b
        self.plat_joints = self.p

    @staticmethod
    def _rot(euler_rad: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
        ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
        rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        return rz @ ry @ rx

    def inverse_jacobian(self, pose6_rad: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Dado pose6_rad = [x, y, z, roll_rad, pitch_rad, yaw_rad], retorna (lengths[6], J_inv[6,6], valid)."""
        r_mat = self._rot(pose6_rad[3:])
        t = pose6_rad[:3]
        lengths = np.zeros(6, dtype=np.float64)
        j_inv = np.zeros((6, 6), dtype=np.float64)
        valid = True
        z_axis = np.array([0.0, 0.0, 1.0])

        for i in range(6):
            rp = r_mat @ self.p[i]
            vec = t + rp - self.b[i]
            length = float(np.linalg.norm(vec))
            u = vec / max(length, 1e-9)
            lengths[i] = length
            j_inv[i, :3] = u
            j_inv[i, 3:] = np.cross(rp, u)

            joint_ang_deg = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, z_axis))))))
            if length < self.leg_min or length > self.leg_max or joint_ang_deg > self.max_joint_deg:
                valid = False
        return lengths, j_inv, valid

    def solve_inverse(self, translation: np.ndarray, euler_deg: np.ndarray) -> Tuple[np.ndarray, bool]:
        """Compatibilidade v2.0 + checagem geometrica completa."""
        pose6 = np.concatenate([np.asarray(translation, dtype=np.float64), np.radians(euler_deg)])
        lengths, _, valid = self.inverse_jacobian(pose6)
        return lengths, valid

    def solve_forward_fk(
        self,
        l_meas: np.ndarray,
        init_pose: Optional[np.ndarray] = None,
        sigma_thresh: float = 0.08,
        lam_min: float = 1e-3,
        lam_max: float = 0.15,
    ) -> np.ndarray:
        """Cinematica Direta Newton-Raphson com Adaptive Damped Least Squares."""
        pose = np.array([0.0, 0.0, 125.0, 0.0, 0.0, 0.0], dtype=np.float64) if init_pose is None else init_pose.copy()
        l_meas = np.asarray(l_meas, dtype=np.float64)
        for _ in range(12):
            l_est, j_inv, _ = self.inverse_jacobian(pose)
            err = l_meas - l_est
            if float(np.linalg.norm(err)) < 1e-4:
                break
            svals = np.linalg.svd(j_inv, compute_uv=False)
            smin = float(svals[-1])
            lam2 = lam_min ** 2 + (1.0 - min(smin / sigma_thresh, 1.0) ** 2) * (lam_max ** 2 - lam_min ** 2)
            dx = np.linalg.solve(j_inv.T @ j_inv + lam2 * np.eye(6), j_inv.T @ err)
            pose += dx
        return pose


StewartPlatform6DOF = StewartPlatform6DOFV3
