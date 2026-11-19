# -*- coding: utf-8 -*-
"""Vision 2FA Package: Pinhole Calibration, ArUco+HSV, MobileNet/YOLO IoU, MediaPipe & Optical Tracker."""

from .camera_calib import CameraCalibrator
from .aruco_pose import (
    rvec_to_quaternion,
    solve_aruco_pose_sim,
    render_synthetic_aruco_badge,
    detect_aruco_and_verify_token,
)
from .yolo_ondemand import calcula_iou, DangerDetector, YOLOOnDemandGuard
from .mediapipe_teacher import MediaPipeTeacher
from .optical_tracker import DogTracker, auto_canny, verify_bolt_alignment

__all__ = [
    "CameraCalibrator",
    "rvec_to_quaternion",
    "solve_aruco_pose_sim",
    "render_synthetic_aruco_badge",
    "detect_aruco_and_verify_token",
    "calcula_iou",
    "DangerDetector",
    "YOLOOnDemandGuard",
    "MediaPipeTeacher",
    "DogTracker",
    "auto_canny",
    "verify_bolt_alignment",
]
