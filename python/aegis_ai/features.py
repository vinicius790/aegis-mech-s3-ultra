# -*- coding: utf-8 -*-
"""Feature Extraction, Interpolation, and Q1.6 Quantization Engine."""

import math
import numpy as np
from typing import List, Tuple
from .config import S_E, ARENA_X_MIN, ARENA_X_MAX, ARENA_Y_MIN, ARENA_Y_MAX

def reamostrar_trajetoria(
    t_arr: np.ndarray,
    x_arr: np.ndarray,
    y_arr: np.ndarray,
    num_pontos: int = 24
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Reamostra temporalmente para num_pontos uniformemente espacados em tempo relativo [-1, +1]."""
    if len(t_arr) < 2:
        t_base = np.linspace(-1.0, 1.0, num_pontos)
        return t_base, np.zeros(num_pontos), np.zeros(num_pontos), np.zeros(num_pontos), np.zeros(num_pontos)

    t_min = t_arr[0]
    t_max = t_arr[-1]
    span = t_max - t_min if (t_max - t_min) > 1e-6 else 1.0
    t_norm = 2.0 * (t_arr - t_min) / span - 1.0 # Invariante 3: [-1, +1]

    t_uniform = np.linspace(-1.0, 1.0, num_pontos)
    x_interp = np.interp(t_uniform, t_norm, x_arr)
    y_interp = np.interp(t_uniform, t_norm, y_arr)

    # Derivadas de velocidade vx, vy
    dt = 2.0 / (num_pontos - 1)
    vx = np.gradient(x_interp, dt)
    vy = np.gradient(y_interp, dt)

    return t_uniform, x_interp, y_interp, vx, vy

def quantizar_vetor_120(
    x_pts: np.ndarray,
    y_pts: np.ndarray,
    vx_pts: np.ndarray,
    vy_pts: np.ndarray,
    t_pts: np.ndarray
) -> np.ndarray:
    """Monta o vetor canonico de 120 bytes (24x5) quantizado em int8 Q1.6."""
    assert len(x_pts) == 24 and len(y_pts) == 24
    # Normaliza x e y para [-1, +1] baseado na arena util [120, 320] mm
    x_norm = 2.0 * (x_pts - ARENA_X_MIN) / (ARENA_X_MAX - ARENA_X_MIN) - 1.0
    y_norm = 2.0 * (y_pts - ARENA_Y_MIN) / (ARENA_Y_MAX - ARENA_Y_MIN) - 1.0

    # Normaliza velocidades (clamp em +-1.0)
    vx_norm = np.clip(vx_pts / 500.0, -1.0, 1.0)
    vy_norm = np.clip(vy_pts / 500.0, -1.0, 1.0)

    # Matriz 24x5 canônica: [x, y, vx, vy, t_rel]
    matriz = np.stack([x_norm, y_norm, vx_norm, vy_norm, t_pts], axis=-1) # shape (24, 5)
    vetor_float = matriz.flatten() # 120 floats

    # Quantizacao simetrica int8: round(v / S_E) com S_E = 1/64
    vetor_int8 = np.clip(np.round(vetor_float / S_E), -127, 127).astype(np.int8)
    return vetor_int8
