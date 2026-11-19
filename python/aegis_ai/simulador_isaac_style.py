# -*- coding: utf-8 -*-
"""Physics Simulator Wrapper (Backwards-Compatible Shim over Vectorized aegis_sim/)."""

from typing import Tuple
import numpy as np

from .aegis_sim.events import EventManager, VectorizedEventManager
from .aegis_sim.raycaster import DualConeRayCaster, VectorizedDualConeRayCaster
from .aegis_sim.env import VectorizedAegisEnv


def gerar_trajetoria_sintetica(
    forma: str,
    num_amostras: int = 30,
    duracao_s: float = 0.6,
    seed: int = 42,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Gera trajetoria realista em tempo continuo dentro da arena [120, 320] mm."""
    rng = np.random.RandomState(seed)
    t = np.linspace(0.0, duracao_s, num_amostras)
    s = t / duracao_s

    if forma == "I_v":
        x = 220.0 + 5.0 * np.sin(np.pi * s)
        y = 140.0 + 140.0 * s
    elif forma == "I_h":
        x = 140.0 + 140.0 * s
        y = 220.0 + 5.0 * np.sin(np.pi * s)
    elif forma == "O":
        theta = 2.0 * np.pi * s
        x = 220.0 + 60.0 * np.cos(theta)
        y = 220.0 + 60.0 * np.sin(theta)
    elif forma == "L":
        x = np.where(s < 0.5, 160.0, 160.0 + 120.0 * (s - 0.5) * 2.0)
        y = np.where(s < 0.5, 280.0 - 120.0 * s * 2.0, 160.0)
    elif forma == "V":
        x = np.where(s < 0.5, 150.0 + 70.0 * s * 2.0, 220.0 + 70.0 * (s - 0.5) * 2.0)
        y = np.where(s < 0.5, 280.0 - 120.0 * s * 2.0, 160.0 + 120.0 * (s - 0.5) * 2.0)
    elif forma == "Z":
        # Curva Z continua sem descontinuidades em s=0.33 e s=0.66
        x = np.where(
            s < 0.33,
            150.0 + 100.0 * (s / 0.33),
            np.where(
                s < 0.66,
                250.0 - 100.0 * ((s - 0.33) / 0.33),
                150.0 + 100.0 * ((s - 0.66) / 0.34),
            ),
        )
        y = np.where(
            s < 0.33,
            270.0,
            np.where(s < 0.66, 270.0 - 100.0 * ((s - 0.33) / 0.33), 170.0),
        )
    else:
        x = 220.0 + rng.normal(0.0, 5.0, num_amostras)
        y = 220.0 + rng.normal(0.0, 5.0, num_amostras)

    return t * 1000.0, x, y


__all__ = [
    "EventManager",
    "DualConeRayCaster",
    "VectorizedEventManager",
    "VectorizedDualConeRayCaster",
    "VectorizedAegisEnv",
    "gerar_trajetoria_sintetica",
]
