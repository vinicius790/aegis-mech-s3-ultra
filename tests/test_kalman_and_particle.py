# -*- coding: utf-8 -*-
import pytest
import math
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.kalman_particle import KalmanFilter1D, ParticleFilter1D

def test_kalman_mahalanobis_rejection():
    kf = KalmanFilter1D()
    kf.predict(0.02)
    # Leitura plausivel
    assert kf.update(205.0, is_tof=False) is True
    # Outlier absurdo (multipath) deve ser rejeitado
    assert kf.update(850.0, is_tof=False) is False
    # NaN deve ser rejeitado sem explodir covariancia
    assert kf.update(float("nan"), is_tof=True) is False

def test_particle_filter_convergence():
    pf = ParticleFilter1D(num_particles=32, pos_init=200.0)
    for _ in range(10):
        pf.predict(v=0.0, dt=0.02)
        pf.update(z=210.0)
    mean, std = pf.estimate()
    assert abs(mean - 210.0) < 5.0
