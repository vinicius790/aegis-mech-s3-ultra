# -*- coding: utf-8 -*-
import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.simulador_isaac_style import EventManager, DualConeRayCaster, gerar_trajetoria_sintetica

def test_event_manager_phases():
    em = EventManager(seed=123)
    em.prestartup(temp_c=20.0)
    assert em.c_t > 340.0
    em.startup()
    res = em.reset()
    assert "drift_mm" in res

def test_geracao_trajetoria_formas():
    for f in ["I_v", "I_h", "O", "L", "V", "Z"]:
        t, x, y = gerar_trajetoria_sintetica(f, num_amostras=25)
        assert len(t) == 25
        assert len(x) == 25
        assert len(y) == 25
        # Confere se os limites estao na arena
        assert np.all(x >= 100.0) and np.all(x <= 350.0)
        assert np.all(y >= 100.0) and np.all(y <= 350.0)
