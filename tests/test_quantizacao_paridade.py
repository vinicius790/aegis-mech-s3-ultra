# -*- coding: utf-8 -*-
import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.quantizacao import round_shift, clamp_int8, QuantizedLinear

def test_round_shift_properties():
    # Invariante 1 & 2: Arredondamento simetrico bit-a-bit
    assert round_shift(128, 6) == 2
    assert round_shift(-128, 6) == -2
    assert round_shift(32, 6) == 1
    assert round_shift(-32, 6) == -1
    assert round_shift(0, 6) == 0

def test_quantized_linear_layer():
    w = np.array([[10, -5], [4, 8]], dtype=np.int8)
    b = np.array([0, 0], dtype=np.int32)
    layer = QuantizedLinear(w, b, shift_s=2)
    x = np.array([20, 10], dtype=np.int8)
    # acc = [200 - 50, 80 + 80] = [150, 160]
    # round_shift(150, 2) = floor((150+2)/4) = 38
    # round_shift(160, 2) = floor((160+2)/4) = 40
    out = layer.forward_hidden(x, apply_relu=True)
    assert out[0] == 38
    assert out[1] == 40
