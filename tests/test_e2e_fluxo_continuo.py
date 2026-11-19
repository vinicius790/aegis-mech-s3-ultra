# -*- coding: utf-8 -*-
import pytest
import csv
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.avaliacao import calcular_far_frr
import numpy as np

def test_fluxo_e2e_dataset_integridade():
    fluxo_file = Path(__file__).parent.parent / "data" / "sintetico" / "fluxo_e2e.csv"
    assert fluxo_file.exists()
    with open(fluxo_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        assert header == ["t_ms", "x_mm", "y_mm"]
        rows = list(reader)
        assert len(rows) > 50

def test_far_frr_metrica():
    # Simula distribuicao de scores genuinos vs impostores
    genuinos = np.array([0.92, 0.88, 0.95, 0.91, 0.89])
    impostores = np.array([0.15, 0.22, 0.31, 0.18, 0.25])
    scores = np.concatenate([genuinos, impostores])
    labels = np.concatenate([np.ones_like(genuinos), np.zeros_like(impostores)])
    
    res = calcular_far_frr(scores, labels, threshold=0.75)
    assert res["FAR"] == 0.0 # Zero falso aceite
    assert res["FRR"] == 0.0 # Zero falsa rejeicao
