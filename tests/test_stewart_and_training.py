# -*- coding: utf-8 -*-
import pytest
import numpy as np
import torch
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.actuator_net import StewartPlatform6DOF
from aegis_ai.treino import treinar_modelo_tiny, SyntheticGestureDataset
from aegis_ai.modelos import TamperNetIMU, GestureOODAutoencoder
from aegis_ai.quantizacao import calibrate_optimal_shift
from aegis_ai.xai import calcular_saliencia_temporal, generate_channel_importance

def test_stewart_platform_kinematics():
    stewart = StewartPlatform6DOF(base_radius=120.0, plat_radius=80.0, leg_min=105.0, leg_max=155.0)
    translation = np.array([0.0, 0.0, 125.0])
    euler_deg = np.array([0.0, 0.0, 0.0]) # Em repouso horizontal

    leg_lengths, valid = stewart.solve_inverse(translation, euler_deg)
    assert len(leg_lengths) == 6
    assert valid is True
    assert np.all(leg_lengths >= 105.0) and np.all(leg_lengths <= 155.0)

    # Teste de violacao de limite (inclinacao angular absurda)
    _, invalid = stewart.solve_inverse(np.array([0.0, 0.0, 180.0]), np.array([45.0, 45.0, 0.0]))
    assert invalid is False

def test_tamper_net_forward():
    tamper_net = TamperNetIMU(in_features=12, num_classes=5)
    dummy_input = torch.randn(4, 12)
    out = tamper_net(dummy_input)
    assert out.shape == (4, 5)

def test_ood_autoencoder_reconstruction():
    ae = GestureOODAutoencoder(in_dim=120, latent_dim=16)
    x = torch.randn(2, 120)
    rec, z = ae(x)
    assert rec.shape == (2, 120)
    assert z.shape == (2, 16)
    err = ae.compute_reconstruction_error(x)
    assert len(err) == 2

def test_xai_saliency_and_channels():
    v120 = np.ones(120, dtype=np.float32)
    saliencia = calcular_saliencia_temporal(v120, np.zeros(7))
    assert len(saliencia) == 24
    assert abs(np.sum(saliencia) - 1.0) < 1e-5

    channels = generate_channel_importance(v120)
    assert len(channels) == 5
    assert "pos_x" in channels
    assert abs(sum(channels.values()) - 1.0) < 1e-5

def test_calibrate_optimal_shift():
    acts = np.array([10.0, -250.0, 500.0, 1024.0])
    s = calibrate_optimal_shift(acts)
    assert 1 <= s <= 31
    # 1024 / 127 ~ 8.06 -> ceil(log2(8.06)) = 4
    assert s >= 3

def test_tiny_model_training_cycle():
    # Treina por 2 epocas rapidas para validar pipeline
    res = treinar_modelo_tiny(num_epocas=2, batch_size=8, seed=42)
    assert "model" in res
    assert res["final_loss"] > 0.0
    assert len(res["loss_history"]) == 2
