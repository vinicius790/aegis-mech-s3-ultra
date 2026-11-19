# -*- coding: utf-8 -*-
import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.actuator_net import ActuatorNetMLP, Kinematics2DOF

def test_actuator_net_inference():
    net = ActuatorNetMLP()
    feats = np.zeros(10, dtype=np.float32)
    ff = net.forward(feats)
    assert isinstance(ff, float)
    assert -250.0 <= ff <= 250.0

def test_kinematics_2dof_singularity_guard():
    kin = Kinematics2DOF(l1=120.0, l2=100.0)
    # Ponto no espaco seguro
    th1, th2 = kin.inverse_kinematics(150.0, 50.0)
    assert not np.isnan(th1) and not np.isnan(th2)
    # Ponto muito distante (deve ser clamped sem gerar NaN)
    th1_far, th2_far = kin.inverse_kinematics(500.0, 500.0)
    assert not np.isnan(th1_far) and not np.isnan(th2_far)
