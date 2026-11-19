# -*- coding: utf-8 -*-
"""Testes Abrangentes v3.0 — Digital Twin Vetorizado aegis_sim/, DriveBam, FK 6-DOF, LEAPP & 7-OOD."""

from pathlib import Path
import sys
import numpy as np
import torch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

from aegis_ai.aegis_sim import (
    ActuatorNetLSTM,
    ActuatorNetMLP,
    BaseImu,
    CircularBuffer,
    ContactSensor,
    DelayBuffer,
    DifficultyScheduler,
    DriveBamActuator,
    LEAPPRunner,
    NoiseModelWithAdditiveBias,
    ResetSampledQuaternionNoiseModel,
    StewartPlatform6DOFV3,
    TeacherStudentDistiller,
    VectorizedAegisEnv,
    VectorizedDualConeRayCaster,
    robust_eval_7_axes,
)
from aegis_ai.ros2_bridge import Ros2TelemetryBridge
from aegis_ai.contratos import TelemetriaFrame


def test_vectorized_aegis_env_terminated_vs_timeout():
    env = VectorizedAegisEnv(num_envs=8, max_steps=4, seed=42)
    obs0 = env.reset()
    assert obs0.shape == (8, 10)
    for step_idx in range(4):
        obs, rew, term, tout, info = env.step(torch.ones((8, 1)) * 0.5)
        assert obs.shape == (8, 10)
        assert rew.shape == (8,)
        if step_idx < 3:
            assert not bool(tout.any())
        else:
            assert bool(tout.all())
        assert "v_eff" in info and "bolt_mm" in info


def test_dual_cone_raycaster_batch_and_crosstalk():
    rc = VectorizedDualConeRayCaster(num_envs=4)
    xy = torch.tensor([[200.0, 215.0], [250.0, 215.0], [150.0, 220.0], [300.0, 200.0]])
    clean = rc.cast_batch(xy, ping_interval_ms=30.0)
    assert torch.allclose(clean["us_x"][0], torch.tensor(200.0), atol=40.0)
    xtalk = rc.cast_batch(xy, ping_interval_ms=10.0)
    assert torch.all(xtalk["us_x"] <= clean["us_x"])


def test_drive_bam_voltage_sag_and_stribeck():
    act = DriveBamActuator(v_nom=12.0, g_sag=0.5, i_max=2.133)
    tau, v_eff, current_a = act.step(torch.tensor([1.0, 0.2]), torch.tensor([0.0, 10.0]), torch.tensor([4.0, 0.0]))
    assert float(v_eff[0]) < 12.0  # Voltage sag sob carga
    assert abs(float(current_a[0])) <= 2.134  # Hardware R_ILIM DRV8871 clamp
    assert float(tau[0]) > 0.0


def test_stewart_6dof_jacobian_and_fk_newton_raphson():
    stewart = StewartPlatform6DOFV3()
    true_pose = np.array([6.0, -4.0, 130.0, 0.05, -0.04, 0.08], dtype=np.float64)
    lengths, j_inv, valid = stewart.inverse_jacobian(true_pose)
    assert valid is True and j_inv.shape == (6, 6)
    recovered = stewart.solve_forward_fk(lengths)
    assert np.linalg.norm(recovered - true_pose) < 1e-3


def test_leapp_and_robust_eval_7_axes_pass():
    res = robust_eval_7_axes(num_envs=32)
    assert len(res["axes"]) == 7
    assert res["passed"] is True
    assert res["min_success_rate"] >= 0.98

    runner = LEAPPRunner()
    st = runner.init_state(4)
    u, st2 = runner.step(torch.full((4,), 28.0), torch.zeros(4), st)
    assert u.shape == (4,) and bool(st2["initialized"].all())


def test_actuator_mlp_csv_fit_and_lstm():
    net = ActuatorNetMLP(csv_path=ROOT / "data" / "actuator_logs" / "actuator_logs.csv")
    out = net.forward(np.array([0.45, 0.45, 0.32, 0.07, 0, 0, 0, 0, 0, 0], dtype=np.float32))
    assert -250.0 <= out <= 250.0 and abs(out) > 1.0

    lstm = ActuatorNetLSTM(in_dim=4, hidden_dim=16)
    ff, _ = lstm(torch.randn(3, 5, 4))
    assert ff.shape == (3,)


def test_buffers_imu_contact_curriculum_and_ros2():
    db = DelayBuffer(history_len=3, num_envs=4, dim=2)
    db.reset(lags=torch.tensor([0, 1, 2, 1]))
    y1 = db.compute(torch.ones(4, 2))
    assert float(y1[0, 0]) == 1.0 and float(y1[1, 0]) == 0.0

    cb = CircularBuffer(history_len=3, num_envs=2, dim=2)
    h0 = cb.append(torch.full((2, 2), 5.0))
    assert torch.all(h0 == 5.0)  # Zero-branch backfill

    qn = ResetSampledQuaternionNoiseModel().perturb(torch.tensor([[1.0, 0.0, 0.0, 0.0]]))
    assert abs(float(torch.linalg.norm(qn)) - 1.0) < 1e-5

    imu = BaseImu().compute(torch.zeros(2, 3), torch.zeros(2, 3), torch.zeros(2, 3), impact_g=3.2)
    assert float(imu["accel_g"][0, 0]) > 3.0
    cs = ContactSensor(2).step(torch.tensor([10.0, 0.0]))
    assert bool(cs["in_contact"][0]) and not bool(cs["in_contact"][1])

    sched = DifficultyScheduler()
    assert sched.update(0.95) == 1
    assert TeacherStudentDistiller().distill_step(torch.randn(4, 4), torch.randn(4, 6)) >= 0.0

    bridge = Ros2TelemetryBridge()
    msgs = bridge.build_messages(
        TelemetriaFrame(
            msg_type=1, seq_num=1, t_cap_us=100,
            us_x_mm=float("nan"), us_y_mm=200.0, tof_x_mm=200.0, tof_y_mm=200.0,
            kalman_x_mm=200.0, kalman_y_mm=200.0, kalman_vx=0.0, kalman_vy=0.0, temp_c=25.0,
        )
    )
    assert msgs["/scan"]["ranges_m"][1] == float("inf")  # Index 1 = us_x_mm (nan -> inf)
