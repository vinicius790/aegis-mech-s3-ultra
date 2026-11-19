# -*- coding: utf-8 -*-
"""Testes Abrangentes v3.0 — TinyML Bit-Exact, SupCon, GroupKFold, Grad-CAM, ROC/EER & Manifesto."""

import json
import re
from pathlib import Path
import sys
import numpy as np
import torch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

from aegis_ai.avaliacao import calcular_roc_auc_eer, matriz_confusao_e_f1, validar_paridade_bit_exact
from aegis_ai.config import S_E
from aegis_ai.features import quantizar_vetor_120, reamostrar_trajetoria
from aegis_ai.modelos import QuantizedTiny1DCNNBio, Tiny1DCNNBio, GestureOODAutoencoder, TamperNetIMU
from aegis_ai.otimizacao import run_optuna_study
from aegis_ai.quantizacao import clamp_int8
from aegis_ai.simulador_isaac_style import gerar_trajetoria_sintetica
from aegis_ai.treino import supervised_cosine_metric_loss, treinar_modelo_tiny
from aegis_ai.xai import calcular_saliencia_temporal, grad_cam_1d


def _parse_c_int_array(text: str, name: str) -> np.ndarray:
    m = re.search(rf"{name}\[[^\]]+\]\s*=\s*\{{([^;]+)\}};", text, re.S)
    assert m is not None, f"Array {name} nao encontrado no header C++"
    vals = [int(x.strip()) for x in m.group(1).replace("\n", " ").split(",") if x.strip()]
    return np.array(vals, dtype=np.int32)


def test_tiny1dcnn_param_counts_exact():
    model = Tiny1DCNNBio()
    counts = model.count_params()
    assert counts["unfolded"] == 5023
    assert counts["folded"] == 4879
    assert sum(p.numel() for p in GestureOODAutoencoder().parameters()) == 13288
    assert sum(p.numel() for p in TamperNetIMU().parameters()) == 797


def test_symmetric_clamp_int8_boundary():
    assert clamp_int8(-128) == -127
    assert clamp_int8(-500) == -127
    assert clamp_int8(500) == 127


def test_manifest_and_exported_headers_bit_exact_20_vectors():
    manifest_p = ROOT / "data" / "sintetico" / "modelo" / "manifest.json"
    manifest = json.loads(manifest_p.read_text(encoding="utf-8"))
    assert manifest["num_params"] == 4879
    assert manifest["num_params_unfolded"] == 5023
    assert len(manifest["model_sha256"]) == 64

    ai_dir = ROOT / "firmware" / "src" / "ai"
    cnn_txt = (ai_dir / "model_cnn.h").read_text(encoding="utf-8")
    tv_txt = (ai_dir / "test_vectors.h").read_text(encoding="utf-8")
    act_txt = (ai_dir / "actuator_model.h").read_text(encoding="utf-8")

    assert "AEGIS_CNN_FOLDED_PARAMS 4879" in cnn_txt
    assert "evaluate_mlp" in act_txt and "0.05f, 0.05f" not in act_txt

    shifts = {}
    for s_key in ("AEGIS_S1_SHIFT", "AEGIS_SDW_SHIFT", "AEGIS_SPW_SHIFT", "AEGIS_S3_SHIFT", "AEGIS_SFC_SHIFT"):
        m = re.search(rf"#define\s+{s_key}\s+(\d+)", cnn_txt)
        assert m is not None
        shifts[s_key] = int(m.group(1))

    layers = {
        "conv1_w": _parse_c_int_array(cnn_txt, "CONV1_W").astype(np.int8).reshape(16, 5, 3),
        "conv1_b": _parse_c_int_array(cnn_txt, "CONV1_B"),
        "s1": shifts["AEGIS_S1_SHIFT"],
        "dw_w": _parse_c_int_array(cnn_txt, "DW_W").astype(np.int8).reshape(16, 3),
        "dw_b": _parse_c_int_array(cnn_txt, "DW_B"),
        "s_dw": shifts["AEGIS_SDW_SHIFT"],
        "pw_w": _parse_c_int_array(cnn_txt, "PW_W").astype(np.int8).reshape(24, 16, 1),
        "pw_b": _parse_c_int_array(cnn_txt, "PW_B"),
        "s_pw": shifts["AEGIS_SPW_SHIFT"],
        "conv3_w": _parse_c_int_array(cnn_txt, "CONV3_W").astype(np.int8).reshape(32, 24, 3),
        "conv3_b": _parse_c_int_array(cnn_txt, "CONV3_B"),
        "s3": shifts["AEGIS_S3_SHIFT"],
        "fc_w": _parse_c_int_array(cnn_txt, "FC_W").astype(np.int8).reshape(32, 32),
        "fc_b": _parse_c_int_array(cnn_txt, "FC_B"),
        "s_fc": shifts["AEGIS_SFC_SHIFT"],
        "cls_w": _parse_c_int_array(cnn_txt, "CLS_W").astype(np.int8).reshape(7, 32),
        "cls_b": _parse_c_int_array(cnn_txt, "CLS_B"),
        "bio_w": _parse_c_int_array(cnn_txt, "BIO_W").astype(np.int8).reshape(16, 32),
        "bio_b": _parse_c_int_array(cnn_txt, "BIO_B"),
    }
    qcnn = QuantizedTiny1DCNNBio(layers)

    in_match = re.search(r"TEST_INPUTS\[AEGIS_NUM_TEST_VECTORS\]\[120\]\s*=\s*\{(.+?)\};", tv_txt, re.S)
    ex_match = re.search(r"EXPECTED_LOGITS\[AEGIS_NUM_TEST_VECTORS\]\[7\]\s*=\s*\{(.+?)\};", tv_txt, re.S)
    assert in_match and ex_match
    in_rows = re.findall(r"\{([^\}]+)\}", in_match.group(1))
    ex_rows = re.findall(r"\{([^\}]+)\}", ex_match.group(1))
    assert len(in_rows) == 20 and len(ex_rows) == 20

    for x_str, y_str in zip(in_rows, ex_rows):
        x_i8 = np.array([int(v) for v in x_str.split(",")], dtype=np.int8)
        exp_logits = np.array([int(v) for v in y_str.split(",")], dtype=np.int32)
        got_logits, _ = qcnn.infer_int8(x_i8)
        assert validar_paridade_bit_exact(got_logits, exp_logits)["passed"] is True


def test_supcon_loss_has_nonzero_gradient():
    raw = torch.randn(12, 16, requires_grad=True)
    emb = torch.nn.functional.normalize(raw, p=2, dim=-1)
    ids = torch.tensor([0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3])
    loss = supervised_cosine_metric_loss(emb, ids, margin=0.35)
    loss.backward()
    assert raw.grad is not None
    assert float(raw.grad.abs().sum()) > 1e-4


def test_curve_z_continuity_and_features_vx_vy():
    t, x, y = gerar_trajetoria_sintetica("Z", num_amostras=60)
    # Verifica continuidade sem salto repentino em torno de s=0.33 (indice ~20)
    assert np.max(np.abs(np.diff(x))) < 25.0
    assert np.max(np.abs(np.diff(y))) < 25.0
    t_r, x_r, y_r, vx, vy = reamostrar_trajetoria(t, x, y, num_pontos=24)
    v120 = quantizar_vetor_120(x_r, y_r, vx, vy, t_r)
    assert v120.dtype == np.int8 and len(v120) == 120
    assert int(np.min(v120)) >= -127
    assert np.any(v120[2::5] != 0)  # vx diferente de zero!


def test_group_kfold_training_and_gradcam_xai():
    res = treinar_modelo_tiny(num_epocas=6, batch_size=16, seed=7, lr=2e-3, run_group_kfold=True)
    assert res["cv_accuracy"] >= 0.70
    sample = res["dataset"].data[0]
    cam = grad_cam_1d(res["model"], sample)
    assert cam.shape == (24,)
    assert abs(float(np.sum(cam)) - 1.0) < 1e-5
    cam_via_wrapper = calcular_saliencia_temporal(sample, np.zeros(7), model=res["model"])
    assert np.allclose(cam, cam_via_wrapper)


def test_optuna_pareto_and_roc_auc_eer():
    study = run_optuna_study(n_trials=3, num_epocas=2, seed=42)
    assert study["n_trials_executed"] == 3
    assert study["feasible"] is True
    assert study["flash_bytes"] <= 32768
    scores = np.array([0.91, 0.94, 0.86, 0.89, 0.10, 0.15, 0.22, 0.18])
    labels = np.array([1, 1, 1, 1, 0, 0, 0, 0])
    roc = calcular_roc_auc_eer(scores, labels)
    assert roc["AUC"] > 0.95 and roc["EER"] < 0.05
    cm = matriz_confusao_e_f1(np.array([0, 1, 2]), np.array([0, 1, 2]), num_classes=3)
    assert cm["accuracy"] == 1.0 and cm["macro_f1"] == 1.0
