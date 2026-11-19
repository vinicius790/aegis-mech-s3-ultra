# -*- coding: utf-8 -*-
"""Model Evaluation: FAR/FRR, ROC/AUC/EER, Confusion Matrix & Bit-Exact Parity Check."""

import numpy as np
from typing import Dict, Any, List, Optional


def calcular_far_frr(scores: np.ndarray, labels: np.ndarray, threshold: float) -> Dict[str, float]:
    """Calcula taxa de falso aceite (FAR) e falsa rejeicao (FRR)."""
    scores = np.asarray(scores, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.int64)
    impostores = scores[labels == 0]
    genuinos = scores[labels == 1]

    far = float(np.mean(impostores >= threshold)) if len(impostores) > 0 else 0.0
    frr = float(np.mean(genuinos < threshold)) if len(genuinos) > 0 else 0.0
    return {"FAR": far, "FRR": frr}


def calcular_roc_auc_eer(scores: np.ndarray, labels: np.ndarray) -> Dict[str, Any]:
    """Calcula curva ROC (varredura decrescente de limiares), AUC e Equal Error Rate (EER)."""
    scores = np.asarray(scores, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.int64)
    uniq = sorted(set(scores.tolist()), reverse=True)
    thresholds = [float(scores.max()) + 1.0] + uniq + [float(scores.min()) - 1.0]

    tprs: List[float] = []
    fprs: List[float] = []
    best_diff = 1e9
    eer = 0.0
    eer_thresh = 0.5

    for th in thresholds:
        res = calcular_far_frr(scores, labels, float(th))
        far, frr = res["FAR"], res["FRR"]
        tprs.append(1.0 - frr)
        fprs.append(far)
        diff = abs(far - frr)
        if diff < best_diff:
            best_diff = diff
            eer = 0.5 * (far + frr)
            eer_thresh = float(th)

    trapz_fn = getattr(np, "trapezoid", None) or getattr(np, "trapz")
    auc = float(trapz_fn(np.array(tprs), np.array(fprs)))
    return {"AUC": auc, "EER": float(eer), "eer_threshold": eer_thresh}


def matriz_confusao_e_f1(y_true: np.ndarray, y_pred: np.ndarray, num_classes: int = 7) -> Dict[str, Any]:
    """Calcula matriz de confusao NxN e Macro-F1 sem dependencias obrigatorias extras."""
    cm = np.zeros((num_classes, num_classes), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        if 0 <= t < num_classes and 0 <= p < num_classes:
            cm[int(t), int(p)] += 1

    f1s: List[float] = []
    for c in range(num_classes):
        tp = cm[c, c]
        fp = np.sum(cm[:, c]) - tp
        fn = np.sum(cm[c, :]) - tp
        denom = 2 * tp + fp + fn
        f1s.append((2.0 * tp / denom) if denom > 0 else 0.0)

    acc = float(np.trace(cm) / max(1, np.sum(cm)))
    return {"confusion_matrix": cm.tolist(), "accuracy": acc, "macro_f1": float(np.mean(f1s))}


def validar_paridade_bit_exact(logits_a: np.ndarray, logits_b: np.ndarray) -> Dict[str, Any]:
    """Verifica igualdade absoluta int32 entre dois motores de inferencia."""
    diff = np.abs(np.asarray(logits_a, dtype=np.int32) - np.asarray(logits_b, dtype=np.int32))
    max_diff = int(np.max(diff)) if diff.size > 0 else 0
    return {"passed": max_diff == 0, "max_abs_diff": max_diff}
