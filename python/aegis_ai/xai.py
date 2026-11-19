# -*- coding: utf-8 -*-
"""Explainable AI (XAI): True 1D Grad-CAM over Tiny1DCNNBio & Channel Attribution."""

from typing import Dict, Optional
import numpy as np
import torch
import torch.nn.functional as F


def grad_cam_1d(model: torch.nn.Module, vetor_120: np.ndarray, target_class: Optional[int] = None) -> np.ndarray:
    """Computa mapa Grad-CAM 1D verdadeiro sobre a ultima camada convolucional (conv3 -> 24 passos)."""
    model.eval()
    activations = {}
    gradients = {}

    target_layer = getattr(model, "conv3", None)
    if target_layer is None:
        return calcular_saliencia_temporal(vetor_120, np.zeros(7))

    def fwd_hook(_mod, _inp, out):
        activations["conv3"] = out

    def bwd_hook(_mod, _gin, gout):
        gradients["conv3"] = gout[0]

    h_f = target_layer.register_forward_hook(fwd_hook)
    h_b = target_layer.register_full_backward_hook(bwd_hook)
    try:
        x_t = torch.tensor(vetor_120, dtype=torch.float32).view(1, 24, 5).permute(0, 2, 1)
        x_t.requires_grad_(True)
        logits, _ = model(x_t)
        cls_idx = int(torch.argmax(logits, dim=-1).item()) if target_class is None else int(target_class)
        model.zero_grad()
        logits[0, cls_idx].backward()

        act = activations["conv3"].detach()        # (1, 32, 12)
        grad = gradients["conv3"].detach()         # (1, 32, 12)
        weights = grad.mean(dim=-1, keepdim=True)  # (1, 32, 1)
        cam = F.relu((weights * act).sum(dim=1, keepdim=True))  # (1, 1, 12)
        cam_24 = F.interpolate(cam, size=24, mode="linear", align_corners=False).squeeze().cpu().numpy()
        s = float(np.sum(cam_24))
        if s > 1e-6:
            return (cam_24 / s).astype(np.float64)
        return calcular_saliencia_temporal(vetor_120, np.zeros(7))
    finally:
        h_f.remove()
        h_b.remove()


def calcular_saliencia_temporal(
    vetor_120: np.ndarray,
    logits: np.ndarray,
    model: Optional[torch.nn.Module] = None,
    target_class: Optional[int] = None,
) -> np.ndarray:
    """Retorna saliência normalizada nos 24 passos temporais (Grad-CAM real se model fornecido)."""
    if model is not None:
        return grad_cam_1d(model, vetor_120, target_class=target_class)

    matriz = np.asarray(vetor_120, dtype=np.float64).reshape(24, 5)
    saliencia_por_passo = np.sum(np.abs(matriz), axis=-1)
    soma = float(np.sum(saliencia_por_passo))
    if soma > 1e-6:
        return saliencia_por_passo / soma
    return np.ones(24, dtype=np.float64) / 24.0


def generate_channel_importance(vetor_120: np.ndarray) -> Dict[str, float]:
    """Calcula a importancia relativa dos 5 canais [pos_x, pos_y, vel_x, vel_y, time_rel]."""
    matriz = np.asarray(vetor_120, dtype=np.float64).reshape(24, 5)
    channel_energy = np.mean(np.abs(matriz), axis=0)
    total = float(np.sum(channel_energy))
    if total < 1e-6:
        total = 1.0
    names = ["pos_x", "pos_y", "vel_x", "vel_y", "time_rel"]
    return {names[i]: float(channel_energy[i] / total) for i in range(5)}
