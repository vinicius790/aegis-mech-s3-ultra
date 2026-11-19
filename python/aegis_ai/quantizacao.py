# -*- coding: utf-8 -*-
"""Bit-Exact Integer Quantization Engine (qz Module) & PyTorch QAT / BN-Folding."""

import numpy as np
import torch
import torch.nn as nn
from typing import Tuple


def round_shift(acc: int, shift: int) -> int:
    """Implementa exatamente a equacao do firmware C++ de arredondamento simetrico:
    round_shift(acc, S) = floor((acc + 2^(S-1)) / 2^S) para acc >= 0
                         -floor(((-acc) + 2^(S-1)) / 2^S) para acc < 0
    """
    if shift == 0:
        return acc
    half = 1 << (shift - 1)
    if acc >= 0:
        return (acc + half) >> shift
    return -(((-acc) + half) >> shift)


def clamp_int8(val: int) -> int:
    """Clamp simetrico estrito [-127, 127] alinhado ao config.PESO_MIN/PESO_MAX e C++."""
    return max(-127, min(127, val))


def fold_conv1d_bn(conv: nn.Conv1d, bn: nn.BatchNorm1d) -> Tuple[torch.Tensor, torch.Tensor]:
    """Dobra analiticamente BatchNorm1d para dentro dos pesos/vieses de Conv1d antes da quantizacao."""
    w = conv.weight.detach().clone()
    b = conv.bias.detach().clone() if conv.bias is not None else torch.zeros(w.size(0), dtype=w.dtype)
    scale = bn.weight.detach() * torch.rsqrt(bn.running_var.detach() + bn.eps)
    w_folded = w * scale.view(-1, 1, 1)
    b_folded = (b - bn.running_mean.detach()) * scale + bn.bias.detach()
    return w_folded, b_folded


def quantizar_pesos_simetrico(tensor_f: np.ndarray) -> Tuple[np.ndarray, float]:
    """Quantiza tensor float para int8 [-127, 127] com escala simetrica."""
    max_abs = float(np.max(np.abs(tensor_f)))
    scale = max_abs / 127.0 if max_abs > 1e-9 else (1.0 / 127.0)
    q = np.clip(np.round(tensor_f / scale), -127, 127).astype(np.int8)
    return q, scale


class FakeQuantizeSTE(torch.autograd.Function):
    """Straight-Through Estimator para Quantization-Aware Training (QAT) simetrico."""

    @staticmethod
    def forward(ctx, x, scale, min_val=-127, max_val=127):
        q = torch.clamp(torch.round(x / scale), min_val, max_val)
        return q * scale

    @staticmethod
    def backward(ctx, grad_output):
        return grad_output, None, None, None


class QuantizedLinear:
    """Camada linear puramente inteira com acumulador int32 e saida quantizada int8."""

    def __init__(self, weights_int8: np.ndarray, bias_int32: np.ndarray, shift_s: int):
        self.w = weights_int8.astype(np.int32)
        self.b = bias_int32.astype(np.int32)
        self.shift = int(shift_s)

    def forward_hidden(self, x_int8: np.ndarray, apply_relu: bool = True) -> np.ndarray:
        acc = self.b + self.w @ x_int8.astype(np.int32)
        out = np.zeros_like(acc, dtype=np.int8)
        for i in range(len(acc)):
            v = round_shift(int(acc[i]), self.shift)
            if apply_relu and v < 0:
                v = 0
            out[i] = clamp_int8(v)
        return out

    def forward_logits(self, x_int8: np.ndarray) -> np.ndarray:
        acc = self.b + self.w @ x_int8.astype(np.int32)
        return acc.astype(np.int32)


class QuantizedConv1d:
    """Convolucao 1D inteira bit-exata com o firmware C++ conv1d_i8."""

    def __init__(self, weights_int8: np.ndarray, bias_int32: np.ndarray, shift_s: int, stride: int = 1, padding: int = 1):
        self.w = weights_int8.astype(np.int32)  # shape (c_out, c_in, k_size)
        self.b = bias_int32.astype(np.int32)    # shape (c_out,)
        self.shift = int(shift_s)
        self.stride = int(stride)
        self.padding = int(padding)

    def forward(self, x_int8: np.ndarray, apply_relu: bool = True) -> np.ndarray:
        # x_int8 shape: (c_in, l_in)
        c_out, c_in, k_size = self.w.shape
        l_in = x_int8.shape[1]
        l_out = (l_in + 2 * self.padding - k_size) // self.stride + 1
        out = np.zeros((c_out, l_out), dtype=np.int8)
        for oc in range(c_out):
            for t in range(l_out):
                acc = int(self.b[oc])
                for ic in range(c_in):
                    for k in range(k_size):
                        ti = t * self.stride - self.padding + k
                        if 0 <= ti < l_in:
                            acc += int(self.w[oc, ic, k]) * int(x_int8[ic, ti])
                v = round_shift(acc, self.shift)
                if apply_relu and v < 0:
                    v = 0
                out[oc, t] = clamp_int8(v)
        return out


class QuantizedDepthwiseConv1d:
    """Convolucao 1D Depthwise inteira bit-exata com o firmware C++ depthwise_conv1d_k3_s2_i8."""

    def __init__(self, weights_int8: np.ndarray, bias_int32: np.ndarray, shift_s: int, stride: int = 2, padding: int = 1):
        self.w = weights_int8.astype(np.int32)  # shape (channels, 1, k_size) ou (channels, k_size)
        if self.w.ndim == 3:
            self.w = self.w[:, 0, :]
        self.b = bias_int32.astype(np.int32)
        self.shift = int(shift_s)
        self.stride = int(stride)
        self.padding = int(padding)

    def forward(self, x_int8: np.ndarray, apply_relu: bool = True) -> np.ndarray:
        channels, k_size = self.w.shape
        l_in = x_int8.shape[1]
        l_out = (l_in + 2 * self.padding - k_size) // self.stride + 1
        out = np.zeros((channels, l_out), dtype=np.int8)
        for c in range(channels):
            for t in range(l_out):
                acc = int(self.b[c])
                for k in range(k_size):
                    ti = t * self.stride - self.padding + k
                    if 0 <= ti < l_in:
                        acc += int(self.w[c, k]) * int(x_int8[c, ti])
                v = round_shift(acc, self.shift)
                if apply_relu and v < 0:
                    v = 0
                out[c, t] = clamp_int8(v)
        return out


def gap_len_i8(x_int8: np.ndarray) -> np.ndarray:
    """Global Average Pooling inteiro com arredondamento simetrico exacto ao C++ gap_len12_to_i8."""
    channels, length = x_int8.shape
    half = length // 2
    out = np.zeros(channels, dtype=np.int8)
    for c in range(channels):
        acc = int(np.sum(x_int8[c].astype(np.int32)))
        q = (acc + half) // length if acc >= 0 else -((-acc + half) // length)
        out[c] = clamp_int8(q)
    return out


def calibrate_optimal_shift(activations: np.ndarray, max_shift: int = 31) -> int:
    """Encontra o shift S1 que melhor preserva a faixa dinamica sem overflow em int8 [-127, 127]."""
    max_val = float(np.max(np.abs(activations)))
    if max_val <= 1e-6:
        return 6
    target_shift = int(np.clip(np.ceil(np.log2(max(1.0, max_val / 127.0))), 1, max_shift))
    return target_shift
