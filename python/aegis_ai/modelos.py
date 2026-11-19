# -*- coding: utf-8 -*-
"""Comprehensive Neural Network Architectures + Bit-Exact Integer Tiny1DCNNBio Engine.

1. MLP48 (Baseline)
2. Tiny1DCNNBio (5.023 params pre-fold / 4.879 dobrados BN->Conv1d)
3. QuantizedTiny1DCNNBio (Inferencia 100% inteira bit-a-bit com C++)
4. GestureOODAutoencoder (Open-Set OOD Rejection: 13.288 params)
5. TamperNetIMU (100 Hz MPU6050 Physical Attack Classifier: 797 params)
"""

from typing import Tuple, Dict, Any
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from .quantizacao import (
    QuantizedConv1d,
    QuantizedDepthwiseConv1d,
    QuantizedLinear,
    fold_conv1d_bn,
    gap_len_i8,
    quantizar_pesos_simetrico,
)


class MLP48(nn.Module):
    """Baseline reproduzindo a decupagem do video original (48 -> 64 -> 7)."""

    def __init__(self, in_features: int = 48, hidden: int = 64, num_classes: int = 7):
        super().__init__()
        self.fc1 = nn.Linear(in_features, hidden)
        self.fc2 = nn.Linear(hidden, num_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = F.relu(self.fc1(x))
        return self.fc2(h)


class DepthwiseSeparableConv1d(nn.Module):
    """Convolucao 1D separavel em profundidade para maxima eficiencia no ESP32-S3."""

    def __init__(self, in_ch: int, out_ch: int, kernel_size: int = 3, stride: int = 1, padding: int = 1):
        super().__init__()
        self.depthwise = nn.Conv1d(
            in_ch, in_ch, kernel_size=kernel_size, stride=stride, padding=padding, groups=in_ch
        )
        self.pointwise = nn.Conv1d(in_ch, out_ch, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.pointwise(self.depthwise(x))


class Tiny1DCNNBio(nn.Module):
    """Rede principal 1D-CNN com cabeca dupla: Classificacao de Gestos + Biometria 16-D."""

    def __init__(self, in_channels: int = 5, seq_len: int = 24, num_classes: int = 7, bio_dim: int = 16):
        super().__init__()
        self.in_channels = in_channels
        self.seq_len = seq_len
        self.num_classes = num_classes
        self.bio_dim = bio_dim

        self.conv1 = nn.Conv1d(in_channels, 16, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm1d(16)

        self.ds_conv = DepthwiseSeparableConv1d(16, 24, kernel_size=3, stride=2, padding=1)
        self.bn2 = nn.BatchNorm1d(24)

        self.conv3 = nn.Conv1d(24, 32, kernel_size=3, padding=1)
        self.bn3 = nn.BatchNorm1d(32)

        self.pool = nn.AdaptiveAvgPool1d(1)
        self.fc_shared = nn.Linear(32, 32)
        self.head_class = nn.Linear(32, num_classes)
        self.head_bio = nn.Linear(32, bio_dim)

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        if x.dim() == 2:
            x = x.view(-1, self.seq_len, self.in_channels).permute(0, 2, 1)

        h = F.relu(self.bn1(self.conv1(x)))
        h = F.relu(self.bn2(self.ds_conv(h)))
        h = F.relu(self.bn3(self.conv3(h)))

        h_pooled = self.pool(h).squeeze(-1)
        h_dense = F.relu(self.fc_shared(h_pooled))

        logits = self.head_class(h_dense)
        bio_emb = F.normalize(self.head_bio(h_dense), p=2, dim=-1)
        return logits, bio_emb

    def count_params(self) -> Dict[str, int]:
        unfolded = sum(p.numel() for p in self.parameters())
        bn_params = (
            sum(p.numel() for p in self.bn1.parameters())
            + sum(p.numel() for p in self.bn2.parameters())
            + sum(p.numel() for p in self.bn3.parameters())
        )
        return {"unfolded": unfolded, "folded": unfolded - bn_params}


class QuantizedTiny1DCNNBio:
    """Versao puramente inteira (INT8/INT32) com paridade bit-exata para o ESP32-S3."""

    def __init__(self, layers: Dict[str, Any]):
        self.conv1 = QuantizedConv1d(layers["conv1_w"], layers["conv1_b"], layers["s1"], stride=1, padding=1)
        self.dw = QuantizedDepthwiseConv1d(layers["dw_w"], layers["dw_b"], layers["s_dw"], stride=2, padding=1)
        self.pw = QuantizedConv1d(layers["pw_w"], layers["pw_b"], layers["s_pw"], stride=1, padding=0)
        self.conv3 = QuantizedConv1d(layers["conv3_w"], layers["conv3_b"], layers["s3"], stride=1, padding=1)
        self.fc_shared = QuantizedLinear(layers["fc_w"], layers["fc_b"], layers["s_fc"])
        self.head_class = QuantizedLinear(layers["cls_w"], layers["cls_b"], shift_s=0)
        self.head_bio = QuantizedLinear(layers["bio_w"], layers["bio_b"], shift_s=0)
        self.raw_layers = layers

    @classmethod
    def from_float_model(
        cls,
        model: Tiny1DCNNBio,
        calib_x_float: torch.Tensor,
        in_scale: float = 1.0 / 64.0,
    ) -> "QuantizedTiny1DCNNBio":
        model.eval()
        if calib_x_float.dim() == 2:
            calib_3d = calib_x_float.view(-1, model.seq_len, model.in_channels).permute(0, 2, 1)
        else:
            calib_3d = calib_x_float

        def fit_conv(w_f: np.ndarray, b_f: np.ndarray, s_in: float, act_max: float):
            w_q, s_w = quantizar_pesos_simetrico(w_f)
            s_acc = s_w * s_in
            b_q = np.round(b_f / max(s_acc, 1e-12)).astype(np.int32)
            # Escolhe shift s de modo que s_acc * 2^s >= act_max / 115 (evita saturacao em 127)
            s_out_target = max(act_max / 115.0, s_acc)
            shift = int(np.clip(np.ceil(np.log2(max(1.0, s_out_target / max(s_acc, 1e-12)))), 1, 20))
            s_out = s_acc * (1 << shift)
            return w_q, b_q, shift, s_out

        def fit_linear_head(w_f: np.ndarray, b_f: np.ndarray, s_in: float):
            w_q, s_w = quantizar_pesos_simetrico(w_f)
            s_acc = s_w * s_in
            b_q = np.round(b_f / max(s_acc, 1e-12)).astype(np.int32)
            return w_q, b_q, s_acc

        with torch.no_grad():
            w1_f, b1_f = fold_conv1d_bn(model.conv1, model.bn1)
            a1 = F.relu(F.conv1d(calib_3d, w1_f, b1_f, padding=1))

            wdw_f = model.ds_conv.depthwise.weight.detach()
            bdw_f = model.ds_conv.depthwise.bias.detach()
            adw = F.conv1d(a1, wdw_f, bdw_f, stride=2, padding=1, groups=16)

            wpw_f, bpw_f = fold_conv1d_bn(model.ds_conv.pointwise, model.bn2)
            apw = F.relu(F.conv1d(adw, wpw_f, bpw_f, padding=0))

            w3_f, b3_f = fold_conv1d_bn(model.conv3, model.bn3)
            a3 = F.relu(F.conv1d(apw, w3_f, b3_f, padding=1))

            agap = model.pool(a3).squeeze(-1)
            afc = F.relu(model.fc_shared(agap))

        w1_q, b1_q, s1, so1 = fit_conv(w1_f.cpu().numpy(), b1_f.cpu().numpy(), in_scale, float(a1.abs().max()))
        wdw_q, bdw_q, s_dw, so_dw = fit_conv(wdw_f.cpu().numpy(), bdw_f.cpu().numpy(), so1, float(adw.abs().max()))
        wpw_q, bpw_q, s_pw, so_pw = fit_conv(wpw_f.cpu().numpy(), bpw_f.cpu().numpy(), so_dw, float(apw.abs().max()))
        w3_q, b3_q, s3, so3 = fit_conv(w3_f.cpu().numpy(), b3_f.cpu().numpy(), so_pw, float(a3.abs().max()))

        # GAP mantem escala so3!
        wfc_q, bfc_q, s_fc, so_fc = fit_conv(
            model.fc_shared.weight.detach().cpu().numpy(),
            model.fc_shared.bias.detach().cpu().numpy(),
            so3,
            float(afc.abs().max()),
        )
        wcls_q, bcls_q, s_cls = fit_linear_head(
            model.head_class.weight.detach().cpu().numpy(),
            model.head_class.bias.detach().cpu().numpy(),
            so_fc,
        )
        wbio_q, bbio_q, s_bio = fit_linear_head(
            model.head_bio.weight.detach().cpu().numpy(),
            model.head_bio.bias.detach().cpu().numpy(),
            so_fc,
        )

        return cls({
            "conv1_w": w1_q, "conv1_b": b1_q, "s1": s1,
            "dw_w": wdw_q, "dw_b": bdw_q, "s_dw": s_dw,
            "pw_w": wpw_q, "pw_b": bpw_q, "s_pw": s_pw,
            "conv3_w": w3_q, "conv3_b": b3_q, "s3": s3,
            "fc_w": wfc_q, "fc_b": bfc_q, "s_fc": s_fc,
            "cls_w": wcls_q, "cls_b": bcls_q, "cls_scale": s_cls,
            "bio_w": wbio_q, "bio_b": bbio_q, "bio_scale": s_bio,
        })

    def infer_int8(self, x_vec_120: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Executa inferencia inteira identica bit-a-bit ao C++ aegis::ai::tiny1dcnn_infer_int8."""
        x_seq = np.asarray(x_vec_120, dtype=np.int8).reshape(24, 5).T  # (5, 24)
        h1 = self.conv1.forward(x_seq, apply_relu=True)
        hdw = self.dw.forward(h1, apply_relu=False)
        hpw = self.pw.forward(hdw, apply_relu=True)
        h3 = self.conv3.forward(hpw, apply_relu=True)
        hgap = gap_len_i8(h3)
        hfc = self.fc_shared.forward_hidden(hgap, apply_relu=True)
        logits_i32 = self.head_class.forward_logits(hfc)
        bio_i32 = self.head_bio.forward_logits(hfc).astype(np.float32)
        norm = float(np.linalg.norm(bio_i32))
        bio_unit = bio_i32 / norm if norm > 1e-6 else np.zeros_like(bio_i32)
        return logits_i32, bio_unit


class GestureOODAutoencoder(nn.Module):
    """Autoencoder para rejeicao de gestos fora da distribuicao (Open-Set OOD)."""

    def __init__(self, in_dim: int = 120, latent_dim: int = 16):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(in_dim, 48),
            nn.ReLU(),
            nn.Linear(48, latent_dim),
            nn.ReLU(),
        )
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, 48),
            nn.ReLU(),
            nn.Linear(48, in_dim),
        )

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        z = self.encoder(x)
        x_rec = self.decoder(z)
        return x_rec, z

    def compute_reconstruction_error(self, x: torch.Tensor) -> torch.Tensor:
        x_rec, _ = self.forward(x)
        return F.mse_loss(x_rec, x, reduction="none").mean(dim=-1)


class TamperNetIMU(nn.Module):
    """Classificador leve de ataques fisicos a 100 Hz baseado em janela MPU6050."""

    def __init__(self, in_features: int = 12, num_classes: int = 5):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_features, 24),
            nn.ReLU6(),
            nn.Linear(24, 16),
            nn.ReLU6(),
            nn.Linear(16, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)
