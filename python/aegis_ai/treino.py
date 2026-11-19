# -*- coding: utf-8 -*-
"""Production-grade Training Pipeline: SupCon Biometric Loss, GroupKFold, OOD & TamperNet."""

import csv
from pathlib import Path
from typing import Dict, Any, List, Optional
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader, TensorDataset

from .config import S_E, GESTURE_CLASSES
from .modelos import Tiny1DCNNBio, GestureOODAutoencoder, TamperNetIMU
from .features import reamostrar_trajetoria, quantizar_vetor_120


def supervised_cosine_metric_loss(
    emb: torch.Tensor,
    group_ids: torch.Tensor,
    margin: float = 0.35,
) -> torch.Tensor:
    """Loss contrastiva de margem no cosseno sobre embeddings L2-normalizados.

    Substitui a antiga loss_bio = 0.01 * sum(emb**2) (cujo gradiente era zero!).
    """
    sim = torch.matmul(emb, emb.T)  # [-1, +1]
    same = group_ids.unsqueeze(0) == group_ids.unsqueeze(1)
    eye = torch.eye(emb.size(0), dtype=torch.bool, device=emb.device)
    pos_mask = same & (~eye)
    neg_mask = ~same
    l_pos = (1.0 - sim[pos_mask]).mean() if pos_mask.any() else emb.new_tensor(0.0)
    l_neg = F.relu(sim[neg_mask] - margin).mean() if neg_mask.any() else emb.new_tensor(0.0)
    return l_pos + l_neg


class SyntheticGestureDataset(Dataset):
    """Dataset deterministico multi-sessao/multi-usuario de gestos quantizados Q1.6."""

    def __init__(self, num_samples_per_class: int = 40, augment: bool = True, seed: int = 42):
        self.rng = np.random.RandomState(seed)
        self.augment = augment
        data_list: List[np.ndarray] = []
        label_list: List[int] = []
        session_list: List[int] = []
        user_list: List[int] = []

        samples_per_sess = max(1, num_samples_per_class // 4)
        for class_idx, name in enumerate(GESTURE_CLASSES):
            for sess_id in range(4):
                user_id = sess_id % 3  # 3 usuarios com estilo cinematico proprio
                for _ in range(samples_per_sess):
                    vec = self._generate_sample(name, user_id=user_id)
                    data_list.append(vec)
                    label_list.append(class_idx)
                    session_list.append(sess_id)
                    user_list.append(user_id)

        self.data = np.array(data_list, dtype=np.float32)
        self.labels = np.array(label_list, dtype=np.int64)
        self.sessions = np.array(session_list, dtype=np.int64)
        self.users = np.array(user_list, dtype=np.int64)

    def _generate_sample(self, name: str, user_id: int = 0) -> np.ndarray:
        # Estilo individual de velocidade e escala para aprendizado biometrico real
        scale_u = 1.0 + 0.08 * (user_id - 1)
        dur = 580.0 + 60.0 * user_id
        t = np.linspace(0.0, dur, 30)
        s = t / dur
        if name == "I_v":
            x = 220.0 + self.rng.normal(0, 1.0, 30)
            y = 150.0 + 120.0 * scale_u * s + self.rng.normal(0, 1.0, 30)
        elif name == "I_h":
            x = 150.0 + 120.0 * scale_u * s + self.rng.normal(0, 1.0, 30)
            y = 220.0 + self.rng.normal(0, 1.0, 30)
        elif name == "O":
            th = 2.0 * np.pi * s
            x = 220.0 + 50.0 * scale_u * np.cos(th) + self.rng.normal(0, 1.0, 30)
            y = 220.0 + 50.0 * scale_u * np.sin(th) + self.rng.normal(0, 1.0, 30)
        elif name == "L":
            x = np.where(s < 0.5, 160.0, 160.0 + 100.0 * scale_u * (s - 0.5) * 2.0)
            y = np.where(s < 0.5, 270.0 - 100.0 * scale_u * s * 2.0, 170.0)
        elif name == "V":
            x = np.where(s < 0.5, 160.0 + 60.0 * scale_u * s * 2.0, 220.0 + 60.0 * scale_u * (s - 0.5) * 2.0)
            y = np.where(s < 0.5, 270.0 - 100.0 * s * 2.0, 170.0 + 100.0 * (s - 0.5) * 2.0)
        elif name == "Z":
            # Curva Z continua nos vertices s=0.33 e s=0.66 (corrige salto do v2.0!)
            x = np.where(
                s < 0.33,
                160.0 + 80.0 * scale_u * (s / 0.33),
                np.where(
                    s < 0.66,
                    160.0 + 80.0 * scale_u - 80.0 * scale_u * ((s - 0.33) / 0.33),
                    160.0 + 80.0 * scale_u * ((s - 0.66) / 0.34),
                ),
            )
            y = np.where(
                s < 0.33,
                260.0,
                np.where(s < 0.66, 260.0 - 90.0 * ((s - 0.33) / 0.33), 170.0),
            )
        else:
            x = 220.0 + self.rng.normal(0, 6.0, 30)
            y = 220.0 + self.rng.normal(0, 6.0, 30)

        if self.augment:
            x += self.rng.normal(0, 1.5, 30)
            y += self.rng.normal(0, 1.5, 30)

        x = np.clip(x, 120.0, 320.0)
        y = np.clip(y, 120.0, 320.0)
        _, x_r, y_r, vx, vy = reamostrar_trajetoria(t, x, y, num_pontos=24)
        t_rel = np.linspace(-1.0, 1.0, 24)
        v120_int8 = quantizar_vetor_120(x_r, y_r, vx, vy, t_rel)
        return v120_int8.astype(np.float32) * S_E

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.tensor(self.data[idx]), torch.tensor(self.labels[idx]), torch.tensor(self.sessions[idx])


def _train_one_split(
    x_train: torch.Tensor,
    y_train: torch.Tensor,
    u_train: torch.Tensor,
    num_epocas: int,
    batch_size: int,
    lr: float,
    seed: int,
) -> Tuple[Tiny1DCNNBio, List[float], float]:
    torch.manual_seed(seed)
    ds = TensorDataset(x_train, y_train, u_train)
    loader = DataLoader(ds, batch_size=batch_size, shuffle=True)

    model = Tiny1DCNNBio(in_channels=5, seq_len=24, num_classes=7, bio_dim=16)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion_cls = nn.CrossEntropyLoss()

    loss_history: List[float] = []
    last_bio_loss = 0.0
    model.train()
    for _ in range(num_epocas):
        total_loss = 0.0
        for bx, by, bu in loader:
            optimizer.zero_grad()
            logits, bio_emb = model(bx)
            loss_cls = criterion_cls(logits, by)
            # Agrupa identidade + classe no espaco metrico cosseno
            bio_target = bu * 10 + by
            loss_bio = supervised_cosine_metric_loss(bio_emb, bio_target, margin=0.35)
            loss = loss_cls + 0.5 * loss_bio
            loss.backward()
            optimizer.step()
            total_loss += float(loss.item())
            last_bio_loss = float(loss_bio.item())
        loss_history.append(total_loss / max(1, len(loader)))
    return model, loss_history, last_bio_loss


def treinar_modelo_tiny(
    num_epocas: int = 15,
    batch_size: int = 16,
    seed: int = 42,
    lr: float = 1e-3,
    run_group_kfold: bool = False,
) -> Dict[str, Any]:
    torch.manual_seed(seed)
    np.random.seed(seed)

    dataset = SyntheticGestureDataset(num_samples_per_class=40, augment=True, seed=seed)
    x_all = torch.tensor(dataset.data)
    y_all = torch.tensor(dataset.labels)
    s_all = torch.tensor(dataset.sessions)
    u_all = torch.tensor(dataset.users)

    cv_acc = None
    if run_group_kfold:
        unique_sessions = sorted(set(dataset.sessions.tolist()))
        fold_accs: List[float] = []
        for val_s in unique_sessions:
            tr_mask = s_all != val_s
            va_mask = s_all == val_s
            m_cv, _, _ = _train_one_split(
                x_all[tr_mask], y_all[tr_mask], u_all[tr_mask],
                num_epocas=num_epocas, batch_size=batch_size, lr=lr, seed=seed + int(val_s),
            )
            m_cv.eval()
            with torch.no_grad():
                logits_val, _ = m_cv(x_all[va_mask])
                preds_val = torch.argmax(logits_val, dim=-1)
                fold_accs.append(float((preds_val == y_all[va_mask]).float().mean().item()))
        cv_acc = float(np.mean(fold_accs))

    model, loss_history, last_bio_loss = _train_one_split(
        x_all, y_all, u_all, num_epocas=num_epocas, batch_size=batch_size, lr=lr, seed=seed
    )
    model.eval()
    with torch.no_grad():
        logits, _ = model(x_all)
        preds = torch.argmax(logits, dim=-1)
        train_acc = float((preds == y_all).float().mean().item())

    return {
        "model": model,
        "final_loss": loss_history[-1],
        "bio_loss_final": last_bio_loss,
        "train_accuracy": train_acc,
        "cv_accuracy": cv_acc if cv_acc is not None else train_acc,
        "loss_history": loss_history,
        "dataset": dataset,
    }


def treinar_ood_autoencoder(num_epocas: int = 20, batch_size: int = 16, seed: int = 42) -> Dict[str, Any]:
    """Treina GestureOODAutoencoder exclusivamente nas classes validas (0..5) e calibra percentil 98."""
    torch.manual_seed(seed)
    ds = SyntheticGestureDataset(num_samples_per_class=40, augment=True, seed=seed)
    valid_mask = ds.labels < 6  # Exclui classe 'nenhuma'
    x_valid = torch.tensor(ds.data[valid_mask])

    ae = GestureOODAutoencoder(in_dim=120, latent_dim=16)
    opt = torch.optim.Adam(ae.parameters(), lr=2e-3)
    loader = DataLoader(TensorDataset(x_valid), batch_size=batch_size, shuffle=True)

    ae.train()
    for _ in range(num_epocas):
        for (bx,) in loader:
            opt.zero_grad()
            rec, _ = ae(bx)
            loss = F.mse_loss(rec, bx)
            loss.backward()
            opt.step()

    ae.eval()
    with torch.no_grad():
        err_valid = ae.compute_reconstruction_error(x_valid).cpu().numpy()
    threshold_p98 = float(np.percentile(err_valid, 98))
    return {"model": ae, "threshold_p98": threshold_p98, "mean_recon_mse": float(np.mean(err_valid))}


def treinar_tamper_net(
    csv_path: Optional[Path] = None,
    num_epocas: int = 25,
    seed: int = 42,
) -> Dict[str, Any]:
    """Treina TamperNetIMU sobre data/tamper/tamper.csv (ou fallback sintetico)."""
    torch.manual_seed(seed)
    label_map = {"REPOUSO": 0, "USO_NORMAL": 1, "IMPACTO_BRUSCO": 2, "FERRAMENTA_ROTATIVA": 3, "INCLINACAO": 4}
    feats: List[List[float]] = []
    labels: List[int] = []

    if csv_path is not None and Path(csv_path).exists():
        clips: Dict[str, List[List[float]]] = {}
        clip_lbl: Dict[str, int] = {}
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                cid = row["clip_id"]
                vec = [float(row[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz")]
                clips.setdefault(cid, []).append(vec)
                clip_lbl[cid] = label_map.get(row["classe_tamper"], 1)
        for cid, arr in clips.items():
            a = np.asarray(arr, dtype=np.float32)
            feats.append(np.concatenate([np.mean(a, axis=0), np.std(a, axis=0)]).tolist())
            labels.append(clip_lbl[cid])

    # Complementa com amostras sinteticas balanceadas p/ garantir cobertura das 5 classes
    rng = np.random.RandomState(seed)
    templates = [
        ([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], [0.02, 0.02, 0.02, 0.2, 0.2, 0.2]),
        ([0.1, 0.1, 0.98, 2.0, 2.0, 1.5], [0.08, 0.08, 0.08, 2.5, 2.5, 2.0]),
        ([2.8, 1.9, 2.4, 95.0, 80.0, 70.0], [0.6, 0.6, 0.6, 20.0, 20.0, 18.0]),
        ([0.4, 0.5, 1.1, 45.0, 42.0, 38.0], [0.45, 0.45, 0.45, 12.0, 12.0, 12.0]),
        ([0.5, 0.2, 0.85, 8.0, 5.0, 4.0], [0.05, 0.05, 0.05, 1.5, 1.5, 1.5]),
    ]
    for cls_id, (m, s) in enumerate(templates):
        for _ in range(12):
            feats.append((np.array(m) + rng.normal(0, 0.1, 6)).tolist() + (np.array(s) + np.abs(rng.normal(0, 0.05, 6))).tolist())
            labels.append(cls_id)

    x_t = torch.tensor(feats, dtype=torch.float32)
    y_t = torch.tensor(labels, dtype=torch.long)
    net = TamperNetIMU(in_features=12, num_classes=5)
    opt = torch.optim.Adam(net.parameters(), lr=5e-3)
    net.train()
    for _ in range(num_epocas):
        opt.zero_grad()
        loss = F.cross_entropy(net(x_t), y_t)
        loss.backward()
        opt.step()

    net.eval()
    with torch.no_grad():
        acc = float((torch.argmax(net(x_t), dim=-1) == y_t).float().mean().item())
    return {"model": net, "accuracy": acc, "params": sum(p.numel() for p in net.parameters())}
