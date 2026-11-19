# -*- coding: utf-8 -*-
"""Full C++ Header Code Generator (Tiny1DCNNBio Folded + OOD + Tamper + Test Vectors + SHA-256 Manifest)."""

import re
import json
import hashlib
from pathlib import Path
from typing import List, Dict, Any
import numpy as np
import torch

from .config import S_E, GESTURE_CLASSES, FW_VERSION
from .modelos import Tiny1DCNNBio, QuantizedTiny1DCNNBio, GestureOODAutoencoder, TamperNetIMU
from .quantizacao import quantizar_pesos_simetrico

IDENTIFIER_REGEX = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def validar_identificador(nome: str):
    if not IDENTIFIER_REGEX.match(nome):
        raise ValueError(f"Identificador C invalido: {nome}")


def _fmt_i8(arr: np.ndarray) -> str:
    return ", ".join(str(int(x)) for x in np.asarray(arr, dtype=np.int8).flatten())


def _fmt_i32(arr: np.ndarray) -> str:
    return ", ".join(str(int(x)) for x in np.asarray(arr, dtype=np.int32).flatten())


def _fmt_f32(arr: np.ndarray) -> str:
    return ", ".join(f"{float(x):.6f}f" for x in np.asarray(arr, dtype=np.float32).flatten())


def gerar_model_header(
    caminho_out: Path,
    w1: np.ndarray,
    b1: np.ndarray,
    s1: int,
    w_out: np.ndarray,
    b_out: np.ndarray,
):
    """Compatibilidade legada de cabeçalho compacto (usado quando chamado diretamente)."""
    validar_identificador("AEGIS_MODEL_H")
    conteudo = f"""#pragma once
// Cabecalho gerado automaticamente pelo exportador Python da AEGIS AI
// NAO EDITAR ESTE ARQUIVO MANUALMENTE

#include <stdint.h>

#define AEGIS_INPUT_DIM {w1.shape[1]}
#define AEGIS_HIDDEN_DIM {w1.shape[0]}
#define AEGIS_NUM_CLASSES {w_out.shape[0]}
#define AEGIS_S1_SHIFT {s1}

namespace aegis::model {{

alignas(16) static const int8_t W1[AEGIS_HIDDEN_DIM * AEGIS_INPUT_DIM] = {{
    {_fmt_i8(w1)}
}};

alignas(16) static const int32_t B1[AEGIS_HIDDEN_DIM] = {{
    {_fmt_i32(b1)}
}};

alignas(16) static const int8_t W_OUT[AEGIS_NUM_CLASSES * AEGIS_HIDDEN_DIM] = {{
    {_fmt_i8(w_out)}
}};

alignas(16) static const int32_t B_OUT[AEGIS_NUM_CLASSES] = {{
    {_fmt_i32(b_out)}
}};

}} // namespace aegis::model
"""
    caminho_out.write_text(conteudo, encoding="utf-8")


def gerar_model_cnn_header(caminho_out: Path, qcnn: QuantizedTiny1DCNNBio):
    """Exporta 100% dos 4.879 parametros dobrados (conv1+dw+pw+conv3+fc+class+bio) sem truncamento."""
    L = qcnn.raw_layers
    conteudo = f"""#pragma once
// Modelo Tiny1DCNNBio Completo com Folding BatchNorm -> Conv1d (4.879 parametros inteiros)
// GERADO AUTOMATICAMENTE POR python/aegis_ai/exportador_c.py — NAO EDITAR MANUALMENTE
#include <stdint.h>

#ifndef AEGIS_INPUT_DIM
#define AEGIS_INPUT_DIM 120
#endif
#ifndef AEGIS_NUM_CLASSES
#define AEGIS_NUM_CLASSES 7
#endif
#ifndef AEGIS_BIO_DIM
#define AEGIS_BIO_DIM 16
#endif

#define AEGIS_CNN_FOLDED_PARAMS 4879
#define AEGIS_S1_SHIFT  {L['s1']}
#define AEGIS_SDW_SHIFT {L['s_dw']}
#define AEGIS_SPW_SHIFT {L['s_pw']}
#define AEGIS_S3_SHIFT  {L['s3']}
#define AEGIS_SFC_SHIFT {L['s_fc']}

namespace aegis::cnn {{

alignas(16) static const int8_t CONV1_W[240] = {{ {_fmt_i8(L['conv1_w'])} }};
alignas(16) static const int32_t CONV1_B[16] = {{ {_fmt_i32(L['conv1_b'])} }};

alignas(16) static const int8_t DW_W[48] = {{ {_fmt_i8(L['dw_w'])} }};
alignas(16) static const int32_t DW_B[16] = {{ {_fmt_i32(L['dw_b'])} }};

alignas(16) static const int8_t PW_W[384] = {{ {_fmt_i8(L['pw_w'])} }};
alignas(16) static const int32_t PW_B[24] = {{ {_fmt_i32(L['pw_b'])} }};

alignas(16) static const int8_t CONV3_W[2304] = {{ {_fmt_i8(L['conv3_w'])} }};
alignas(16) static const int32_t CONV3_B[32] = {{ {_fmt_i32(L['conv3_b'])} }};

alignas(16) static const int8_t FC_W[1024] = {{ {_fmt_i8(L['fc_w'])} }};
alignas(16) static const int32_t FC_B[32] = {{ {_fmt_i32(L['fc_b'])} }};

alignas(16) static const int8_t CLS_W[224] = {{ {_fmt_i8(L['cls_w'])} }};
alignas(16) static const int32_t CLS_B[7] = {{ {_fmt_i32(L['cls_b'])} }};

alignas(16) static const int8_t BIO_W[512] = {{ {_fmt_i8(L['bio_w'])} }};
alignas(16) static const int32_t BIO_B[16] = {{ {_fmt_i32(L['bio_b'])} }};

}} // namespace aegis::cnn
"""
    caminho_out.write_text(conteudo, encoding="utf-8")


def gerar_ood_header(caminho_out: Path, ae: GestureOODAutoencoder, threshold_p98: float):
    """Exporta integralmente GestureOODAutoencoder (13.288 params) em float32 compacto."""
    sd = ae.state_dict()
    conteudo = f"""#pragma once
// GestureOODAutoencoder completo (120 -> 48 -> 16 -> 48 -> 120 = 13.288 parametros)
#include <stdint.h>

#define AEGIS_OOD_THRESHOLD {threshold_p98:.6f}f

namespace aegis::ood {{

static const float E1_W[48 * 120] = {{ {_fmt_f32(sd['encoder.0.weight'].cpu().numpy())} }};
static const float E1_B[48]       = {{ {_fmt_f32(sd['encoder.0.bias'].cpu().numpy())} }};
static const float E2_W[16 * 48]  = {{ {_fmt_f32(sd['encoder.2.weight'].cpu().numpy())} }};
static const float E2_B[16]       = {{ {_fmt_f32(sd['encoder.2.bias'].cpu().numpy())} }};
static const float D1_W[48 * 16]  = {{ {_fmt_f32(sd['decoder.0.weight'].cpu().numpy())} }};
static const float D1_B[48]       = {{ {_fmt_f32(sd['decoder.0.bias'].cpu().numpy())} }};
static const float D2_W[120 * 48] = {{ {_fmt_f32(sd['decoder.2.weight'].cpu().numpy())} }};
static const float D2_B[120]      = {{ {_fmt_f32(sd['decoder.2.bias'].cpu().numpy())} }};

}} // namespace aegis::ood
"""
    caminho_out.write_text(conteudo, encoding="utf-8")


def gerar_tamper_header(caminho_out: Path, net: TamperNetIMU):
    """Exporta TamperNetIMU completo (797 params: 12 -> 24 -> 16 -> 5)."""
    sd = net.state_dict()
    conteudo = f"""#pragma once
// TamperNetIMU completo (12 -> 24 -> 16 -> 5 = 797 parametros)
#include <stdint.h>

namespace aegis::tamper {{

constexpr float ACCEL_IMPACT_THRESHOLD_G = 2.5f;
constexpr float GYRO_IMPACT_THRESHOLD_DPS = 80.0f;

static const float W1[24 * 12] = {{ {_fmt_f32(sd['net.0.weight'].cpu().numpy())} }};
static const float B1[24]      = {{ {_fmt_f32(sd['net.0.bias'].cpu().numpy())} }};
static const float W2[16 * 24] = {{ {_fmt_f32(sd['net.2.weight'].cpu().numpy())} }};
static const float B2[16]      = {{ {_fmt_f32(sd['net.2.bias'].cpu().numpy())} }};
static const float W3[5 * 16]  = {{ {_fmt_f32(sd['net.4.weight'].cpu().numpy())} }};
static const float B3[5]       = {{ {_fmt_f32(sd['net.4.bias'].cpu().numpy())} }};

}} // namespace aegis::tamper
"""
    caminho_out.write_text(conteudo, encoding="utf-8")


def gerar_test_vectors_header(
    caminho_out: Path,
    vetores_entrada: List[np.ndarray],
    logits_esperados: List[np.ndarray],
):
    validar_identificador("AEGIS_TEST_VECTORS_H")
    n = len(vetores_entrada)
    in_dim = len(vetores_entrada[0])
    out_dim = len(logits_esperados[0])

    entradas_str = ["    {" + ", ".join(str(int(x)) for x in v) + "}" for v in vetores_entrada]
    logits_str = ["    {" + ", ".join(str(int(x)) for x in l) + "}" for l in logits_esperados]

    conteudo = f"""#pragma once
// Golden Vectors para POST real no boot do ESP32-S3 e paridade C++/Python

#include <stdint.h>

#define AEGIS_NUM_TEST_VECTORS {n}

namespace aegis::test {{

static const int8_t TEST_INPUTS[AEGIS_NUM_TEST_VECTORS][{in_dim}] = {{
{",\n".join(entradas_str)}
}};

static const int32_t EXPECTED_LOGITS[AEGIS_NUM_TEST_VECTORS][{out_dim}] = {{
{",\n".join(logits_str)}
}};

}} // namespace aegis::test
"""
    caminho_out.write_text(conteudo, encoding="utf-8")


def exportar_ecossistema_completo(
    ai_include_dir: Path,
    manifest_path: Path,
    model_cnn: Tiny1DCNNBio,
    dataset_data: np.ndarray,
    ood_ae: GestureOODAutoencoder,
    ood_thresh: float,
    tamper_net: TamperNetIMU,
) -> Dict[str, Any]:
    """Quantiza Tiny1DCNNBio, gera cabecalhos completos + 20 Golden Vectors + manifest SHA-256 real."""
    ai_include_dir.mkdir(parents=True, exist_ok=True)
    calib_t = torch.tensor(dataset_data, dtype=torch.float32)
    qcnn = QuantizedTiny1DCNNBio.from_float_model(model_cnn, calib_t, in_scale=S_E)

    gerar_model_cnn_header(ai_include_dir / "model_cnn.h", qcnn)
    # Atualiza model.h para incluir model_cnn.h e manter compatibilidade com includes antigos
    (ai_include_dir / "model.h").write_text(
        '#pragma once\n#include "model_cnn.h"\n', encoding="utf-8"
    )
    gerar_ood_header(ai_include_dir / "ood_model.h", ood_ae, ood_thresh)
    gerar_tamper_header(ai_include_dir / "tamper_model.h", tamper_net)

    # 20 Golden Vectors distribuidos uniformemente pelo dataset
    idxs = np.linspace(0, len(dataset_data) - 1, 20, dtype=int)
    vec_list: List[np.ndarray] = []
    logit_list: List[np.ndarray] = []
    for idx in idxs:
        v_i8 = np.clip(np.round(dataset_data[idx] / S_E), -127, 127).astype(np.int8)
        logits_i32, _ = qcnn.infer_int8(v_i8)
        vec_list.append(v_i8)
        logit_list.append(logits_i32)

    gerar_test_vectors_header(ai_include_dir / "test_vectors.h", vec_list, logit_list)

    # Hash SHA-256 real dos artefatos gerados
    h = hashlib.sha256()
    for fname in ("model_cnn.h", "ood_model.h", "tamper_model.h", "test_vectors.h"):
        h.update((ai_include_dir / fname).read_bytes())
    model_sha256 = h.hexdigest()

    counts = model_cnn.count_params()
    manifest_obj = {
        "version": FW_VERSION,
        "model_sha256": model_sha256,
        "firmware_sha256": hashlib.sha256((ai_include_dir / "model_cnn.h").read_bytes()).hexdigest(),
        "se_scale": S_E,
        "s1_shift": int(qcnn.raw_layers["s1"]),
        "shifts": {
            "s1": int(qcnn.raw_layers["s1"]),
            "s_dw": int(qcnn.raw_layers["s_dw"]),
            "s_pw": int(qcnn.raw_layers["s_pw"]),
            "s3": int(qcnn.raw_layers["s3"]),
            "s_fc": int(qcnn.raw_layers["s_fc"]),
        },
        "input_dim": 120,
        "classes": GESTURE_CLASSES,
        "num_params": counts["folded"],
        "num_params_unfolded": counts["unfolded"],
        "ood_params": sum(p.numel() for p in ood_ae.parameters()),
        "tamper_params": sum(p.numel() for p in tamper_net.parameters()),
        "offsets": {
            "bootloader": "0x0",
            "partitions": "0x8000",
            "boot_app0": "0xe000",
            "app": "0x10000",
        },
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest_obj, indent=2), encoding="utf-8")
    return {"qcnn": qcnn, "manifest": manifest_obj, "golden_inputs": vec_list, "golden_logits": logit_list}
