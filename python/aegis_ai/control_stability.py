# -*- coding: utf-8 -*-
"""Discrete Closed-Loop Control Stability Verifier (Faddeev-LeVerrier + Durand-Kerner).

Portado integralmente da engine espectral de robotica-computacional/07-controle
com derivada exclusivamente na medicao (elimina derivative kick).
"""

import cmath
import math
from typing import Dict, Any, List


def matriz_malha(
    kp: float,
    ki: float,
    kd: float,
    h: float = 0.01,
    tau: float = 0.08,
    k_plant: float = 0.045,
) -> List[List[float]]:
    """Constroi matriz aumentada 3x3 em malha fechada com derivada estritamente na medicao.

    Estado: [y_k, y_{k-1}, I_{k-1}], Planta 1a ordem discretizada (ZOH): y_{k+1} = a*y_k + c*u_k,
    onde c = k_plant * tau * (1 - a) (ganho fisico DC permille -> mm da cremalheira M1.25).
    """
    a = math.exp(-h / max(tau, 1e-6))
    c = k_plant * tau * (1.0 - a)
    g_yk = -(kp + ki * h + kd / h)
    g_yp = kd / h
    g_ik = ki
    return [
        [a + c * g_yk, c * g_yp, c * g_ik],
        [1.0,          0.0,      0.0     ],
        [-h,           0.0,      1.0     ],
    ]


def faddeev_leverrier(A: List[List[float]]) -> List[float]:
    """Calcula coeficientes [1, p1, ..., pn] de det(lambda*I - A) = 0."""
    n = len(A)

    def matmul(x: List[List[float]], y: List[List[float]]) -> List[List[float]]:
        return [[sum(x[i][k] * y[k][j] for k in range(n)) for j in range(n)] for i in range(n)]

    m_mat = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    coefs = [1.0]
    p = 0.0
    a_m = m_mat
    for k in range(1, n + 1):
        if k > 1:
            m_mat = [[a_m[i][j] + (p if i == j else 0.0) for j in range(n)] for i in range(n)]
        a_m = matmul(A, m_mat)
        tr = sum(a_m[i][i] for i in range(n))
        p = -tr / k
        coefs.append(p)
    return coefs


def durand_kerner(coefs: List[float], max_iter: int = 80) -> List[complex]:
    """Encontra todas as raizes complexas (polos z) de z^n + p1*z^{n-1} + ... + pn = 0."""
    n = len(coefs) - 1
    if n <= 0:
        return []

    def eval_poly(z: complex) -> complex:
        val = complex(coefs[0], 0.0)
        for c in coefs[1:]:
            val = val * z + c
        return val

    roots = [cmath.exp(2j * math.pi * k / n) * (0.4 + 0.05 * k) for k in range(n)]
    for _ in range(max_iter):
        next_roots = list(roots)
        for i in range(n):
            denom = complex(1.0, 0.0)
            for j in range(n):
                if i != j:
                    diff = next_roots[i] - next_roots[j]
                    denom *= diff if abs(diff) > 1e-12 else complex(1e-12, 1e-12)
            next_roots[i] = next_roots[i] - eval_poly(next_roots[i]) / denom
        roots = next_roots
    return roots


def avaliar_estabilidade_pid(
    kp: float,
    ki: float,
    kd: float,
    h: float = 0.01,
    tau: float = 0.08,
    k_plant: float = 0.045,
) -> Dict[str, Any]:
    """Retorna polos, raio espectral e regime dinamico no plano-z."""
    a_mat = matriz_malha(kp, ki, kd, h=h, tau=tau, k_plant=k_plant)
    coefs = faddeev_leverrier(a_mat)
    poles = durand_kerner(coefs)
    rho = max((abs(z) for z in poles), default=0.0)
    if rho >= 1.0:
        regime = "instavel"
    elif rho >= 0.96:
        regime = "limite"
    elif any(abs(z.imag) > 1e-3 for z in poles):
        regime = "subamortecido_estavel"
    else:
        regime = "estavel"
    return {
        "matrix": a_mat,
        "char_poly": coefs,
        "poles": poles,
        "spectral_radius": float(rho),
        "is_stable": bool(rho < 1.0),
        "regime": regime,
    }


def auditar_tabela_fuzzy_3x3(h: float = 0.01, tau: float = 0.08) -> Dict[str, Any]:
    """Verifica todos os 9 pontos da matriz Fuzzy-PID do LockController v3.0."""
    kp_set = [30.0, 45.0, 65.0]
    kd_set = [0.8, 1.2, 2.4]
    ki = 2.0
    results = []
    all_stable = True
    max_rho = 0.0
    for kp in kp_set:
        for kd in kd_set:
            res = avaliar_estabilidade_pid(kp, ki, kd, h=h, tau=tau)
            results.append({"kp": kp, "ki": ki, "kd": kd, **res})
            all_stable = all_stable and res["is_stable"]
            max_rho = max(max_rho, res["spectral_radius"])
    return {"all_stable": all_stable, "max_spectral_radius": max_rho, "points": results}
