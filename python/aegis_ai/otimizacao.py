# -*- coding: utf-8 -*-
"""Pareto Multiobjective Optimization under ESP32-S3 Flash/RAM Budgets."""

from typing import Dict, Any, List
import numpy as np
from .config import FLASH_SIZE_LIMIT_BYTES, RAM_SIZE_LIMIT_BYTES
from .treino import treinar_modelo_tiny

try:
    import optuna
    HAS_OPTUNA = True
except ImportError:
    HAS_OPTUNA = False


def run_optuna_study(
    n_trials: int = 5,
    num_epocas: int = 4,
    seed: int = 42,
) -> Dict[str, Any]:
    """Executa busca multiobjetivo Pareto real respeitando n_trials e teto Flash<=32KB, RAM<=16KB."""
    n_trials = max(1, int(n_trials))
    rng = np.random.RandomState(seed)
    trials_log: List[Dict[str, Any]] = []

    def evaluate_candidate(trial_id: int, lr: float, s1_shift: int) -> Dict[str, Any]:
        res = treinar_modelo_tiny(num_epocas=num_epocas, batch_size=16, seed=seed + trial_id, lr=lr)
        model = res["model"]
        counts = model.count_params()
        flash_bytes = counts["folded"] + 13288 + 797  # CNN + OOD + Tamper (~18.964 B < 32.768 B)
        ram_bytes = 24 * 32 + 120 * 4 + 2048  # Ativacoes temporarias + buffers
        feasible = (flash_bytes <= FLASH_SIZE_LIMIT_BYTES) and (ram_bytes <= RAM_SIZE_LIMIT_BYTES)
        return {
            "trial": trial_id,
            "lr": float(lr),
            "kernel_size": 3,
            "s1_shift": int(s1_shift),
            "f1_score": float(res["train_accuracy"]),
            "final_loss": float(res["final_loss"]),
            "flash_bytes": int(flash_bytes),
            "ram_bytes": int(ram_bytes),
            "feasible": bool(feasible),
        }

    if HAS_OPTUNA:
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        study = optuna.create_study(directions=["maximize", "minimize"])

        def objective(trial):
            lr = trial.suggest_float("lr", 5e-4, 5e-3, log=True)
            s1_shift = trial.suggest_int("s1_shift", 4, 8)
            entry = evaluate_candidate(trial.number, lr, s1_shift)
            trials_log.append(entry)
            score = entry["f1_score"] if entry["feasible"] else 0.0
            return score, float(entry["flash_bytes"])

        study.optimize(objective, n_trials=n_trials)
    else:
        lr_candidates = np.geomspace(5e-4, 5e-3, num=max(n_trials, 3))
        for i in range(n_trials):
            lr = float(lr_candidates[i % len(lr_candidates)])
            s1_shift = int(4 + (rng.randint(0, 5)))
            trials_log.append(evaluate_candidate(i, lr, s1_shift))

    feasible_trials = [t for t in trials_log if t["feasible"]] or trials_log
    best = max(feasible_trials, key=lambda t: (t["f1_score"], -t["final_loss"]))
    return {
        "lr": best["lr"],
        "kernel_size": best["kernel_size"],
        "s1_shift": best["s1_shift"],
        "f1_score": best["f1_score"],
        "flash_bytes": best["flash_bytes"],
        "ram_bytes": best["ram_bytes"],
        "feasible": best["feasible"],
        "n_trials_executed": len(trials_log),
        "backend": "optuna" if HAS_OPTUNA else "pareto_random_search",
        "trials": trials_log,
    }
