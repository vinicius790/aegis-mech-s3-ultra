# Lab 08 — Pipeline TinyML Ponta-a-Ponta: Paridade `vx/vy`, `GroupKFold`, `SupCon` & Quantização QAT Bit-Exact

- **Origens Integradas:** `457B425B` (`@palis.dev`), [features.py](../../python/aegis_ai/features.py), [treino.py](../../python/aegis_ai/treino.py), [exportador_c.py](../../python/aegis_ai/exportador_c.py), [gesture_featurizer.cpp](../../firmware/src/ai/gesture_featurizer.cpp), [inference_int8.cpp](../../firmware/src/ai/inference_int8.cpp).

## Bugs Auditados Corrigidos
1. `gesture_featurizer.cpp` gravava `vx = vy = 0` e usava clamp assimétrico `-128`.
2. `treino.py` usava `loss_bio = 0.01 * sum(bio_emb**2)` com gradiente nulo sobre vetor L2 e salto geométrico em `"Z"`.
3. `exportador_c.py` exportava apenas MLP truncada de 32 elementos em vez dos **4.879 parâmetros dobrados (`5.023` pré-fold)** do `Tiny1DCNNBio`.

## Roteiro Executivo
1. Coletar ou simular 4 sessões com senha **`O - L - I`** por 3 perfis de usuário (`SyntheticGestureDataset` ou `serial_recorder.py`).
2. Rodar `treinar_modelo_tiny(..., run_group_kfold=True)`, busca multiobjetivo Pareto `run_optuna_study()` (`Flash <= 32KB, RAM <= 16KB`), `fold_conv1d_bn()`, exportação completa para `model_cnn.h + test_vectors.h` e Grad-CAM 1D em `xai.py`.
3. Confirmar `POST: 20/20 GOLDEN VECTORS PASS` com **`max |logits_Py - logits_Cpp| == 0`**.

## Quality Gate Numérico
- **`max |logits_Py - logits_Cpp| == 0`** nos `20/20` vetores dourados; acurácia `>= 97,5%` *out-of-session*; `FAR <= 1,0%`, `FRR <= 3,0%`; inferência `<= 1,0 ms @ 240 MHz`.
