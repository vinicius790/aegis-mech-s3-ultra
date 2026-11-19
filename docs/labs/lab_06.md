# Lab 06 — Fusão Bayesiana (`EKF Forma de Joseph` + `ParticleFilterLite32`), `VL53L1X XSHUT` & Ground-Truth MediaPipe

- **Origens Integradas:** `robotica-computacional Mod 03 (laser.py, odom.py)` + `Mod 08`, `Introducao-a-robotica (MediaPipe)`, `IsaacLab`.

## Bug Auditado
- No `v2.0`, [kalman_particle.cpp](../../firmware/src/ai/kalman_particle.cpp) não aplicava ruído estocástico nem reamostragem sistemática em `ParticleFilterLite32`, colapsando pesos em menos de 8 passos; além disso, `KalmanFilter2D` não usava a **Forma de Joseph**.

## Roteiro Executivo de Bancada
1. Validar sequenciamento `XSHUT` (`0x29 -> 0x30 / 0x31`) no barramento I2C a `400 kHz`.
2. Posicionar anteparo refletor lateral para injetar eco *multipath* artificial nos `HC-SR04` e gravar sessão com [serial_recorder.py](../../demo/serial_recorder.py) + câmera rodando [mediapipe_teacher.py](../../python/aegis_ai/vision_2fa/mediapipe_teacher.py).
3. Visualizar rejeições de Mahalanobis ($d_M^2 > 6{,}63$) e nuvem das 32 partículas no [rerun_logger.py](../../demo/rerun_logger.py) e no Painel 1 de [app_web.py](../../demo/app_web.py).

## Quality Gate Numérico
- RMSE posicional EKF vs referência MediaPipe $< 6{,}0\,\text{mm}$; rejeição de ecos espúrios $> 95\%$; $N_{\text{eff}} \ge 16$ preservado após `systematic_resample()`.
