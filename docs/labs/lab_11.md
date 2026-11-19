# Lab 11 — Digital Twin Vetorizado (`aegis_sim/` Isaac Lab Style), `DriveBam`, `LEAPP` & Plataforma Stewart 6-DOF

- **Origens Integradas:** `IsaacLab-main.zip`, `Robotics-Projects/Stewart 6-DOF`, `awesome-robotics-projects (SO-ARM100)`.

## Roteiro Computacional & Bancada
1. Instanciar `VectorizedAegisEnv(num_envs=64)` em [python/aegis_ai/aegis_sim/](../../python/aegis_ai/aegis_sim/), verificando separação estrita `terminated` vs `time_out`.
2. Validar `VectorizedDualConeRayCaster` (`16 ToF + 13 US` raios) com injeção proposital de *crosstalk* (`ping_interval_ms = 10 < 30 ms`).
3. Ajustar `ActuatorNetMLP` / `ActuatorNetLSTM` em logs reais de [data/actuator_logs/actuator_logs.csv](../../data/actuator_logs/actuator_logs.csv) + modelo `DriveBamActuator` (*Voltage Sag + Stribeck + Backlash*).
4. Testar IK `6×6` Jacobian, *Adaptive-DLS* e **Cinemática Direta Newton-Raphson** em `StewartPlatform6DOFV3` (`kinematics.py`).
5. Executar `robust_eval_7_axes()` validando o controlador funcional puro `LEAPPRunner`.

## Quality Gate Numérico
- Taxa de sucesso $\ge 98\%$ nos **7 cenários extremos OOD** de `robust_eval_7_axes()`; convergência FK Stewart `< 0,01 mm`; Zero `NaN` sob singularidade 2-DOF.
