# Lab 07 — Controle Híbrido: `SCurveProfile` Quíntico, `Fuzzy-PID` (Medição-D), `Faddeev-LeVerrier` & `Line Follower`

- **Origens Integradas:** `6C8E4FC0` (*Soft-Starter vs Partida Direta*), `robotica-computacional Mod 07` (`Faddeev-LeVerrier + Durand-Kerner`), `Robotics-Projects/Line Follower/Code.ino`.
- **Pré-Simulação Web (`Painel 4` & `Xplora3D`):**
  1. Validar as 9 combinações Fuzzy-PID no [control_stability.py](../../python/aegis_ai/control_stability.py) e no Painel 4 (*Polos Z*) do `app_web.py`.
  2. Rodar o **Experimento B (Soft-Start PWM Quíntico)** no **[Xplora3D](https://xplora.phinalabs.com/3d/#)** observando a rampa gradual no `Scope`.

## Bugs Reais Auditados
1. `Robotics-Projects/Line Follower/Code.ino:162`: `kd * lasterror` (não calcula variação $\Delta e / \Delta t$).
2. `lock_controller.cpp` (`v2.0`): derivava sobre o erro `(e - e_prev)/dt` causando *derivative kick* no degrau `0 -> 28 mm` e não instanciava `SCurveProfile`.

## Roteiro Executivo de Bancada
1. Executar `auditar_tabela_fuzzy_3x3()` confirmando raio espectral $\rho < 1{,}0$ para todos os 9 pares `(kp, kd)`.
2. Gravar [04_motor_encoder_safe_sweep.cpp](../../firmware/tools/bringup/04_motor_encoder_safe_sweep.cpp) lendo `AS5600` (`0,17453 mm/deg`) + `INA219` (`Cal = 4096`).
3. Medir corrente de inrush comparando partida degrau vs rampa quíntica `SCurveProfile`, e validar corte por `MAX_AGE = 150 ms` desconectando o cabo I2C de teste durante o movimento.

## Quality Gate Numérico
- Redução de pico *inrush* $\ge 40\%$; erro estático $< 0{,}5\,\text{mm}$; $\rho(\mathbf{A}_{\text{malha}}) < 0{,}97 < 1{,}0$; parada `MAX_AGE` $\le 155\,\text{ms}$.
