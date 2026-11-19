# Lab 05 — Engenharia de Chicotes (`Chicote 03`), Star-Ground (`NT1`) & Refatoração FSM `O(1)`

- **Origens Integradas:** `1108A3A5` (*Chicote de Pista · 03*), `BA21951D` (*@wiring.studio*), `Semafaro_Arduino_cpp`, `Maquina-de-Cafe`, `sistema-de-entrada`, `robotica-computacional Mod 01/03`.
- **Documentação Base:** [hardware/harness_wireviz.yml](../../hardware/harness_wireviz.yml) e Painel 3 (*Rastreador de Chicote SVG*) de [demo/app_web.py](../../demo/app_web.py).

## Bugs Reais Auditados
1. `Semafaro_Codigo_Arduino_cpp.cpp`: uso de `delay(2000)` bloqueando resposta imediata.
2. `Maquina-de-Cafe-Automatica`: `while(true)` trava botão `#` de cancelamento.
3. `sistema-de-entrada/versao-1.ino:74-82`: incrementa contador checando **apenas `primeiro`** e ignorando `segundo`.

## Roteiro Executivo de Bancada
1. Confeccionar o chicote `W_ARENA_BUS` (`22 AWG` tronco / `26 AWG` pontas) com **zero emendas em vão livre**, emendas **escalonadas $\ge 30\,\text{mm}$** atrás do conector `JST-XH 10P` e termo-retrátil dupla parede com cola.
2. Confirmar continuidade separada entre `AGND` (sensores + `R2 = 2,2 kΩ`) e `PGND` (`DRV8871` + `SOL1`) unindo somente em **`NT1` (`C_BULK1`)**.
3. Medir ruído pico-a-pico no pino `GPIO 1` (`ADC1`) com motor `M1` ativo usando [security_fsm.cpp](../../firmware/src/mechatronics/security_fsm.cpp) (`arena_dwell_ms >= 120 ms`).

## Quality Gate Numérico
- Queda de ruído ADC sob carga do motor $\ge 50\%$; jitter loop `Task_Comms_HMI_FSM` `< 1 ms @ 30 Hz`; **zero `delay()`**.
