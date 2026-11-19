# Lab 02 — Proteção Indutiva ($-L\,di/dt$), Ponte-H `DRV8871`, Optoisolamento & Segurança em C++

- **Origens Integradas:** `3F2B4AAE`, `lab-uc Exp 16`, `logica-leds`, `Projeto_Display_de_Led_de_7S`, `placar-digital-wokwi`, `apostila_acnp Proj 01..11`.
- **Pré-Simulação 3D Web (*Smoke-Free*):**
  1. **[Learn Arduino 3D](https://learn-arduino-red.vercel.app/pt/):** Lições **`08 Faça um som`** (`Buzzer piezo`) e **`10 Controle um motor`** (`TIP120 + Motor DC`).
  2. **[Xplora3D](https://xplora.phinalabs.com/3d/#):** Montar circuito do **Experimento B** (`MOSFET IRLZ44N + Diode 1N5819 Schottky + DC Motor + Scope Probe P1` no Drain) conforme [simuladores_3d_web.md](../simuladores_3d_web.md). Observe no `Scope` que o diodo de roda livre grampeia o transiente de desligamento!

## Bugs Reais Auditados
1. `lab-uc Exp 16`: aciona motor DC com `BD137` **sem diodo flyback** → transiente $V = -L \frac{di}{dt}$ destrói o transistor.
2. `logica-leds/sketch.ino`: loops `for(i=0; i<=8; i++)` e `for(i=9; i>=0; i--)` num vetor `leds[8]` → *Out-of-Bounds Memory Access* em C++ (`-fsanitize=address,undefined`).

## Roteiro Executivo de Bancada
1. Conferir montagem `GPIO 16 -> 330 Ω -> PC817 (ISO1) -> Zener 12V + R_G 100 Ω -> IRLZ44N` e diodo **`1N5819` diretamente nos terminais da bobina `SOL1`**.
2. Medir corrente limite do `DRV8871` com `R_ILIM = 30,0 kΩ` ($I_{\text{trip}} = 64\,\text{kV}/30\,\text{k}\Omega = 2{,}133\,\text{A}$).
3. Auditar no [circuit_netlist_auditor.py](../../demo/circuit_netlist_auditor.py) caso sem `1N5819`, confirmando bloqueio por `RULE_NO_FLYBACK`.

## Quality Gate Numérico
- Overshoot no dreno de `Q1` $\le V_{\text{bus}} + 1{,}0\,\text{V}$; temperatura `DRV8871` $< 45\,^\circ\text{C}$.
- Zero erros em `[env:native]` (`-Wall -Wextra -Werror -fsanitize=address,undefined`).
