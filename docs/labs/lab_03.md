# Lab 03 — Sensoriamento Ultrassônico (`HC-SR04`) sem Diafonia, Divisores & `<driver/rmt.h>` Não-Bloqueante

- **Origens Integradas:** `457B425B`, `lab-uc Exp 12` + datasheets `ElecFreaks/ITead`, `reguaeletronica-wokwi`, `sistema-de-entrada`, `Robotics-Projects/Ultrasonic Test.ino`.
- **Pré-Simulação 3D Web (*Smoke-Free*):**
  1. **[Learn Arduino 3D](https://learn-arduino-red.vercel.app/pt/):** Lições **`07 Leia um sensor`** e **`16 Monte um projeto`** (`HC-SR04`).
  2. **[Xplora3D](https://xplora.phinalabs.com/3d/#):** Montar `Ultrasonic HC-SR04` + divisor **`1,2 kΩ / 2,2 kΩ`** + script MicroPython **Experimento A** de [simuladores_3d_web.md](../simuladores_3d_web.md), usando *Environment → Prop `Target`* e `Scope P1` no pino `ECHO` pós-divisor.

## Bugs Reais Auditados
1. `lab-uc Exp 12`: pull-up `10 kΩ -> 3,3 V` ligado no pino `ECHO` (`5 V`). **Correção:** Divisor `1,2 kΩ / 2,2 kΩ` ($3{,}235\,\text{V}$ máx).
2. `reguaeletronica-wokwi`: array `numeros[10]` omite dígito `6` (`B01111101`) e `pulseIn()` sem timeout.
3. `sistema-de-entrada`: pino único `TRIG=ECHO` + ausência de intervalo mínimo entre sensores (diafonia acústica!).

## Roteiro Executivo de Bancada
1. Conferir tensão de pico nos pinos `GPIO 5` e `GPIO 7` com osciloscópio ($\le 3{,}30\,\text{V}$).
2. Gravar [02_sensors_us_tof_thermal.cpp](../../firmware/tools/bringup/02_sensors_us_tof_thermal.cpp) e posicionar anteparo plano em **`200,0 mm`** dentro de [arena_base.scad](../../cad/arena_base.scad).
3. Verificar escalonamento Ping-Pong `60 ms/eixo` (`30 ms` alternado X↔Y) e retorno `NaN` para ecos $\ge 38{,}0\,\text{ms}$.

## Quality Gate Numérico
- Tensão máxima em `GPIO 5/7` $\le 3{,}30\,\text{V}$; diafonia $< 0{,}5\%$; erro estático médio $< 2{,}5\,\text{mm}$ @ `200 mm`; bloqueio CPU `= 0 µs`.
