# Lab 04 — Instrumentação Termo-Higrométrica (`DS18B20`, `Magnus-Tetens`, `NTC`) & Massa/Posição (`HX711`, `KY-024`)

- **Origens Integradas:** `lab-uc Exp 02, 03, 04, 05, 14, 15`, `sensor_de_umidade_do_ar_caseiro` (`libraries.rar`), `apostila_robotica_acnp Proj 12`.
- **Pré-Simulação 3D Web (*Smoke-Free*):**
  1. **[Learn Arduino 3D](https://learn-arduino-red.vercel.app/pt/):** Lições **`11 Sinta o calor`** (`TMP36`) e **`15 Sinta a força`** (`FSR 402`).
  2. **[Xplora3D](https://xplora.phinalabs.com/3d/#):** Montar `Thermistor (NTC 10k B3950)` em divisor simétrico com `Resistor 10 kΩ` ligado ao canal ADC `A0` e testar com props **`Ice`** e **`Flame`** no menu `Environment`.

## Bugs Reais Auditados
1. `sensor_de_umidade_do_ar_caseiro.ino`: L132 compara `>= table[0][line] && < table[0][line]` (sempre falso!), L036 traz typo `92, 83, 93` (em vez de `92`) e usa `delay(1000)` síncrono.
2. `lab-uc Exp 05`: `SCK` do `HX711` ligado em `GPIO 35` (*input-only* no ESP32 clássico!) e `VDD = 5 V`.
3. `lab-uc Exp 02`: divisor com resistor de `100 Ω` no NTC de `10 kΩ`.

## Roteiro Executivo de Bancada
1. Testar equação contínua em [psychrometry.py](../../python/aegis_ai/psychrometry.py) validando $c(T, RH)$ contra fórmula seca $c(T)$.
2. Rodar `DS18B20` com `setWaitForConversion(false)` (`1000 ms` assíncrono).
3. Testar `HX711` alimentado em `3,3 V` com `SCK` em `GPIO 47` e calibrar sensor Hall/óptico `KY-024`/`TCRT5000` nos fins de curso.

## Quality Gate Numérico
- Desvio em $c(T, RH) \le 0{,}2\%$; erro de pesagem `HX711` $\le 1{,}5\,\text{g}$; tempo bloqueado CPU `< 100 µs/tick`.
