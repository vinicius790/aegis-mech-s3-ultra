# Lab 01 — Eletrônica Segura, SMPS/Buck `MP1584EN`, DCDB/ATS & Mapa Proibido `ESP32-S3-N16R8`

- **Origens Integradas:** `1F8025B2`, `8FA42342`, `DD0F53E6`, `C0A42D33`, `7DEAC40B`, `80BA0046`, `099C464C`, `Introducao-a-robotica`, `contador-wokwi`, `varredura-wokwi`, `lab-uc Exp 01/02/06`, `apostila_acnp`.
- **Pré-Simulação 3D Web (*Smoke-Free*):**
  1. **[Learn Arduino 3D](https://learn-arduino-red.vercel.app/pt/):** Executar Lições **`01 Conheça a placa`**, **`02 Pisque um LED`** e **`03 Aperte um botão`** ativando *Mostrar fluxo de corrente* e *Inspetor de peças* (`220 Ω` vs `10 kΩ`).
  2. **[Xplora3D](https://xplora.phinalabs.com/3d/#):** Montar na protoboard `LM7805` + `AMS1117-3.3` + divisores resistivos com **`Multimeter`** e conferir no menu *Inspect wiring (`I`)* ausência de curto antes de `Run Sim`. *(Nota: validar `MP1584EN` chaveado com `C_BST=100 nF` em esquemático KiCad e bancada física.)*

## Bugs Reais Auditados
1. `contador-wokwi/diagram.json`: `"id": "r2", "attrs": { "value": "0.22" }` → queima segmentos LED (`< 100 Ω`). **Correção:** `220 Ω` (`~13 mA`).
2. `varredura-wokwi/sketch.ino`: `GPIO 12 (MTDI strapping)` + drenagem de 7 LEDs num pino só. **Correção:** Remover `GPIO 12` e usar driver de corrente.
3. `firmware/platformio.ini` antigo: `flash_mode = opi` num módulo `N16R8` → *boot loop*. **Correção:** `flash_mode = qio` + `memory_type = qio_opi`.

## Roteiro Executivo de Bancada & Bring-Up
1. Rodar auditoria estática antes de ligar a fonte:
   `py demo/circuit_netlist_auditor.py --netlist hardware/kicad/aegis_mech_s3_netlist.json --strict`
2. Alimentar placa com fonte isolada `12 V / 5 A` via fusível `F1 (5 A)` + TVS `SMBJ15A`; medir no osciloscópio (`AC-coupled`, `20 MHz BW`) os barramentos `+5V_BUCK` e `+3V3_SYS`.
3. Acionar a chave cogumelo **E-Stop NF** verificando corte físico imediato em `+12V_VM_SW` enquanto o ESP32-S3 segue ativo lendo `GPIO 42 == HIGH` (`E08_HARDWARE_ESTOP_ACTIVE`).
4. Gravar e rodar [01_i2c_scanner_voltage_check.cpp](../../firmware/tools/bringup/01_i2c_scanner_voltage_check.cpp).

## Quality Gate Numérico
- $V_{5\text{V}} \in [4{,}90; 5{,}10]\,\text{V}$, ripple $\le 50\,\text{mV}_{pp}$; $V_{3{,}3\text{V}} \in [3{,}23; 3{,}37]\,\text{V}$.
- `100%` ACK I2C em `{0x30, 0x31, 0x36, 0x3C, 0x40, 0x68}`.
