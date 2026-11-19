# Gemeo Digital 3D Multi-Protoboard — AEGIS-MECH S3 v3.0 Ultra (`Xplora3D`)

Este diretorio contem o circuito 3D interativo completo (`version 2.1`) organizado em **5 Protoboards Acopladas (1.110 furos em 7 Zonas Funcionais)** e o firmware MicroPython embarcado do **AEGIS-MECH S3 v3.0 Ultra** (integrando os **20 repositorios `.zip`**) para execucao imediata no **Xplora3D** (`https://xplora.phinalabs.com/3d/#`).

## Estatisticas Auditadas
- **Protoboards Acopladas**: `5` (`Main 400 pts` + `Extra 400 pts bb1:` + `3x Mini Protoboards 170 pts mn1:/mn2:/mn3:`)
- **Componentes 3D Montados**: `40` distribuidos em 7 zonas funcionais com zero sobreposicao
- **Jumpers Roteados (`Pre-Cut` Alinhados Verticalmente)**: `99` (0 curtos-circuitos, 0 colisoes de furo)
- **Portas GPIO/ADC/PWM/I2C Ativas**: `24/24`
- **Auditoria Eletrica**: `PASSED = True`

## Matriz Multi-Microcontrolador Integrada no Projeto (`Arduinos, ESP32, ESP32-S3, ESP32-C3, ESP8266, STM32`)

| Familia / Arquitetura | Board ID | Papel no Ecossistema AEGIS-MECH S3 v3.0 | Firmware / Circuito |
| :--- | :--- | :--- | :--- |
| **ESP32-S3 (Dual-Core Xtensa LX7 + Vector PIE)** | `board-esp32-s3-devkitc-1 / UNDA.Xplora UI` | Controlador Principal TinyML INT8 + Biometria Cosseno 16-D + BLE HID + ESP-NOW | `firmware/src/main.cpp & firmware/arduino_aegis_s3_all_in_one/arduino_aegis_s3_all_in_one.ino` |
| **STM32F411CE (ARM Cortex-M4F 100 MHz)** | `UNDA.Xplora 32-Pin Header (A0..A15 / B0..B15)` | Co-Processador de Tempo Real da Bancada 3D Xplora3D (24 GPIOs + I2C + PWM S-Curve + 74HC595) | `docs/xplora3d/aegis_mech_s3_v3.x3d (MicroPython Engine)` |
| **ESP32-C3 (RISC-V Single-Core 160 MHz BLE 5.0)** | `board-esp32-c3-devkitm-1` | Varinha Inercial Vestivel BLE HID Air Mouse + Calibracao Giroscopica 500 Amostras (Z05) | `firmware/tools/comet_air_mouse_wand_c3/comet_air_mouse_wand_c3.ino` |
| **ESP32 Classic (Dual-Core Xtensa LX6 Wi-Fi/BT)** | `board-esp32-devkit-c-v4` | Gateway de Telemetria Ultrasonica HC-SR04 + Termistor NTC 10k + Ponte Serial (Z04/Z15/Z16) | `docs/wokwi/z15_regua_eletronica_hcsr04/sketch.ino` |
| **ESP8266 (NodeMCU ESP-12E Tensilica L106 Wi-Fi)** | `board-nodemcu-esp8266` | No IoT de Psicrometria Bulbo Seco/Umido Magnus-Tetens & Logica de 8 LEDs (Z10/Z18) | `docs/wokwi/z18_logica_leds_8x/sketch.ino` |
| **Arduino Uno R3 (ATmega328P 16 MHz)** | `wokwi-arduino-uno` | Controlador Didatico 74HC595 + 7-Seg POV + Semaforo FSM + 16 Licoes Learn-Arduino (Z13/Z14/Z17..Z20) | `docs/wokwi/z13_placar_digital_74hc595/sketch.ino` |
| **Arduino Mega 2560 (ATmega2560 16 MHz 54 GPIOs)** | `wokwi-arduino-mega` | Controlador Robotico 5x Sensores IR TCRT5000 PID + Portal Bidirecional Duplo Sonar (Z03/Z12/Z16) | `docs/wokwi/z16_sistema_entrada_bidirecional/sketch.ino` |

## Arquivos Gerados
- [`aegis_mech_s3_v3.x3d`](./aegis_mech_s3_v3.x3d) — Arquivo nativo para `File -> Open...` (`Ctrl+O`) no Xplora3D.
- [`aegis_mech_s3_v3_share_url.txt`](./aegis_mech_s3_v3_share_url.txt) — Link completo `#p=` autocontido (5 protoboards + 40 componentes + 99 fios + codigo MicroPython + notas).
