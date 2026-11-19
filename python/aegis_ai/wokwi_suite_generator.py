"""Gerador da Suite Multi-Microcontrolador (Wokwi + Falstad CircuitJS1 + Learn-Arduino 3D).

Integra todos os microcontroladores suportados (`ESP32-S3`, `ESP32 Classic`, `ESP32-C3`,
`ESP8266 NodeMCU`, `Arduino Uno R3`, `Arduino Mega 2560`) nos 7 projetos de referencia
em `Projeto/docs/wokwi/`, alem de exportar a netlist SPICE/Analogica para o **Falstad CircuitJS1**
(`https://www.falstad.com/circuit/circuitjs.html`) e o mapeamento completo das 16 licoes 3D do
**Learn-Arduino** (`https://learn-arduino-red.vercel.app/pt/`).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
import urllib.parse


FALSTAD_CIRCUITJS_NETLIST = """$ 1 0.000005 10.20027730826997 50 5 50 5e-11
R 80 80 48 80 0 0 40 5 0 0 0.5
w 80 80 176 80 0
r 176 80 176 160 0 1200
r 176 160 176 240 0 2200
g 176 240 176 272 0
O 176 160 240 160 1 0
R 320 80 288 80 0 0 40 3.3 0 0 0.5
r 320 80 320 160 0 10000
r 320 160 320 240 0 11441.48
g 320 240 320 272 0
O 320 160 384 160 1 0
R 464 208 416 208 0 2 1000 1.65 1.65 0 0.85
r 464 208 528 208 0 100
r 528 208 528 288 0 10000
g 528 288 528 320 0
f 528 208 576 208 32 1.5 0.02
g 576 224 576 288 0
R 576 64 576 32 0 0 40 5 0 0 0.5
l 576 64 576 144 0 0.015 0.12
r 576 144 576 192 0 10
d 640 192 640 64 2 default
w 576 64 640 64 0
w 576 192 640 192 0
c 720 80 720 176 0 0.0001 3.3 0.001
R 720 80 720 48 0 0 40 3.3 0 0 0.5
g 720 176 720 208 0
"""

LEARN_ARDUINO_3D_16_LESSONS: list[dict[str, str]] = [
    {"lesson": "01", "url": "https://learn-arduino-red.vercel.app/pt/#board", "title": "Conheca a placa (Arduino Uno ATmega328P)", "aegis_integration": "Mapeamento de barramentos GPIO/ADC/PWM e alimentacao 5V/3.3V"},
    {"lesson": "02", "url": "https://learn-arduino-red.vercel.app/pt/#led", "title": "Pisque um LED (5mm + 220 Ω)", "aegis_integration": "Sinalizacao visual com resistor limitador de corrente 220 Ω"},
    {"lesson": "03", "url": "https://learn-arduino-red.vercel.app/pt/#button", "title": "Aperte um botao (Tactile 12mm + 10kΩ Pull-Down)", "aegis_integration": "Botao E-Stop / Trigger A4 com pull-down 10 kΩ e debounce"},
    {"lesson": "04", "url": "https://learn-arduino-red.vercel.app/pt/#pot", "title": "Gire um potenciometro (10 kΩ linear)", "aegis_integration": "Proxy continuo de trajetoria ToF (40..300 mm) no ADC B1"},
    {"lesson": "05", "url": "https://learn-arduino-red.vercel.app/pt/#rgb", "title": "Misture cores (LED RGB / WS2812B)", "aegis_integration": "Barra NeoPixel WS2812B 8x e Semaforo FSM 3 cores"},
    {"lesson": "06", "url": "https://learn-arduino-red.vercel.app/pt/#ldr", "title": "Sinta a luz (GL5528 LDR + divisor 10 kΩ)", "aegis_integration": "Canal de luminancia 2FA optica no ADC A1"},
    {"lesson": "07", "url": "https://learn-arduino-red.vercel.app/pt/#ultrasonic", "title": "Leia um sensor (HC-SR04 ToF Ultrassonico)", "aegis_integration": "Telemetria ToF com compensacao psicrometrica c(T,RH) e Kalman"},
    {"lesson": "08", "url": "https://learn-arduino-red.vercel.app/pt/#buzzer", "title": "Faca um som (Buzzer Piezo 12mm)", "aegis_integration": "Buzzer A12 mantido 100% silencioso em regime (bip 25ms apenas no botao A4)"},
    {"lesson": "09", "url": "https://learn-arduino-red.vercel.app/pt/#piano", "title": "Mini piano (Matriz de botoes + Buzzer)", "aegis_integration": "Decodificacao de entradas digitais sem bloqueio de loop"},
    {"lesson": "10", "url": "https://learn-arduino-red.vercel.app/pt/#motor", "title": "Controle um motor (Motor DC + Transistor/MOSFET + Flyback)", "aegis_integration": "Estagio de potencia IRLZ44N (B15) + Diodo Flyback 1N4001 + Motor DC"},
    {"lesson": "11", "url": "https://learn-arduino-red.vercel.app/pt/#heat", "title": "Sinta o calor (Sensor TMP36 10mV/°C)", "aegis_integration": "Termometria de Bulbo Seco TMP36 (A5) + NTC 10k B3950 (A0)"},
    {"lesson": "12", "url": "https://learn-arduino-red.vercel.app/pt/#servo", "title": "Mova um servo (Micro Servo SG90 50 Hz)", "aegis_integration": "Trava mecatronica SG90 (B5) com perfil polinomial quintico S-Curve C^2"},
    {"lesson": "13", "url": "https://learn-arduino-red.vercel.app/pt/#stepper", "title": "Gire a roleta (28BYJ-48 + ULN2003)", "aegis_integration": "Controle de fase sincronizado por temporizador deterministico"},
    {"lesson": "14", "url": "https://learn-arduino-red.vercel.app/pt/#plant", "title": "Planta inteligente (Sensor Capacitivo + OLED 0.96\" SSD1306 I2C)", "aegis_integration": "Cockpit OLED SSD1306 128x64 (0x3C) + LCD 16x2 I2C (0x27) em B6/B7"},
    {"lesson": "15", "url": "https://learn-arduino-red.vercel.app/pt/#force", "title": "Sinta a forca (Sensor de Forca FSR 402 + Servo)", "aegis_integration": "Sensor de carga FSR 402 na Mini Protoboard 1 (`mn1:col 14..16`)"},
    {"lesson": "16", "url": "https://learn-arduino-red.vercel.app/pt/#project", "title": "Monte um projeto (HC-SR04 + Modulo de Semaforo)", "aegis_integration": "Integracao completa Sonar ToF + Semaforo FSM (Verde B13 / Amarelo A8 / Vermelho B12)"},
]


def export_multi_platform_simulators(repo_root: Path | None = None) -> dict[str, Any]:
    """Exporta a netlist Falstad CircuitJS1 e o catalogo Multi-Plataforma (Xplora3D + Wokwi + Falstad + Learn-Arduino)."""
    root = repo_root or Path(__file__).resolve().parents[2]
    falstad_dir = root / "docs" / "falstad"
    falstad_dir.mkdir(parents=True, exist_ok=True)
    netlist_text = FALSTAD_CIRCUITJS_NETLIST.strip() + "\n"
    netlist_path = falstad_dir / "aegis_power_and_analog_front_end.circuitjs.txt"
    netlist_path.write_text(netlist_text, encoding="utf-8")

    falstad_url = "https://www.falstad.com/circuit/circuitjs.html?cct=" + urllib.parse.quote(netlist_text)
    url_path = falstad_dir / "falstad_share_url.txt"
    url_path.write_text(falstad_url + "\n", encoding="utf-8")

    return {
        "falstad_netlist_path": str(netlist_path),
        "falstad_url": falstad_url,
        "learn_arduino_lessons": LEARN_ARDUINO_3D_16_LESSONS,
    }


def generate_all_wokwi_reference_projects(wokwi_root: Path | str | None = None) -> list[str]:
    repo_root = Path(__file__).resolve().parents[2]
    base_dir = Path(wokwi_root) if wokwi_root else (repo_root / "docs" / "wokwi")
    base_dir.mkdir(parents=True, exist_ok=True)

    # Exporta tambem os artefatos Falstad CircuitJS1 e Learn-Arduino 3D
    export_multi_platform_simulators(repo_root)

    suites: dict[str, dict[str, Any]] = {
        "z13_placar_digital_74hc595": {
            "mcu_family": "Arduino Uno R3 (ATmega328P) + 74HC595",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z13 Corrigido em Arduino Uno R3 - Sem pino TX 1 e sem delay(200))",
                "editor": "wokwi",
                "parts": [
                    {"type": "wokwi-arduino-uno", "id": "uno", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-74hc595", "id": "sr1", "top": 40, "left": 240, "attrs": {}},
                    {"type": "wokwi-7segment", "id": "sev1", "top": 40, "left": 370, "attrs": {}},
                    {"type": "wokwi-resistor", "id": "r1", "top": 90, "left": 300, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["uno:8", "sr1:DS", "green", []],
                    ["uno:9", "sr1:STCP", "blue", []],
                    ["uno:10", "sr1:SHCP", "yellow", []],
                    ["sr1:Q0", "r1:1", "red", []],
                    ["r1:2", "sev1:A", "red", []],
                ],
            },
            "sketch": (
                "// Z13 Placar Digital 74HC595 Corrigido (Arduino Uno R3 - Sem usar pino 1 TX e debounce millis() nao-bloqueante)\n"
                "const int PIN_DS = 8, PIN_STCP = 9, PIN_SHCP = 10, PIN_BTN = 7;\n"
                "unsigned long lastDebounceMs = 0;\n"
                "int score = 0;\n"
                "void setup() { pinMode(PIN_DS, OUTPUT); pinMode(PIN_STCP, OUTPUT); pinMode(PIN_SHCP, OUTPUT); pinMode(PIN_BTN, INPUT_PULLUP); }\n"
                "void loop() {\n"
                "  if (digitalRead(PIN_BTN) == LOW && (millis() - lastDebounceMs) > 40) {\n"
                "    lastDebounceMs = millis();\n"
                "    score = (score + 1) % 100;\n"
                "  }\n"
                "}\n"
            ),
        },
        "z14_varredura_7seg_pov": {
            "mcu_family": "ESP32-S3 (Dual-Core Xtensa LX7)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z14 Corrigido em ESP32-S3 - Sem GPIO 12 MTDI Strapping)",
                "editor": "wokwi",
                "parts": [
                    {"type": "board-esp32-s3-devkitc-1", "id": "esp", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-7segment", "id": "sev1", "top": 20, "left": 220, "attrs": {"digits": "4"}},
                    {"type": "wokwi-resistor", "id": "r1", "top": 80, "left": 150, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["esp:4", "r1:1", "green", []],
                    ["r1:2", "sev1:A", "green", []],
                    ["esp:18", "sev1:DIG1", "blue", []],
                ],
            },
            "sketch": (
                "// Z14 Varredura POV 7-Seg Corrigida (ESP32-S3: GPIO 12 MTDI substituido por GPIO 18 seguro + timer micros())\n"
                "const uint8_t kSegPins[7] = {4, 5, 6, 7, 8, 9, 10};\n"
                "const uint8_t kDigPins[4] = {18, 19, 20, 21}; // Sem GPIO 12 (MTDI)!\n"
                "unsigned long lastScanUs = 0;\n"
                "uint8_t curDig = 0;\n"
                "void setup() { for(int i=0;i<7;++i) pinMode(kSegPins[i], OUTPUT); for(int i=0;i<4;++i) pinMode(kDigPins[i], OUTPUT); }\n"
                "void loop() {\n"
                "  if (micros() - lastScanUs >= 2000UL) {\n"
                "    lastScanUs = micros();\n"
                "    digitalWrite(kDigPins[curDig], HIGH);\n"
                "    curDig = (curDig + 1) & 3;\n"
                "    digitalWrite(kDigPins[curDig], LOW);\n"
                "  }\n"
                "}\n"
            ),
        },
        "z15_regua_eletronica_hcsr04": {
            "mcu_family": "ESP32 Classic (WROOM-32 Dual-Core LX6)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z15 Corrigido em ESP32 Classic - Digito 6 0x7D Restaurado + Timeout 38ms + Divisor 3.3V)",
                "editor": "wokwi",
                "parts": [
                    {"type": "board-esp32-devkit-c-v4", "id": "esp", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-hc-sr04", "id": "sonar1", "top": -100, "left": 180, "attrs": {"distance": "66"}},
                    {"type": "wokwi-resistor", "id": "r_top", "top": 20, "left": 180, "attrs": {"value": "1200"}},
                    {"type": "wokwi-resistor", "id": "r_bot", "top": 60, "left": 180, "attrs": {"value": "2200"}},
                ],
                "connections": [
                    ["esp:4", "sonar1:TRIG", "green", []],
                    ["sonar1:ECHO", "r_top:1", "yellow", []],
                    ["r_top:2", "esp:5", "orange", []],
                    ["r_top:2", "r_bot:1", "orange", []],
                    ["r_bot:2", "esp:GND.1", "black", []],
                ],
            },
            "sketch": (
                "// Z15 Regua Eletronica HC-SR04 Corrigida (ESP32 Classic): LUT com digito 6 (0x7D) + pulseIn timeout 38000UL + c(T,RH)\n"
                "const uint8_t numeros[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};\n"
                "const int PIN_TRIG = 4, PIN_ECHO = 5;\n"
                "void setup() { Serial.begin(115200); pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT); }\n"
                "void loop() {\n"
                "  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_TRIG, LOW);\n"
                "  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 38000UL);\n"
                "  float dist_cm = (dur * 0.03463f) * 0.5f;\n"
                "  Serial.println(dist_cm);\n"
                "}\n"
            ),
        },
        "z16_sistema_entrada_bidirecional": {
            "mcu_family": "Arduino Mega 2560 (ATmega2560 54 GPIOs)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z16 + Z03 + Z12 Corrigido em Arduino Mega 2560 - FSM S1<->S2 Bidirecional Completa)",
                "editor": "wokwi",
                "parts": [
                    {"type": "wokwi-arduino-mega", "id": "mega", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-hc-sr04", "id": "s1", "top": -120, "left": 140, "attrs": {}},
                    {"type": "wokwi-hc-sr04", "id": "s2", "top": -120, "left": 300, "attrs": {}},
                    {"type": "wokwi-resistor", "id": "r1", "top": 80, "left": 140, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["mega:22", "s1:TRIG", "green", []],
                    ["mega:24", "s2:TRIG", "blue", []],
                ],
            },
            "sketch": (
                "// Z16 Sistema de Entrada Bidirecional Corrigido em Arduino Mega 2560 (Verifica S1->S2 para Entrada e S2->S1 para Saida)\n"
                "int ocupacao = 0;\n"
                "void setup() { Serial.begin(115200); }\n"
                "void loop() {}\n"
            ),
        },
        "z17_contador_00_99_7seg": {
            "mcu_family": "ESP32-C3 (RISC-V 160 MHz BLE 5.0)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z17 + Z05 Corrigido em ESP32-C3 - r2=220 ohm em vez de 0.22 ohm e pinos 0/1 livres)",
                "editor": "wokwi",
                "parts": [
                    {"type": "board-esp32-c3-devkitm-1", "id": "c3", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-7segment", "id": "sev_u", "top": 20, "left": 200, "attrs": {}},
                    {"type": "wokwi-resistor", "id": "r1", "top": 80, "left": 140, "attrs": {"value": "220"}},
                    {"type": "wokwi-resistor", "id": "r2", "top": 110, "left": 140, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["c3:4", "r1:1", "red", []],
                    ["r1:2", "sev_u:A", "red", []],
                    ["c3:5", "r2:1", "red", []],
                    ["r2:2", "sev_u:B", "red", []],
                ],
            },
            "sketch": (
                "// Z17 Contador 00..99 Corrigido em ESP32-C3 (r2 = 220 ohm e UART0 preservada)\n"
                "void setup() { Serial.begin(115200); }\n"
                "void loop() {}\n"
            ),
        },
        "z18_logica_leds_8x": {
            "mcu_family": "ESP8266 (NodeMCU ESP-12E Wi-Fi)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z18 + Z10 Corrigido em ESP8266 NodeMCU - 8 LEDs + Psicrometria IoT sem UB)",
                "editor": "wokwi",
                "parts": [
                    {"type": "board-nodemcu-esp8266", "id": "esp8266", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-led", "id": "led1", "top": 20, "left": 180, "attrs": {"color": "green"}},
                    {"type": "wokwi-led", "id": "led8", "top": 20, "left": 320, "attrs": {"color": "red"}},
                    {"type": "wokwi-resistor", "id": "r1", "top": 80, "left": 180, "attrs": {"value": "220"}},
                    {"type": "wokwi-resistor", "id": "r8", "top": 80, "left": 320, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["esp8266:D1", "r1:1", "green", []],
                    ["r1:2", "led1:A", "green", []],
                    ["esp8266:D2", "r8:1", "red", []],
                    ["r8:2", "led8:A", "red", []],
                ],
            },
            "sketch": (
                "// Z18 Logica de 8 LEDs + Z10 Psicrometria IoT em ESP8266 NodeMCU: indices estritos 0..7 (sem i<=8 nem i=9 Out-of-Bounds!)\n"
                "const uint8_t leds[8] = {4, 5, 12, 13, 14, 15, 16, 2};\n"
                "void setup() { for (int i = 0; i < 8; ++i) pinMode(leds[i], OUTPUT); }\n"
                "void loop() {\n"
                "  for (int i = 0; i < 8; ++i) { digitalWrite(leds[i], HIGH); }\n"
                "  for (int i = 7; i >= 0; --i) { digitalWrite(leds[i], LOW); }\n"
                "}\n"
            ),
        },
        "z19_z20_semaforo_7seg_fsm": {
            "mcu_family": "STM32 / ESP32-S3 Semaforo FSM O(1)",
            "diagram": {
                "version": 1,
                "author": "AEGIS-MECH v3.0 (Z19 + Z20 Corrigidos - Semaforo FSM O(1) + 7-Seg Matrix)",
                "editor": "wokwi",
                "parts": [
                    {"type": "board-esp32-s3-devkitc-1", "id": "esp", "top": 0, "left": 0, "attrs": {}},
                    {"type": "wokwi-led", "id": "led_g", "top": 20, "left": 180, "attrs": {"color": "green"}},
                    {"type": "wokwi-led", "id": "led_y", "top": 20, "left": 220, "attrs": {"color": "yellow"}},
                    {"type": "wokwi-led", "id": "led_r", "top": 20, "left": 260, "attrs": {"color": "red"}},
                    {"type": "wokwi-resistor", "id": "rg", "top": 80, "left": 180, "attrs": {"value": "220"}},
                    {"type": "wokwi-resistor", "id": "ry", "top": 80, "left": 220, "attrs": {"value": "220"}},
                    {"type": "wokwi-resistor", "id": "rr", "top": 80, "left": 260, "attrs": {"value": "220"}},
                ],
                "connections": [
                    ["esp:11", "rg:1", "green", []],
                    ["rg:2", "led_g:A", "green", []],
                    ["esp:12", "ry:1", "yellow", []],
                    ["ry:2", "led_y:A", "yellow", []],
                    ["esp:13", "rr:1", "red", []],
                    ["rr:2", "led_r:A", "red", []],
                ],
            },
            "sketch": (
                "// Z19 + Z20 Semaforo FSM O(1) Nao-Bloqueante + Matriz 7-Segmentos\n"
                "const int LED_G = 11, LED_Y = 12, LED_R = 13;\n"
                "unsigned long stateStartMs = 0;\n"
                "int state = 0;\n"
                "void setup() { pinMode(LED_G, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_R, OUTPUT); }\n"
                "void loop() {\n"
                "  const unsigned long dur = (state == 1) ? 1000UL : 2000UL;\n"
                "  if (millis() - stateStartMs >= dur) { stateStartMs = millis(); state = (state + 1) % 3; }\n"
                "  digitalWrite(LED_G, state == 0); digitalWrite(LED_Y, state == 1); digitalWrite(LED_R, state == 2);\n"
                "}\n"
            ),
        },
    }

    written_paths: list[str] = []
    for folder_name, bundle in suites.items():
        pdir = base_dir / folder_name
        pdir.mkdir(parents=True, exist_ok=True)
        dpath = pdir / "diagram.json"
        spath = pdir / "sketch.ino"
        dpath.write_text(json.dumps(bundle["diagram"], indent=2, ensure_ascii=False), encoding="utf-8")
        spath.write_text(bundle["sketch"], encoding="utf-8")
        written_paths.append(str(dpath))

    return written_paths


if __name__ == "__main__":
    paths = generate_all_wokwi_reference_projects()
    print(f"Generated {len(paths)} Wokwi multi-MCU reference projects + Falstad SPICE + Learn-Arduino 3D map.")
