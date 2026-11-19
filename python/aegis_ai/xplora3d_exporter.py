"""Exportador e Auditor de Circuito 3D Multi-Protoboard + Firmware MicroPython para Xplora3D (v2.1).

Sintetiza a bancada mecatronica 3D completa e ultra-organizada do AEGIS-MECH S3 v3.0 Ultra
no formato nativo `.x3d` (JSON v2.1) e URL compartilhavel `#p=<base64url_zlib>` compativel com
https://xplora.phinalabs.com/3d/#, utilizando **5 Protoboards Acopladas (1.110 pontos)** e
**7 Zonas Funcionais com Alinhamento Vertical de Colunas (Delta-X ~ 0)**, integrando os
20 repositorios `.zip` (`Z01..Z20`) e todas as 24 portas GPIO/ADC/PWM/I2C do barramento de 32 pinos:

  - **Zona 1 — Protoboard Principal 400 pts (Centro: Front-End Analogico, Psicrometria & Trigger)**:
    * `Cols 1..5`: Botao `E-Stop / Auth Trigger` (`switch-245`) + Pull-Down `10 kΩ` (`resistor-250`) -> `A4` (`mcu-27`)
    * `Cols 6..10`: Divisor de Tensao HC-SR04 `ECHO` 5V->3.23V (`1.2 kΩ` `resistor-301` + `2.2 kΩ` `resistor-302`) -> `B4` (`mcu-5`)
    * `Cols 11..15`: Termistor `NTC 10k B3950` (`thermistor-329`) + Divisor `10 kΩ` (`resistor-303`) -> `A0` (`mcu-31`) (`Z11/Z04`)
    * `Cols 17..21`: Potenciometro Rotativo `10 kΩ` Proxy Continuo ToF (`potentiometer-246`) -> `B1` (`mcu-22`)
    * `Cols 23..25`: Sensor Termico `TMP36` Bulbo Seco ATS (`tmp36-247`) -> `A5` (`mcu-26`) (`Z10/Z12`)
    * `Cols 27..30`: Sensor Optico `LDR` 2FA (`ldr-248`) + Divisor `10 kΩ` (`resistor-249`) -> `A1` (`mcu-30`) (`Z04/Z11`)

  - **Zona 2 — Protoboard Extra 400 pts (`bb1:`, Direita: Logica Digital 74HC595, LED Bar 10x, Semaforo FSM & Buzzer Silencioso)**:
    * `bb1:Cols 3..10`: Registrador de Deslocamento `74HC595` 8-Bit SIPO DIP-16 (`shiftregister-101`) -> `SER=A6`, `SRCLK=A7`, `RCLK=B0` (`Z13/Z19`)
    * `bb1:Cols 12..21`: Barra Grafica `LED Bar 10-Segmentos` (`ledbar-102`) acionada por `QA..QH` do `74HC595` (`Z18`)
    * `bb1:Cols 23..24`: Buzzer Piezo (`buzzer-272`) -> `A12` (`mcu-11`), mantido 100% SILENCIOSO (`LOW`) em regime continuo
    * `bb1:Cols 26, 28, 30`: Semaforo FSM Nao-Bloqueante 3 LEDs (`Z20`: Verde `B13`, Amarelo `A8`, Vermelho `B12`) + 3x Resistores `220 Ω`

  - **Zona 3 — Mini Protoboard 1 (`mn1:`, Frontal-Esquerda: Sensores Ambientais Digitais & Robotica)**:
    * `mn1:Cols 3..5`: Sensor Digital de Temperatura e Umidade `DHT11` (`dht-103`) -> `A11` (`mcu-12`) (`Z10`)
    * `mn1:Cols 9..12`: Sensor Optico Reflexivo `TCRT5000` Seguidor de Linha / Proximidade (`reflect-106`) -> `B14` (`mcu-17`) (`Z03`)
    * `mn1:Cols 14..16`: Sensor de Forca/Carga `FSR 402` (`fsr-107`) + Divisor `10 kΩ` (`resistor-fsr-111`) (`Z12` / `Learn Arduino L15`)

  - **Zona 4 — Mini Protoboard 2 (`mn2:`, Frontal-Central: Cockpit Exclusivo Character LCD 16x2 I2C)**:
    * `mn2:Cols 3..6`: Display `Character LCD 16x2 I2C` (`0x27`, `lcd-104`) isolado sem sobreposicao no barramento `B6=SCL / B7=SDA` (`Z12/Z15/Z16`)

  - **Zona 5 — Mini Protoboard 3 (`mn3:`, Frontal-Direita: Display 4-Digitos TM1637, Regulacao LDO 3.3V & Power Stage MOSFET)**:
    * `mn3:Cols 2..5`: Display 4 Digitos 7-Segmentos `TM1637` (`tm1637-105`) -> `CLK=A2`, `DIO=A3` (`Z13/Z14/Z17/Z19`)
    * `mn3:Cols 9..11`: Regulador LDO `AMS1117-3.3` (`voltage_regulator-109`) + Capacitor Eletrolitico `100 µF` (`capacitor-110`)
    * `mn3:Cols 13..16`: Power Stage MOSFET Canal-N `IRLZ44N` (`mosfet-320`) + Diodo Flyback `1N4001` (`diode-330`) -> `B15` (`mcu-16`)

  - **Zona 6 — Baia Traseira de Sensores ToF & Telemetria I2C (`z = -5.4`, atras de `bb1:` com margem livre de 1.6 unidades)**:
    * `ultrasonic-317` (`HC-SR04` em `x=6.0, z=-5.4` alinhado com `bb1:col 4` -> `TRIG=B3, ECHO=c6`)
    * `mpu6050-318` (`MPU6050` 6-DoF IMU em `x=9.1, z=-5.4` alinhado com `bb1:col 16` -> `I2C B6/B7`) (`Z05`)
    * `oled-325` (`0.96" OLED SSD1306 128x64` em `x=12.1, z=-5.4` alinhado com `bb1:col 28` -> `I2C B6/B7`)

  - **Zona 7 — Baia Frontal de Atuadores Mecatronicos (`z = +9.4`, a frente de `mn1..mn3` com margem livre de 2.2 unidades)**:
    * `servo-35` (`Servo SG90` S-Curve em `x=-2.3, z=9.4` alinhado com `mn1:col 7` -> `B5`)
    * `neopixel-331` (`WS2812B 8x NeoPixel` em `x=4.4, z=9.4` alinhado com `mn2:col 9` -> `B8`)
    * `relay-108` (`Modulo Rele SPDT 1-Ch` Fail-Secure em `x=9.1, z=9.4` alinhado com `mn3:col 3` -> `A15`)
    * `motor-323` (`Motor DC / Solenoide da Trava` em `x=12.2, z=9.4` alinhado com `mn3:col 15` -> Dreno do MOSFET `mn3:e14`)
"""

from __future__ import annotations

import ast
import base64
import json
from pathlib import Path
import shutil
from typing import Any
import zlib

XPLORA3D_BASE_URL = "https://xplora.phinalabs.com/3d/#p="

UNDA_XPLORA_HEADER_MAP: dict[str, str] = {
    "mcu-1": "GND",
    "mcu-2": "5V",
    "mcu-3": "A15",
    "mcu-4": "B3",
    "mcu-5": "B4",
    "mcu-6": "B5",
    "mcu-7": "GND",
    "mcu-8": "B6",
    "mcu-9": "B7",
    "mcu-10": "3V3",
    "mcu-11": "A12",
    "mcu-12": "A11",
    "mcu-13": "GND",
    "mcu-14": "B8",
    "mcu-15": "A8",
    "mcu-16": "B15",
    "mcu-17": "B14",
    "mcu-18": "B13",
    "mcu-19": "B12",
    "mcu-20": "GND",
    "mcu-21": "3V3",
    "mcu-22": "B1",
    "mcu-23": "B0",
    "mcu-24": "A7",
    "mcu-25": "A6",
    "mcu-26": "A5",
    "mcu-27": "A4",
    "mcu-28": "A3",
    "mcu-29": "A2",
    "mcu-30": "A1",
    "mcu-31": "A0",
    "mcu-32": "GND",
}

MULTI_MCU_ARCHITECTURE_CATALOG: list[dict[str, str]] = [
    {
        "family": "ESP32-S3 (Dual-Core Xtensa LX7 + Vector PIE)",
        "board_id": "board-esp32-s3-devkitc-1 / UNDA.Xplora UI",
        "role": "Controlador Principal TinyML INT8 + Biometria Cosseno 16-D + BLE HID + ESP-NOW",
        "firmware": "firmware/src/main.cpp & firmware/arduino_aegis_s3_all_in_one/arduino_aegis_s3_all_in_one.ino",
    },
    {
        "family": "STM32F411CE (ARM Cortex-M4F 100 MHz)",
        "board_id": "UNDA.Xplora 32-Pin Header (A0..A15 / B0..B15)",
        "role": "Co-Processador de Tempo Real da Bancada 3D Xplora3D (24 GPIOs + I2C + PWM S-Curve + 74HC595)",
        "firmware": "docs/xplora3d/aegis_mech_s3_v3.x3d (MicroPython Engine)",
    },
    {
        "family": "ESP32-C3 (RISC-V Single-Core 160 MHz BLE 5.0)",
        "board_id": "board-esp32-c3-devkitm-1",
        "role": "Varinha Inercial Vestivel BLE HID Air Mouse + Calibracao Giroscopica 500 Amostras (Z05)",
        "firmware": "firmware/tools/comet_air_mouse_wand_c3/comet_air_mouse_wand_c3.ino",
    },
    {
        "family": "ESP32 Classic (Dual-Core Xtensa LX6 Wi-Fi/BT)",
        "board_id": "board-esp32-devkit-c-v4",
        "role": "Gateway de Telemetria Ultrasonica HC-SR04 + Termistor NTC 10k + Ponte Serial (Z04/Z15/Z16)",
        "firmware": "docs/wokwi/z15_regua_eletronica_hcsr04/sketch.ino",
    },
    {
        "family": "ESP8266 (NodeMCU ESP-12E Tensilica L106 Wi-Fi)",
        "board_id": "board-nodemcu-esp8266",
        "role": "No IoT de Psicrometria Bulbo Seco/Umido Magnus-Tetens & Logica de 8 LEDs (Z10/Z18)",
        "firmware": "docs/wokwi/z18_logica_leds_8x/sketch.ino",
    },
    {
        "family": "Arduino Uno R3 (ATmega328P 16 MHz)",
        "board_id": "wokwi-arduino-uno",
        "role": "Controlador Didatico 74HC595 + 7-Seg POV + Semaforo FSM + 16 Licoes Learn-Arduino (Z13/Z14/Z17..Z20)",
        "firmware": "docs/wokwi/z13_placar_digital_74hc595/sketch.ino",
    },
    {
        "family": "Arduino Mega 2560 (ATmega2560 16 MHz 54 GPIOs)",
        "board_id": "wokwi-arduino-mega",
        "role": "Controlador Robotico 5x Sensores IR TCRT5000 PID + Portal Bidirecional Duplo Sonar (Z03/Z12/Z16)",
        "firmware": "docs/wokwi/z16_sistema_entrada_bidirecional/sketch.ino",
    },
]


def _col_x(col: float) -> float:
    """Coordenada X na protoboard principal de 400 pontos (origem x=0.0)."""
    return round((col - 15.5) * 0.255, 6)


def _bb1_col_x(col: float) -> float:
    """Coordenada X na protoboard extra de 400 pontos acoplada a direita (origem x=9.0)."""
    return round(9.0 + (col - 15.5) * 0.255, 6)


def _mn_col_x(slot: int, col: float) -> float:
    """Coordenada X nas 3 mini protoboards de 170 pontos acopladas a frente."""
    origins = {0: -1.798, 1: 4.425, 2: 10.648}
    return round(origins[slot] + (col - 9.0) * 0.255, 6)


AEGIS_MICROPYTHON_CODE = """# AEGIS-MECH S3 v3.0 Ultra - Bancada 3D Multi-Protoboard (5 Protoboards, 7 Zonas, 20 Repos .zip)
# Arquiteturas Integradas: ESP32-S3 + STM32F411CE + ESP32-C3 + ESP32 + ESP8266 + Arduino Uno/Mega
# - Zona 1 (Main 400): Botao A4, Divisor HC-SR04 B4, NTC 10k B3950 A0, Pot ToF B1, TMP36 A5, LDR A1
# - Zona 2 (Extra 400 bb1): 74HC595 (A6/A7/B0) + LED Bar 10-seg, Buzzer Silencioso (A12), Semaforo FSM (B13/A8/B12)
# - Zona 3 (Mini 1 mn1): DHT11 Digital (A11), TCRT5000 IR Line/Prox (B14), Force Sensor FSR 402
# - Zona 4 (Mini 2 mn2): Character LCD 16x2 I2C (0x27) dedicado sem sobreposicao
# - Zona 5 (Mini 3 mn3): Display 4-Digitos TM1637 (A2/A3) + Regulador AMS1117-3.3 + Cap 100uF + MOSFET IRLZ44N (B15)
# - Zona 6/7 (Baias Externas): HC-SR04 (B3/B4), MPU6050 (I2C), OLED 0.96" (I2C), Servo SG90 (B5),
#                              NeoPixel 8x (B8), Rele Fail-Secure (A15), Motor DC / Solenoide

from machine import Pin, ADC, PWM, I2C
from time import sleep_ms
import math

# 1. Mapeamento Completo das 24 Portas GPIO/ADC/PWM/I2C
btn_trig   = Pin('A4', Pin.IN, Pin.PULL_DOWN)  # mcu-27: Botao Trigger / Auth Toggle (Main col 1..3)
trig_pin   = Pin('B3', Pin.OUT)                # mcu-4 : HC-SR04 TRIG (Baia Traseira)
echo_pin   = Pin('B4', Pin.IN)                 # mcu-5 : HC-SR04 ECHO 3.23V (Main col 6..10)
pot_tof    = ADC(Pin('B1'))                    # mcu-22: Trajetoria ToF (Main col 17..21)
ats_tmp36  = ADC(Pin('A5'))                    # mcu-26: Bulbo Seco TMP36 (Main col 23..25)
ntc_adc    = ADC(Pin('A0'))                    # mcu-31: Termistor NTC 10k B3950 (Main col 11..15)
ldr_2fa    = ADC(Pin('A1'))                    # mcu-30: Luminancia Optica 2FA (Main col 27..30)
tcrt_dout  = Pin('B14', Pin.IN)                # mcu-17: Sensor Optico TCRT5000 DOUT (Mini 1 col 11)
dht_pin    = Pin('A11', Pin.IN)                # mcu-12: Sensor Digital DHT11 DATA (Mini 1 col 4)

# Registrador de Deslocamento 74HC595 -> LED Bar 10-Segmentos (Extra 400 bb1)
sr_ser     = Pin('A6', Pin.OUT)                # mcu-25: 74HC595 SER (bb1:f5)
sr_clk     = Pin('A7', Pin.OUT)                # mcu-24: 74HC595 SRCLK (bb1:f8)
sr_latch   = Pin('B0', Pin.OUT)                # mcu-23: 74HC595 RCLK (bb1:f7)

# Atuadores PWM & Fail-Secure Lock
servo_pwm  = PWM(Pin('B5'))                    # mcu-6 : Servo SG90 Trava S-Curve (50 Hz)
servo_pwm.freq(50)
mosfet_pwm = PWM(Pin('B15'))                   # mcu-16: Gate MOSFET IRLZ44N (Mini 3 col 13)
mosfet_pwm.freq(1000)
relay_pin  = Pin('A15', Pin.OUT)               # mcu-3 : Modulo Rele SPDT Fail-Secure
relay_pin.value(0)

# Sinalizacao Semafaro FSM (Z20) + Buzzer 100% Silencioso em Regime
buz_pin    = Pin('A12', Pin.OUT)               # mcu-11: Buzzer Piezo (bb1:col 23..24 - SILENCIOSO!)
buz_pin.value(0)
led_green  = Pin('B13', Pin.OUT)               # mcu-18: LED Verde (bb1:col 26)
led_yellow = Pin('A8', Pin.OUT)                # mcu-15: LED Amarelo (bb1:col 28)
led_red    = Pin('B12', Pin.OUT)               # mcu-19: LED Vermelho (bb1:col 30)

# Barramento I2C Compartilhado (B6=SCL, B7=SDA -> OLED 0x3C + LCD 16x2 0x27 + MPU6050 0x68)
i2c = None
oled = None
lcd = None
try:
    i2c = I2C(0, scl=Pin('B6'), sda=Pin('B7'))
    import ssd1306
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
except Exception:
    oled = None

try:
    if 'I2cLcd' in globals() and i2c is not None:
        lcd = I2cLcd(i2c, 0x27, 2, 16)
except Exception:
    lcd = None

# Display 4 Digitos 7-Segmentos TM1637 (Mini 3 col 2..5 -> CLK=A2, DIO=A3)
tm = None
try:
    if 'TM1637' in globals():
        tm = TM1637(clk=Pin('A2'), dio=Pin('A3'))
    else:
        import tm1637
        tm = tm1637.TM1637(clk=Pin('A2'), dio=Pin('A3'))
except Exception:
    tm = None

# Barra Enderecavel WS2812B NeoPixel (8 LEDs em B8 / mcu-14)
np = None
try:
    import neopixel
    np = neopixel.NeoPixel(Pin('B8', Pin.OUT), 8)
except Exception:
    np = None

def shift_out_74hc595(byte_val):
    sr_latch.value(0)
    for bit in range(7, -1, -1):
        sr_clk.value(0)
        sr_ser.value((byte_val >> bit) & 1)
        sr_clk.value(1)
    sr_clk.value(0)
    sr_latch.value(1)

def read_ntc_celsius(adc_u16):
    clamped = max(16, min(65519, adc_u16))
    r_ntc = 10000.0 * (clamped / float(65535 - clamped))
    inv_t = (1.0 / 298.15) + (1.0 / 3950.0) * math.log(r_ntc / 10000.0)
    return max(-55.0, min(150.0, (1.0 / inv_t) - 273.15))

def compute_psychrometry_and_sound(ts_c, tm_c):
    es_s = 6.1078 * (10.0 ** ((7.5 * ts_c) / (ts_c + 237.3)))
    es_m = 6.1078 * (10.0 ** ((7.5 * tm_c) / (tm_c + 237.3)))
    ea = max(0.5, es_m - 0.668 * max(0.0, ts_c - tm_c))
    rh = max(5.0, min(100.0, (ea / max(es_s, 0.1)) * 100.0))
    xw = ea / 1013.25
    c_mps = 331.3 * math.sqrt(1.0 + (ts_c / 273.15)) * (1.0 + 0.16 * xw)
    return rh, c_mps

# Filtro de Kalman 1D na Forma de Joseph (auto-inicializado no 1o frame)
k_init = False
k_x = 152.0
k_v = 0.0
k_p00 = 16.0
k_p11 = 10.0
Q_POS = 1.5
R_MEAS = 25.0

def kalman_joseph_step(z_mm, dt_s):
    global k_init, k_x, k_v, k_p00, k_p11
    if not k_init:
        k_x = z_mm
        k_v = 0.0
        k_init = True
        return k_x, k_v, 0.0, False
    x_pred = k_x + k_v * dt_s
    p00_pred = k_p00 + dt_s * dt_s * k_p11 + Q_POS
    innov = z_mm - x_pred
    s_cov = p00_pred + R_MEAS
    maha_d = math.sqrt((innov * innov) / s_cov)
    if maha_d > 4.5:
        k_x = x_pred + 0.25 * innov
        k_p00 = p00_pred + R_MEAS
        return k_x, k_v, maha_d, True
    gain = p00_pred / s_cov
    one_m_k = 1.0 - gain
    k_x = x_pred + gain * innov
    k_v = 0.7 * k_v + 0.3 * (innov / max(dt_s, 0.05))
    k_p00 = (one_m_k * one_m_k) * p00_pred + (gain * gain) * R_MEAS
    return k_x, k_v, maha_d, False

def scurve_quintic(tau):
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    t2 = tau * tau
    t3 = t2 * tau
    return t3 * (10.0 - 15.0 * tau + 6.0 * t2)

def write_servo_deg(deg):
    duty = int((0.05 + (deg / 180.0) * 0.05) * 65535)
    servo_pwm.duty_u16(duty)

def update_displays(temp_c, rh_pct, c_air, dist_mm, seq_str, cos_sim, level_0_8, unlocked, fault):
    # 1. 74HC595 -> LED Bar 10-Segmentos (mascara thermometer bargraph)
    mask = (1 << level_0_8) - 1
    shift_out_74hc595(mask)

    # 2. NeoPixel WS2812B 8x
    if np is not None:
        try:
            for idx in range(8):
                if fault:
                    np[idx] = (180, 0, 0)
                elif unlocked:
                    np[idx] = (0, 170, 45) if idx < level_0_8 else (0, 25, 10)
                else:
                    np[idx] = (0, 80, 180) if idx < level_0_8 else (8, 12, 24)
            np.write()
        except Exception:
            pass

    # 3. TM1637 4-Digit 7-Segment Display (distancia Kalman em mm)
    if tm is not None:
        try:
            tm.number(int(round(dist_mm)))
        except Exception:
            pass

    # 4. Character LCD 16x2 I2C (0x27)
    if lcd is not None:
        try:
            lcd.move_to(0, 0)
            l1 = ('AEGIS ' + ('OPEN-90' if unlocked else 'SCAN-OK') + '   ')[:16]
            lcd.putstr(l1)
            lcd.move_to(0, 1)
            l2 = ('d=' + str(int(round(dist_mm))) + 'mm ' + seq_str + '        ')[:16]
            lcd.putstr(l2)
        except Exception:
            pass

    # 5. OLED SSD1306 128x64 I2C (0x3C)
    if oled is not None:
        try:
            oled.fill(0)
            oled.text('AEGIS-MECH S3 v3', 0, 0)
            oled.text(str(round(temp_c, 1)) + 'C RH:' + str(int(rh_pct)) + '% ' + str(int(c_air)) + 'm/s', 0, 13)
            oled.text('Kalman:' + str(round(dist_mm, 1)) + 'mm', 0, 26)
            oled.text('Seq:' + seq_str + ' B:' + str(round(cos_sim, 2)), 0, 39)
            oled.text('ST:' + ('FAULT!' if fault else ('OPEN-90' if unlocked else 'SCAN-OK')), 0, 52)
            oled.show()
        except Exception:
            pass

GESTURE_PASS = ['I_v', 'O', 'L']
WAVE_OFFSETS = [-55.0, -45.0, 0.0, 10.0, 95.0, 105.0, 0.0, 0.0]
seq_idx = 0
unlocked = False
hold_open_ticks = 0
scurve_step = 0
prev_btn = 0
last_line = ''
tick = 0

write_servo_deg(0)
mosfet_pwm.duty_u16(0)
relay_pin.value(0)
buz_pin.value(0)
led_green.value(0)
led_yellow.value(1)
led_red.value(0)
shift_out_74hc595(0)

print('=== AEGIS-MECH S3 v3.0 ULTRA | 5-BREADBOARD 7-ZONE MULTI-MCU TWIN READY ===')

while True:
    v_tmp = (ats_tmp36.read_u16() / 65535.0) * 3.3
    ts_c = (v_tmp - 0.5) * 100.0
    ntc_c = read_ntc_celsius(ntc_adc.read_u16())
    tm_c = min(ts_c, 0.7 * ts_c + 0.3 * ntc_c - 1.8)
    rh_pct, c_air = compute_psychrometry_and_sound(ts_c, tm_c)

    trig_pin.value(1)
    trig_pin.value(0)
    base_mm = 40.0 + (pot_tof.read_u16() / 65535.0) * 260.0
    wave_mm = WAVE_OFFSETS[(tick // 4) % len(WAVE_OFFSETS)]
    raw_mm = max(55.0, min(340.0, base_mm + wave_mm))
    lux_pct = round((ldr_2fa.read_u16() / 65535.0) * 100.0)
    line_ok = tcrt_dout.value()

    dist_mm, vel_mms, maha_d, outlier = kalman_joseph_step(raw_mm, 0.12)

    if dist_mm < 130.0:
        pred_cls = 'I_v'
    elif dist_mm < 215.0:
        pred_cls = 'O'
    else:
        pred_cls = 'L'

    bio_cos_sim = 0.93 if lux_pct >= 10 else 0.64
    fault_active = (ts_c > 65.0) or (ts_c < -15.0)

    # Botao manual A4 (unico evento que emite bip curto de 25ms)
    now_btn = btn_trig.value()
    if now_btn == 1 and prev_btn == 0:
        unlocked = not unlocked
        hold_open_ticks = 25 if unlocked else 0
        buz_pin.value(1)
        sleep_ms(25)
        buz_pin.value(0)
    prev_btn = now_btn

    # Buzzer mantido 100% SILENCIOSO (LOW) em todo o regime continuo
    buz_pin.value(0)

    if (not fault_active) and (not unlocked) and (bio_cos_sim >= 0.75):
        if pred_cls == GESTURE_PASS[seq_idx]:
            seq_idx += 1
            if seq_idx >= 3:
                unlocked = True
                hold_open_ticks = 18
                seq_idx = 0

    if unlocked and hold_open_ticks > 0:
        hold_open_ticks -= 1
        if hold_open_ticks == 0:
            unlocked = False

    if fault_active:
        unlocked = False
        write_servo_deg(0)
        mosfet_pwm.duty_u16(0)
        relay_pin.value(0)
        led_green.value(0)
        led_yellow.value(0)
        led_red.value(1)
        angle_deg = 0
    elif unlocked:
        scurve_step = min(8, scurve_step + 1)
        s_val = scurve_quintic(scurve_step / 8.0)
        angle_deg = int(round(90.0 * s_val))
        write_servo_deg(angle_deg)
        pwm_duty = int((0.85 if scurve_step < 4 else 0.35) * 65535)
        mosfet_pwm.duty_u16(pwm_duty)
        relay_pin.value(1)
        led_green.value(1)
        led_yellow.value(0)
        led_red.value(0)
    else:
        scurve_step = max(0, scurve_step - 1)
        s_val = scurve_quintic(scurve_step / 8.0)
        angle_deg = int(round(90.0 * s_val))
        write_servo_deg(angle_deg)
        mosfet_pwm.duty_u16(0)
        relay_pin.value(0)
        led_green.value(0)
        led_yellow.value(1)
        led_red.value(0)

    bar_lvl = min(8, max(1, int(round((dist_mm / 340.0) * 8.0))))
    seq_str = '>'.join(GESTURE_PASS[:seq_idx]) if seq_idx > 0 else (
        'I_v>O>L' if unlocked else 'READY'
    )
    update_displays(ts_c, rh_pct, c_air, dist_mm, seq_str, bio_cos_sim, bar_lvl, unlocked, fault_active)

    line = (
        'AEGIS | T=' + str(round(ts_c, 1)) + 'C RH=' + str(int(rh_pct)) + '% c=' + str(round(c_air, 1)) + 'm/s'
        + ' | d=' + str(round(dist_mm, 1)) + 'mm (' + pred_cls + ')'
        + ' | seq=' + seq_str
        + ' | cos=' + str(round(bio_cos_sim, 2))
        + ' | srv=' + str(angle_deg) + 'deg'
        + ' | st=' + ('UNLOCKED' if unlocked else 'SCANNING_OK')
    )
    if line != last_line:
        last_line = line
        print(line)

    tick += 1
    sleep_ms(120)
"""


def build_aegis_xplora3d_project() -> dict[str, Any]:
    """Constroi a bancada 3D v2.1 (40 componentes em 5 protoboards, 99 jumpers alinhados verticalmente) para o Xplora3D."""
    components: list[dict[str, Any]] = [
        # ====================================================================
        # PROTOBOARDS EXTRAS ACOPLADAS (4 extras + 1 principal = 5 protoboards / 1.110 furos)
        # ====================================================================
        {
            "id": "breadboard_400-1",
            "type": "breadboard_400",
            "value": 0,
            "unit": "",
            "position": [0.9, 0.015, 0.0],
            "rotation": [0, 0, 0],
            "locked": True,
            "leads": [],
            "props": {
                "kind": "400",
                "slot": 0,
                "boardColor": "#f4f1ea",
                "layout": "side",
            },
        },
        {
            "id": "breadboard_mini-2",
            "type": "breadboard_mini",
            "value": 0,
            "unit": "",
            "position": [-0.1798, 0.015, 0.54],
            "rotation": [0, 0, 0],
            "locked": True,
            "leads": [],
            "props": {
                "kind": "mini",
                "slot": 0,
                "boardColor": "#f4f1ea",
                "layout": "side",
            },
        },
        {
            "id": "breadboard_mini-3",
            "type": "breadboard_mini",
            "value": 0,
            "unit": "",
            "position": [0.4425, 0.015, 0.54],
            "rotation": [0, 0, 0],
            "locked": True,
            "leads": [],
            "props": {
                "kind": "mini",
                "slot": 1,
                "boardColor": "#f4f1ea",
                "layout": "side",
            },
        },
        {
            "id": "breadboard_mini-4",
            "type": "breadboard_mini",
            "value": 0,
            "unit": "",
            "position": [1.0648, 0.015, 0.54],
            "rotation": [0, 0, 0],
            "locked": True,
            "leads": [],
            "props": {
                "kind": "mini",
                "slot": 2,
                "boardColor": "#f4f1ea",
                "layout": "side",
            },
        },
        # ====================================================================
        # ZONA 1 — PROTOBOARD PRINCIPAL 400 PTS (Cols 1..30: Front-End Analogico & Psicrometria)
        # ====================================================================
        # 5. Botao E-Stop / Trigger de Autenticacao (cols 28..30, rows g/d -> A4 / mcu-27)
        {
            "id": "switch-245",
            "type": "switch",
            "value": 0,
            "unit": "",
            "position": [_col_x(29), 0.09, 0.0],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "A1", "holeId": "g28"},
                {"name": "A2", "holeId": "g30"},
                {"name": "B1", "holeId": "d28"},
                {"name": "B2", "holeId": "d30"},
            ],
            "props": {
                "capColor": "#e63300",
                "hasCap": True,
                "isPressed": False,
                "buttonType": "momentary",
            },
            "spiceModel": {"type": "R", "value": 1000000000},
        },
        # 6. Resistor Pull-Down 10k do Botao (h30 <-> h29)
        {
            "id": "resistor-250",
            "type": "resistor",
            "value": 10000,
            "unit": "Ω",
            "position": [_col_x(29.5), 0.351, -0.8925],
            "rotation": [0, 3.141592653589793, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "h30"},
                {"name": "lead2", "holeId": "h29"},
            ],
            "props": {
                "tolerance": "5%",
                "wattage": 0.25,
                "bodyColor": "#c9a06b",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 10000},
        },
        # 7. Divisor de Tensao HC-SR04 ECHO R1 = 1.2kΩ (c3 <-> c5, alinhado a B4 / mcu-5)
        {
            "id": "resistor-301",
            "type": "resistor",
            "value": 1200,
            "unit": "Ω",
            "position": [_col_x(4), 0.351, 0.8925],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "c3"},
                {"name": "lead2", "holeId": "c5"},
            ],
            "props": {
                "tolerance": "1%",
                "wattage": 0.25,
                "bodyColor": "#3b6ea5",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 1200},
        },
        # 8. Divisor de Tensao HC-SR04 ECHO R2 = 2.2kΩ (b5 <-> b7)
        {
            "id": "resistor-302",
            "type": "resistor",
            "value": 2200,
            "unit": "Ω",
            "position": [_col_x(6), 0.351, 1.1475],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "b5"},
                {"name": "lead2", "holeId": "b7"},
            ],
            "props": {
                "tolerance": "1%",
                "wattage": 0.25,
                "bodyColor": "#3b6ea5",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 2200},
        },
        # 9. Termistor NTC 10k Beta=3950 (Z11 Projeto12 / Z04 Exp 02: f21 <-> f22 -> A0 / mcu-31)
        {
            "id": "thermistor-329",
            "type": "thermistor",
            "value": 11441.48,
            "unit": "Ω",
            "position": [_col_x(21.5), 0.09, -0.3825],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "f21"},
                {"name": "lead2", "holeId": "f22"},
            ],
            "props": {"beta": 3950, "r25": 10000, "temperature": 22},
            "spiceModel": {"type": "R", "value": 11441.48},
        },
        # 10. Resistor Serie 10k do Termistor NTC (g22 <-> g23)
        {
            "id": "resistor-303",
            "type": "resistor",
            "value": 10000,
            "unit": "Ω",
            "position": [_col_x(22.5), 0.351, -0.6375],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "g22"},
                {"name": "lead2", "holeId": "g23"},
            ],
            "props": {
                "tolerance": "1%",
                "wattage": 0.25,
                "bodyColor": "#3b6ea5",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 10000},
        },
        # 11. Potenciometro 10k Proxy Continuo ToF (f13, f14, f15 -> B1 / mcu-22)
        {
            "id": "potentiometer-246",
            "type": "potentiometer",
            "value": 10000,
            "unit": "Ω",
            "position": [_col_x(14), 0.09, -0.3825],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "A", "holeId": "f13"},
                {"name": "B", "holeId": "f15"},
                {"name": "W", "holeId": "f14"},
            ],
            "props": {"wiper": 0.35, "taper": "B", "variant": "rotary"},
            "spiceModel": {"type": "R", "value": 10000},
        },
        # 12. Sensor Termico Acustico TMP36 ATS Bulbo Seco (f17, f18, f19 -> A5 / mcu-26)
        {
            "id": "tmp36-247",
            "type": "tmp36",
            "value": 0,
            "unit": "°C",
            "position": [_col_x(18), 0.09, -0.3825],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "vcc", "holeId": "f17"},
                {"name": "vout", "holeId": "f18"},
                {"name": "gnd", "holeId": "f19"},
            ],
            "props": {"temperature": 22},
            "spiceModel": {"type": "V", "modelName": "TMP36"},
        },
        # 13. Sensor Optico LDR 2FA (f25, f26 -> A1 / mcu-30)
        {
            "id": "ldr-248",
            "type": "ldr",
            "value": 9237.756,
            "unit": "Ω",
            "position": [_col_x(25.5), 0.09, -0.3825],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "f25"},
                {"name": "lead2", "holeId": "f26"},
            ],
            "props": {},
            "spiceModel": {"type": "R", "value": 9237.756},
        },
        # 14. Resistor Divisor LDR 10k (g26 <-> g27)
        {
            "id": "resistor-249",
            "type": "resistor",
            "value": 10000,
            "unit": "Ω",
            "position": [_col_x(26.5), 0.351, -0.6375],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "g26"},
                {"name": "lead2", "holeId": "g27"},
            ],
            "props": {
                "tolerance": "5%",
                "wattage": 0.25,
                "bodyColor": "#c9a06b",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 10000},
        },
        # ====================================================================
        # ZONA 2 — PROTOBOARD EXTRA 400 PTS (`bb1:`, Direita: 74HC595 + LED Bar 10x + Buzzer + Semaforo FSM)
        # ====================================================================
        # 15. Registrador de Deslocamento 74HC595 8-Bit SIPO DIP-16 (Z13/Z19: bb1:e3..e10 / bb1:f3..f10)
        {
            "id": "shiftregister-101",
            "type": "shiftregister",
            "value": 0,
            "unit": "",
            "position": [_bb1_col_x(6.5), 0.09, 0.0],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "QB", "holeId": "bb1:e3"},
                {"name": "QC", "holeId": "bb1:e4"},
                {"name": "QD", "holeId": "bb1:e5"},
                {"name": "QE", "holeId": "bb1:e6"},
                {"name": "QF", "holeId": "bb1:e7"},
                {"name": "QG", "holeId": "bb1:e8"},
                {"name": "QH", "holeId": "bb1:e9"},
                {"name": "GND", "holeId": "bb1:e10"},
                {"name": "VCC", "holeId": "bb1:f3"},
                {"name": "QA", "holeId": "bb1:f4"},
                {"name": "SER", "holeId": "bb1:f5"},
                {"name": "OE", "holeId": "bb1:f6"},
                {"name": "RCLK", "holeId": "bb1:f7"},
                {"name": "SRCLK", "holeId": "bb1:f8"},
                {"name": "SRCLR", "holeId": "bb1:f9"},
                {"name": "QHS", "holeId": "bb1:f10"},
            ],
            "props": {},
            "spiceModel": {"type": "X", "modelName": "74HC595"},
        },
        # 16. Barra Grafica LED Bar 10-Segmentos (Z18 logica-leds: bb1:f12..f21 / bb1:e12..e21)
        {
            "id": "ledbar-102",
            "type": "ledbar",
            "value": 0,
            "unit": "",
            "position": [_bb1_col_x(16.5), 0.09, 0.0],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "A1", "holeId": "bb1:f12"},
                {"name": "A2", "holeId": "bb1:f13"},
                {"name": "A3", "holeId": "bb1:f14"},
                {"name": "A4", "holeId": "bb1:f15"},
                {"name": "A5", "holeId": "bb1:f16"},
                {"name": "A6", "holeId": "bb1:f17"},
                {"name": "A7", "holeId": "bb1:f18"},
                {"name": "A8", "holeId": "bb1:f19"},
                {"name": "A9", "holeId": "bb1:f20"},
                {"name": "A10", "holeId": "bb1:f21"},
                {"name": "K1", "holeId": "bb1:e12"},
                {"name": "K2", "holeId": "bb1:e13"},
                {"name": "K3", "holeId": "bb1:e14"},
                {"name": "K4", "holeId": "bb1:e15"},
                {"name": "K5", "holeId": "bb1:e16"},
                {"name": "K6", "holeId": "bb1:e17"},
                {"name": "K7", "holeId": "bb1:e18"},
                {"name": "K8", "holeId": "bb1:e19"},
                {"name": "K9", "holeId": "bb1:e20"},
                {"name": "K10", "holeId": "bb1:e21"},
            ],
            "props": {"barColor": "#00d4ff", "count": 10, "pattern": None},
            "spiceModel": {"type": "D", "modelName": "LED"},
        },
        # 17. Buzzer Piezo de Confirmacao (bb1:b23=positive, bb1:b24=negative -> 100% SILENCIOSO em regime)
        {
            "id": "buzzer-272",
            "type": "buzzer",
            "value": 42,
            "unit": "Ω",
            "position": [_bb1_col_x(23.5), 0.09, 1.1475],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "negative", "holeId": "bb1:b24"},
                {"name": "positive", "holeId": "bb1:b23"},
            ],
            "props": {"frequency": 2300, "voltage": 5},
            "spiceModel": {"type": "R", "value": 42},
        },
        # 18. Resistor 220Ω LED Verde Auth OK (bb1:d26 <-> bb1:f26)
        {
            "id": "resistor-270",
            "type": "resistor",
            "value": 220,
            "unit": "Ω",
            "position": [_bb1_col_x(26), 0.351, 0.0],
            "rotation": [0, 1.5707963267948961, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "bb1:d26"},
                {"name": "lead2", "holeId": "bb1:f26"},
            ],
            "props": {
                "bodyColor": "#3b6ea5",
                "burnt": False,
                "tolerance": "5%",
                "wattage": 0.25,
            },
            "spiceModel": {"type": "R", "value": 220},
        },
        # 19. LED Verde Semaforo FSM / Auth OK (bb1:a26 anode -> bb1:-pwr-bot-26 cathode)
        {
            "id": "led-269",
            "type": "led",
            "value": 2,
            "unit": "V",
            "position": [_bb1_col_x(26), 0.09, 1.987],
            "rotation": [0, -4.712388980384686, 0],
            "locked": False,
            "leads": [
                {"name": "anode", "holeId": "bb1:a26"},
                {"name": "cathode", "holeId": "bb1:-pwr-bot-26"},
            ],
            "props": {
                "burnt": False,
                "clearBody": False,
                "forwardVoltage": 2,
                "ledColor": "#00ff55",
                "maxIntensity": 1,
                "shape": "round",
                "size": 5,
            },
            "spiceModel": {"type": "D", "modelName": "LED"},
        },
        # 20. Resistor 220Ω LED Amarelo Semaforo FSM / Scanning (Z20: bb1:d28 <-> bb1:f28)
        {
            "id": "resistor-285",
            "type": "resistor",
            "value": 220,
            "unit": "Ω",
            "position": [_bb1_col_x(28), 0.351, 0.0],
            "rotation": [0, 1.5707963267948961, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "bb1:d28"},
                {"name": "lead2", "holeId": "bb1:f28"},
            ],
            "props": {
                "bodyColor": "#3b6ea5",
                "burnt": False,
                "tolerance": "5%",
                "wattage": 0.25,
            },
            "spiceModel": {"type": "R", "value": 220},
        },
        # 21. LED Amarelo Semaforo FSM / Scanning (Z20: bb1:a28 anode -> bb1:-pwr-bot-28 cathode)
        {
            "id": "led-284",
            "type": "led",
            "value": 2,
            "unit": "V",
            "position": [_bb1_col_x(28), 0.09, 1.987],
            "rotation": [0, -4.712388980384686, 0],
            "locked": False,
            "leads": [
                {"name": "anode", "holeId": "bb1:a28"},
                {"name": "cathode", "holeId": "bb1:-pwr-bot-28"},
            ],
            "props": {
                "burnt": False,
                "clearBody": False,
                "forwardVoltage": 2,
                "ledColor": "#ffcc00",
                "maxIntensity": 1,
                "shape": "round",
                "size": 5,
            },
            "spiceModel": {"type": "D", "modelName": "LED"},
        },
        # 22. Resistor 220Ω LED Vermelho Semaforo FSM / Tamper (bb1:d30 <-> bb1:f30)
        {
            "id": "resistor-282",
            "type": "resistor",
            "value": 220,
            "unit": "Ω",
            "position": [_bb1_col_x(30), 0.351, 0.0],
            "rotation": [0, 1.5707963267948961, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "bb1:d30"},
                {"name": "lead2", "holeId": "bb1:f30"},
            ],
            "props": {
                "bodyColor": "#3b6ea5",
                "burnt": False,
                "tolerance": "5%",
                "wattage": 0.25,
            },
            "spiceModel": {"type": "R", "value": 220},
        },
        # 23. LED Vermelho Semaforo FSM / Tamper (bb1:a30 anode -> bb1:-pwr-bot-30 cathode)
        {
            "id": "led-283",
            "type": "led",
            "value": 2,
            "unit": "V",
            "position": [_bb1_col_x(30), 0.09, 1.987],
            "rotation": [0, -4.712388980384686, 0],
            "locked": False,
            "leads": [
                {"name": "anode", "holeId": "bb1:a30"},
                {"name": "cathode", "holeId": "bb1:-pwr-bot-30"},
            ],
            "props": {
                "burnt": False,
                "clearBody": False,
                "forwardVoltage": 2,
                "ledColor": "#ff2200",
                "maxIntensity": 1,
                "shape": "round",
                "size": 5,
            },
            "spiceModel": {"type": "D", "modelName": "LED"},
        },
        # ====================================================================
        # ZONA 3 — MINI PROTOBOARD 1 (`mn1:`, Frontal-Esquerda: DHT11 + TCRT5000 + FSR 402)
        # Nota: mn1:col k alinha verticalmente (Delta-X ~ 0) com Main col (k - 1)!
        # ====================================================================
        # 24. Sensor Digital Temp + Umidade DHT11 (Z10: mn1:h3, mn1:h4, mn1:h5 -> A11)
        {
            "id": "dht-103",
            "type": "dht",
            "value": 3.3,
            "unit": "V",
            "position": [_mn_col_x(0, 4), 0.09, 4.7],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": "mn1:h3"},
                {"name": "DATA", "holeId": "mn1:h4"},
                {"name": "GND", "holeId": "mn1:h5"},
            ],
            "props": {"variant": "DHT11"},
            "spiceModel": {"type": "X", "modelName": "DHT11"},
        },
        # 25. Sensor Optico Reflexivo TCRT5000 Seguidor de Linha (Z03: mn1:h9..mn1:h12 -> B14)
        {
            "id": "reflect-106",
            "type": "reflect",
            "value": 3.3,
            "unit": "V",
            "position": [_mn_col_x(0, 10.5), 0.09, 4.7],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": "mn1:h9"},
                {"name": "AOUT", "holeId": "mn1:h10"},
                {"name": "DOUT", "holeId": "mn1:h11"},
                {"name": "GND", "holeId": "mn1:h12"},
            ],
            "props": {"threshold": 0.45, "activeHigh": True},
            "spiceModel": {"type": "X", "modelName": "TCRT5000"},
        },
        # 26. Sensor de Forca/Carga FSR 402 (Z12 / Learn Arduino L15: mn1:c14 <-> mn1:c15)
        {
            "id": "fsr-107",
            "type": "fsr",
            "value": 2000,
            "unit": "Ω",
            "position": [_mn_col_x(0, 14.5), 0.09, 6.1],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "mn1:c14"},
                {"name": "lead2", "holeId": "mn1:c15"},
            ],
            "props": {"force": 5},
            "spiceModel": {"type": "R", "value": 2000},
        },
        # 27. Resistor Divisor 10k do FSR 402 (mn1:d15 <-> mn1:d16)
        {
            "id": "resistor-fsr-111",
            "type": "resistor",
            "value": 10000,
            "unit": "Ω",
            "position": [_mn_col_x(0, 15.5), 0.351, 5.9],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "lead1", "holeId": "mn1:d15"},
                {"name": "lead2", "holeId": "mn1:d16"},
            ],
            "props": {
                "tolerance": "5%",
                "wattage": 0.25,
                "bodyColor": "#c9a06b",
                "burnt": False,
            },
            "spiceModel": {"type": "R", "value": 10000},
        },
        # ====================================================================
        # ZONA 4 — MINI PROTOBOARD 2 (`mn2:`, Frontal-Central: Exclusiva para LCD 16x2 I2C sem Sobreposicao!)
        # Nota: mn2:col 1..6 alinha verticalmente com Main col 25..30!
        # ====================================================================
        # 28. Display Character LCD 16x2 I2C (Z12/Z15/Z16: mn2:h3..mn2:h6 -> B6=SCL, B7=SDA)
        {
            "id": "lcd-104",
            "type": "lcd",
            "value": 5,
            "unit": "V",
            "position": [_mn_col_x(1, 4.5), 0.09, 4.7],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "GND", "holeId": "mn2:h3"},
                {"name": "VCC", "holeId": "mn2:h4"},
                {"name": "SDA", "holeId": "mn2:h5"},
                {"name": "SCL", "holeId": "mn2:h6"},
            ],
            "props": {
                "size": "16x2",
                "lcdColor": "blue",
                "address": 39,
                "contrast": 1,
            },
            "spiceModel": {"type": "X", "modelName": "LCD"},
        },
        # ====================================================================
        # ZONA 5 — MINI PROTOBOARD 3 (`mn3:`, Frontal-Direita: TM1637 4-Digitos + Regulador AMS1117 + MOSFET Stage)
        # Nota: mn3:col k alinha 1-para-1 verticalmente (Delta-X = 0.009) com bb1:col (k + 13)!
        # ====================================================================
        # 29. Display 4-Digitos 7-Segmentos TM1637 (Z13/Z14/Z17/Z19: mn3:h2..mn3:h5 -> CLK=A2, DIO=A3)
        {
            "id": "tm1637-105",
            "type": "tm1637",
            "value": 5,
            "unit": "V",
            "position": [_mn_col_x(2, 3.5), 0.09, 4.7],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "CLK", "holeId": "mn3:h2"},
                {"name": "DIO", "holeId": "mn3:h3"},
                {"name": "VCC", "holeId": "mn3:h4"},
                {"name": "GND", "holeId": "mn3:h5"},
            ],
            "props": {
                "segColor": "#ff2a1a",
                "colon": True,
                "dots": False,
            },
            "spiceModel": {"type": "X", "modelName": "TM1637"},
        },
        # 30. Regulador LDO AMS1117-3.3 (Plano Sec 2: mn3:h9=IN, mn3:h10=GND, mn3:h11=OUT)
        {
            "id": "voltage_regulator-109",
            "type": "voltage_regulator",
            "value": 3.3,
            "unit": "V",
            "position": [_mn_col_x(2, 10), 0.09, 4.7],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "IN", "holeId": "mn3:h9"},
                {"name": "GND", "holeId": "mn3:h10"},
                {"name": "OUT", "holeId": "mn3:h11"},
            ],
            "props": {"model": "AMS1117"},
            "spiceModel": {"type": "X", "modelName": "AMS1117"},
        },
        # 31. Capacitor Eletrolitico de Desacoplamento 100 µF (mn3:f11=positive, mn3:f10=negative)
        {
            "id": "capacitor-110",
            "type": "capacitor",
            "value": 100,
            "unit": "µF",
            "position": [_mn_col_x(2, 10.5), 0.09, 5.0],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "positive", "holeId": "mn3:f11"},
                {"name": "negative", "holeId": "mn3:f10"},
            ],
            "props": {
                "capType": "electrolytic",
                "polarized": True,
                "voltage": 25,
            },
            "spiceModel": {"type": "C", "value": 0.0001},
        },
        # 32. Power Stage MOSFET Canal-N IRLZ44N (mn3:c13=GATE, mn3:c14=DRAIN, mn3:c15=SOURCE -> B15)
        {
            "id": "mosfet-320",
            "type": "mosfet",
            "value": 0,
            "unit": "",
            "position": [_mn_col_x(2, 14), 0.09, 6.1],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "GATE", "holeId": "mn3:c13"},
                {"name": "DRAIN", "holeId": "mn3:c14"},
                {"name": "SOURCE", "holeId": "mn3:c15"},
            ],
            "props": {"channel": "N", "model": "IRLZ44N"},
            "spiceModel": {"type": "M", "modelName": "IRLZ44N"},
        },
        # 33. Diodo Flyback 1N4001 (mn3:b14=anode no DRAIN, mn3:b16=cathode no +5V)
        {
            "id": "diode-330",
            "type": "diode",
            "value": 1,
            "unit": "V",
            "position": [_mn_col_x(2, 15), 0.351, 6.3],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "anode", "holeId": "mn3:b14"},
                {"name": "cathode", "holeId": "mn3:b16"},
            ],
            "props": {"model": "1N4001"},
            "spiceModel": {"type": "D", "modelName": "1N4001"},
        },
        # ====================================================================
        # ZONA 6 — BAIA TRASEIRA DE SENSORES ToF & TELEMETRIA I2C (z = -5.4, atras de `bb1:` com ampla folga)
        # ====================================================================
        # 34. Sensor Ultrassonico HC-SR04 ToF Redundante (traseiro-esquerdo de bb1, alinhado a bb1:col 4)
        {
            "id": "ultrasonic-317",
            "type": "ultrasonic",
            "value": 5,
            "unit": "V",
            "position": [6.0, 0.09, -5.4],
            "rotation": [0, 3.141592653589793, 0],
            "locked": False,
            "leads": [
                {"name": "ECHO", "holeId": None},
                {"name": "GND", "holeId": None},
                {"name": "TRIG", "holeId": None},
                {"name": "VCC", "holeId": None},
            ],
            "props": {"auto": False, "distanceCm": 18},
            "spiceModel": {"type": "X", "modelName": "HCSR04"},
        },
        # 35. IMU 6-DoF MPU6050 Anti-Tamper & Air Mouse (Z05: traseiro-central de bb1, alinhado a bb1:col 16)
        {
            "id": "mpu6050-318",
            "type": "mpu6050",
            "value": 104,
            "unit": "",
            "position": [9.1, 0.09, -5.4],
            "rotation": [0, 3.141592653589793, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": None},
                {"name": "GND", "holeId": None},
                {"name": "SCL", "holeId": None},
                {"name": "SDA", "holeId": None},
                {"name": "XDA", "holeId": None},
                {"name": "XCL", "holeId": None},
                {"name": "AD0", "holeId": None},
                {"name": "INT", "holeId": None},
            ],
            "props": {"pitch": 0, "roll": 0},
            "spiceModel": None,
        },
        # 36. Display 0.96" OLED SSD1306 I2C 128x64 (traseiro-direito de bb1, alinhado a bb1:col 28)
        {
            "id": "oled-325",
            "type": "oled",
            "value": 3.3,
            "unit": "V",
            "position": [12.1, 0.09, -5.4],
            "rotation": [0, 3.141592653589793, 0],
            "locked": False,
            "leads": [
                {"name": "GND", "holeId": None},
                {"name": "VCC", "holeId": None},
                {"name": "SCL", "holeId": None},
                {"name": "SDA", "holeId": None},
            ],
            "props": {"address": 60, "oledColor": "white"},
            "spiceModel": {"type": "X", "modelName": "SSD1306"},
        },
        # ====================================================================
        # ZONA 7 — BAIA FRONTAL DE ATUADORES MECATRONICOS (z = +9.4, a frente de `mn1..mn3` com ampla folga)
        # ====================================================================
        # 37. Servo SG90 Trava Mecatronica S-Curve (frontal-esquerdo a frente de mn1)
        {
            "id": "servo-35",
            "type": "servo",
            "value": 5,
            "unit": "V",
            "position": [-2.3, 0.09, 9.4],
            "rotation": [0, 1.5707963267948966, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": None},
                {"name": "GND", "holeId": None},
                {"name": "SIG", "holeId": None},
            ],
            "props": {"angle": 0},
            "spiceModel": {
                "type": "R",
                "value": 625,
                "params": {"description": "SG90 servo (idle draw)"},
            },
        },
        # 38. Barra de LEDs Enderecavel WS2812B NeoPixel 8x (frontal-central a frente de mn2)
        {
            "id": "neopixel-331",
            "type": "neopixel",
            "value": 5,
            "unit": "V",
            "position": [4.4, 0.09, 9.4],
            "rotation": [0, 3.141592653589793, 0],
            "locked": False,
            "leads": [
                {"name": "GND", "holeId": None},
                {"name": "VCC", "holeId": None},
                {"name": "DIN", "holeId": None},
                {"name": "DOUT", "holeId": None},
            ],
            "props": {"count": 8, "form": "stick", "wiring": "serpentine"},
            "spiceModel": {"type": "X", "modelName": "WS2812B"},
        },
        # 39. Modulo Rele SPDT 1-Ch Fail-Secure Lock (frontal-direito 1 a frente de mn3)
        {
            "id": "relay-108",
            "type": "relay",
            "value": 5,
            "unit": "V",
            "position": [9.1, 0.09, 9.4],
            "rotation": [0, 0, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": None},
                {"name": "GND", "holeId": None},
                {"name": "SIG", "holeId": None},
                {"name": "NC", "holeId": None},
                {"name": "COM", "holeId": None},
                {"name": "NO", "holeId": None},
            ],
            "props": {},
            "spiceModel": {"type": "X", "modelName": "RELAY_1CH"},
        },
        # 40. Atuador Motor DC / Solenoide da Fechadura (frontal-direito 2 a frente de mn3)
        {
            "id": "motor-323",
            "type": "motor",
            "value": 3,
            "unit": "V",
            "position": [12.2, 0.09, 9.4],
            "rotation": [0, 4.71238898038469, 0],
            "locked": False,
            "leads": [
                {"name": "VCC", "holeId": None},
                {"name": "GND", "holeId": None},
            ],
            "props": {"rpm": 200, "voltage": 3},
            "spiceModel": {
                "type": "R",
                "value": 10,
                "params": {"description": "DC Motor (simplified)"},
            },
        },
    ]

    jumpers: list[dict[str, Any]] = [
        # ====================================================================
        # 1. BARRAMENTO DE ALIMENTACAO & PONTES INTER-PROTOBOARDS (Main <-> bb1)
        # ====================================================================
        {
            "id": "jumper-pwr-3v3-main",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "mcu-10",
            "endHoleId": "+pwr-top-9",
            "locked": False,
        },
        {
            "id": "jumper-pwr-gnd-top-main",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "mcu-1",
            "endHoleId": "-pwr-top-1",
            "locked": False,
        },
        {
            "id": "jumper-pwr-gnd-bridge-main",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-top-30",
            "endHoleId": "-pwr-bot-30",
            "locked": False,
        },
        {
            "id": "jumper-pwr-5v-bot-main",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "mcu-2",
            "endHoleId": "+pwr-bot-1",
            "locked": False,
        },
        # Pontes horizontais curtas Main col 30 -> Extra 400 (`bb1:col 1`)
        {
            "id": "jumper-bridge-bb1-3v3-top",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-30",
            "endHoleId": "bb1:+pwr-top-1",
            "locked": False,
        },
        {
            "id": "jumper-bridge-bb1-gnd-top",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-top-26",
            "endHoleId": "bb1:-pwr-top-1",
            "locked": False,
        },
        {
            "id": "jumper-bridge-bb1-5v-bot",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "+pwr-bot-30",
            "endHoleId": "bb1:+pwr-bot-1",
            "locked": False,
        },
        {
            "id": "jumper-bridge-bb1-gnd-bot",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-bot-29",
            "endHoleId": "bb1:-pwr-bot-1",
            "locked": False,
        },
        # ====================================================================
        # 2. ZONA 1 — CONEXOES DA PROTOBOARD PRINCIPAL (100% Verticais por Coluna!)
        # ====================================================================
        {
            "id": "jumper-btn-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-28",
            "endHoleId": "j28",
            "locked": False,
        },
        {
            "id": "jumper-btn-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "j29",
            "endHoleId": "-pwr-top-29",
            "locked": False,
        },
        {
            "id": "jumper-btn-sig",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "mcu-27",
            "endHoleId": "j30",
            "locked": False,
        },
        {
            "id": "jumper-ult-div-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "a7",
            "endHoleId": "-pwr-bot-7",
            "locked": False,
        },
        {
            "id": "jumper-ult-echo-mcu",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "e5",
            "endHoleId": "mcu-5",
            "locked": False,
        },
        {
            "id": "jumper-ntc-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-top-21",
            "endHoleId": "j21",
            "locked": False,
        },
        {
            "id": "jumper-ntc-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-23",
            "endHoleId": "j23",
            "locked": False,
        },
        {
            "id": "jumper-ntc-sig",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-31",
            "endHoleId": "j22",
            "locked": False,
        },
        {
            "id": "jumper-pot-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "j13",
            "endHoleId": "-pwr-top-13",
            "locked": False,
        },
        {
            "id": "jumper-pot-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-15",
            "endHoleId": "j15",
            "locked": False,
        },
        {
            "id": "jumper-pot-sig",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-22",
            "endHoleId": "j14",
            "locked": False,
        },
        {
            "id": "jumper-tmp-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-17",
            "endHoleId": "j17",
            "locked": False,
        },
        {
            "id": "jumper-tmp-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-top-19",
            "endHoleId": "j19",
            "locked": False,
        },
        {
            "id": "jumper-tmp-sig",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-26",
            "endHoleId": "j18",
            "locked": False,
        },
        {
            "id": "jumper-ldr-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-top-25",
            "endHoleId": "j25",
            "locked": False,
        },
        {
            "id": "jumper-ldr-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-top-27",
            "endHoleId": "j27",
            "locked": False,
        },
        {
            "id": "jumper-ldr-sig",
            "color": "#ffee00",
            "endStyle": "precut",
            "startHoleId": "mcu-30",
            "endHoleId": "i26",
            "locked": False,
        },
        # ====================================================================
        # 3. ZONA 2 — PROTOBOARD EXTRA 400 (`bb1:`: Hub I2C col 1, 74HC595, LED Bar, Buzzer, Semaforo FSM)
        # ====================================================================
        # Hub I2C dedicado em bb1:col 1 (f1..j1 = SCL/B6, a1..e1 = SDA/B7)
        {
            "id": "jumper-i2c-hub-scl",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-8",
            "endHoleId": "bb1:f1",
            "locked": False,
        },
        {
            "id": "jumper-i2c-hub-sda",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-9",
            "endHoleId": "bb1:e1",
            "locked": False,
        },
        # 74HC595 (bb1:cols 3..10)
        {
            "id": "jumper-sr-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "bb1:+pwr-top-3",
            "endHoleId": "bb1:j3",
            "locked": False,
        },
        {
            "id": "jumper-sr-oe-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:-pwr-top-6",
            "endHoleId": "bb1:j6",
            "locked": False,
        },
        {
            "id": "jumper-sr-srclr-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "bb1:+pwr-top-9",
            "endHoleId": "bb1:j9",
            "locked": False,
        },
        {
            "id": "jumper-sr-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a10",
            "endHoleId": "bb1:-pwr-bot-10",
            "locked": False,
        },
        {
            "id": "jumper-sr-ser-mcu",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "mcu-25",
            "endHoleId": "bb1:j5",
            "locked": False,
        },
        {
            "id": "jumper-sr-rclk-mcu",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-23",
            "endHoleId": "bb1:j7",
            "locked": False,
        },
        {
            "id": "jumper-sr-srclk-mcu",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-24",
            "endHoleId": "bb1:j8",
            "locked": False,
        },
        # 74HC595 QA..QH -> LED Bar A1..A8 (bb1:cols 12..19) + Catodos K1..K8 -> GND
        {
            "id": "jumper-sr-qa-bar1",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:j4",
            "endHoleId": "bb1:j12",
            "locked": False,
        },
        {
            "id": "jumper-sr-qb-bar2",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c3",
            "endHoleId": "bb1:j13",
            "locked": False,
        },
        {
            "id": "jumper-sr-qc-bar3",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c4",
            "endHoleId": "bb1:j14",
            "locked": False,
        },
        {
            "id": "jumper-sr-qd-bar4",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c5",
            "endHoleId": "bb1:j15",
            "locked": False,
        },
        {
            "id": "jumper-sr-qe-bar5",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c6",
            "endHoleId": "bb1:j16",
            "locked": False,
        },
        {
            "id": "jumper-sr-qf-bar6",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c7",
            "endHoleId": "bb1:j17",
            "locked": False,
        },
        {
            "id": "jumper-sr-qg-bar7",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c8",
            "endHoleId": "bb1:j18",
            "locked": False,
        },
        {
            "id": "jumper-sr-qh-bar8",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:c9",
            "endHoleId": "bb1:j19",
            "locked": False,
        },
        {
            "id": "jumper-bar-k1-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a12",
            "endHoleId": "bb1:-pwr-bot-12",
            "locked": False,
        },
        {
            "id": "jumper-bar-k2-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a13",
            "endHoleId": "bb1:-pwr-bot-13",
            "locked": False,
        },
        {
            "id": "jumper-bar-k3-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a14",
            "endHoleId": "bb1:-pwr-bot-14",
            "locked": False,
        },
        {
            "id": "jumper-bar-k4-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a15",
            "endHoleId": "bb1:-pwr-bot-15",
            "locked": False,
        },
        {
            "id": "jumper-bar-k5-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a16",
            "endHoleId": "bb1:-pwr-bot-16",
            "locked": False,
        },
        {
            "id": "jumper-bar-k6-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a17",
            "endHoleId": "bb1:-pwr-bot-17",
            "locked": False,
        },
        {
            "id": "jumper-bar-k7-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a18",
            "endHoleId": "bb1:-pwr-bot-18",
            "locked": False,
        },
        {
            "id": "jumper-bar-k8-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a19",
            "endHoleId": "bb1:-pwr-bot-19",
            "locked": False,
        },
        # Buzzer Piezo Silencioso (bb1:cols 23..24 -> A12 / mcu-11)
        {
            "id": "jumper-buz-sig",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "mcu-11",
            "endHoleId": "bb1:e23",
            "locked": False,
        },
        {
            "id": "jumper-buz-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:a24",
            "endHoleId": "bb1:-pwr-bot-24",
            "locked": False,
        },
        # Semaforo FSM 3 LEDs (bb1:cols 26, 28, 30 -> B13, A8, B12)
        {
            "id": "jumper-led-ok",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-18",
            "endHoleId": "bb1:j26",
            "locked": False,
        },
        {
            "id": "jumper-led-yellow",
            "color": "#ffee00",
            "endStyle": "precut",
            "startHoleId": "mcu-15",
            "endHoleId": "bb1:j28",
            "locked": False,
        },
        {
            "id": "jumper-led-err",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "mcu-19",
            "endHoleId": "bb1:j30",
            "locked": False,
        },
        # ====================================================================
        # 4. ZONA 3 — MINI PROTOBOARD 1 (`mn1:`: Alinhamento Vertical 1:1 com Main col k-1!)
        # ====================================================================
        {
            "id": "jumper-dht-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-bot-2",
            "endHoleId": "mn1:j3",
            "locked": False,
        },
        {
            "id": "jumper-dht-data",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-12",
            "endHoleId": "mn1:j4",
            "locked": False,
        },
        {
            "id": "jumper-dht-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-bot-4",
            "endHoleId": "mn1:j5",
            "locked": False,
        },
        {
            "id": "jumper-tcrt-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-bot-8",
            "endHoleId": "mn1:j9",
            "locked": False,
        },
        {
            "id": "jumper-tcrt-dout",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-17",
            "endHoleId": "mn1:j11",
            "locked": False,
        },
        {
            "id": "jumper-tcrt-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-bot-11",
            "endHoleId": "mn1:j12",
            "locked": False,
        },
        {
            "id": "jumper-fsr-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-bot-13",
            "endHoleId": "mn1:e14",
            "locked": False,
        },
        {
            "id": "jumper-fsr-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-bot-15",
            "endHoleId": "mn1:e16",
            "locked": False,
        },
        # ====================================================================
        # 5. ZONA 4 — MINI PROTOBOARD 2 (`mn2:`: LCD 16x2 I2C Alinhado Verticalmente a Main col 27..28 & bb1:col 1)
        # ====================================================================
        {
            "id": "jumper-lcd-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "-pwr-bot-27",
            "endHoleId": "mn2:j3",
            "locked": False,
        },
        {
            "id": "jumper-lcd-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "+pwr-bot-28",
            "endHoleId": "mn2:j4",
            "locked": False,
        },
        {
            "id": "jumper-lcd-sda",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "bb1:c1",
            "endHoleId": "mn2:j5",
            "locked": False,
        },
        {
            "id": "jumper-lcd-scl",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "bb1:h1",
            "endHoleId": "mn2:j6",
            "locked": False,
        },
        # ====================================================================
        # 6. ZONA 5 — MINI PROTOBOARD 3 (`mn3:`: Alinhamento Vertical 1:1 com bb1:col k+13!)
        # ====================================================================
        {
            "id": "jumper-tm-clk",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mcu-29",
            "endHoleId": "mn3:j2",
            "locked": False,
        },
        {
            "id": "jumper-tm-dio",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mcu-28",
            "endHoleId": "mn3:j3",
            "locked": False,
        },
        {
            "id": "jumper-tm-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "bb1:+pwr-bot-17",
            "endHoleId": "mn3:j4",
            "locked": False,
        },
        {
            "id": "jumper-tm-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:-pwr-bot-18",
            "endHoleId": "mn3:j5",
            "locked": False,
        },
        {
            "id": "jumper-ldo-in-5v",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "bb1:+pwr-bot-22",
            "endHoleId": "mn3:j9",
            "locked": False,
        },
        {
            "id": "jumper-ldo-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:-pwr-bot-23",
            "endHoleId": "mn3:j10",
            "locked": False,
        },
        {
            "id": "jumper-mos-gate",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "mcu-16",
            "endHoleId": "mn3:e13",
            "locked": False,
        },
        {
            "id": "jumper-mos-src-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "bb1:-pwr-bot-28",
            "endHoleId": "mn3:a15",
            "locked": False,
        },
        {
            "id": "jumper-flyback-5v",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "bb1:+pwr-bot-29",
            "endHoleId": "mn3:a16",
            "locked": False,
        },
        # ====================================================================
        # 7. ZONAS 6 & 7 — BAIAS PERIMETRAIS EXTERNAS (100% Alinhadas Verticalmente!)
        # ====================================================================
        # HC-SR04 (Baia Traseira Esquerda de bb1, x=6.0 -> bb1:col 4)
        {
            "id": "jumper-ult-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "ultrasonic-317-VCC",
            "endHoleId": "bb1:+pwr-top-4",
            "locked": False,
        },
        {
            "id": "jumper-ult-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "ultrasonic-317-GND",
            "endHoleId": "bb1:-pwr-top-4",
            "locked": False,
        },
        {
            "id": "jumper-ult-trig",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "ultrasonic-317-TRIG",
            "endHoleId": "mcu-4",
            "locked": False,
        },
        {
            "id": "jumper-ult-echo-in",
            "color": "#ffee00",
            "endStyle": "precut",
            "startHoleId": "ultrasonic-317-ECHO",
            "endHoleId": "e3",
            "locked": False,
        },
        # MPU6050 (Baia Traseira Central de bb1, x=9.1 -> bb1:col 16 & Hub I2C bb1:col 1)
        {
            "id": "jumper-imu-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "mpu6050-318-VCC",
            "endHoleId": "bb1:+pwr-top-16",
            "locked": False,
        },
        {
            "id": "jumper-imu-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "mpu6050-318-GND",
            "endHoleId": "bb1:-pwr-top-16",
            "locked": False,
        },
        {
            "id": "jumper-imu-scl",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "mpu6050-318-SCL",
            "endHoleId": "bb1:i1",
            "locked": False,
        },
        {
            "id": "jumper-imu-sda",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "mpu6050-318-SDA",
            "endHoleId": "bb1:b1",
            "locked": False,
        },
        # OLED SSD1306 (Baia Traseira Direita de bb1, x=12.1 -> bb1:col 28 & Hub I2C bb1:col 1)
        {
            "id": "jumper-oled-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "oled-325-VCC",
            "endHoleId": "bb1:+pwr-top-28",
            "locked": False,
        },
        {
            "id": "jumper-oled-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "oled-325-GND",
            "endHoleId": "bb1:-pwr-top-28",
            "locked": False,
        },
        {
            "id": "jumper-oled-scl",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "oled-325-SCL",
            "endHoleId": "bb1:j1",
            "locked": False,
        },
        {
            "id": "jumper-oled-sda",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "oled-325-SDA",
            "endHoleId": "bb1:a1",
            "locked": False,
        },
        # Servo SG90 (Baia Frontal Esquerda, x=-2.3 -> Main col 6 / B5 mcu-6)
        {
            "id": "jumper-srv-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "servo-35-VCC",
            "endHoleId": "+pwr-bot-6",
            "locked": False,
        },
        {
            "id": "jumper-srv-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "servo-35-GND",
            "endHoleId": "-pwr-bot-6",
            "locked": False,
        },
        {
            "id": "jumper-srv-sig",
            "color": "#ffee00",
            "endStyle": "precut",
            "startHoleId": "servo-35-SIG",
            "endHoleId": "mcu-6",
            "locked": False,
        },
        # WS2812B NeoPixel 8x (Baia Frontal Central, x=4.4 -> Main col 29 / B8 mcu-14)
        {
            "id": "jumper-np-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "neopixel-331-VCC",
            "endHoleId": "+pwr-bot-29",
            "locked": False,
        },
        {
            "id": "jumper-np-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "neopixel-331-GND",
            "endHoleId": "-pwr-bot-29",
            "locked": False,
        },
        {
            "id": "jumper-np-din",
            "color": "#ff9900",
            "endStyle": "precut",
            "startHoleId": "neopixel-331-DIN",
            "endHoleId": "mcu-14",
            "locked": False,
        },
        # Modulo Rele SPDT Fail-Secure (Baia Frontal Direita 1, x=9.1 -> bb1:col 16 / A15 mcu-3)
        {
            "id": "jumper-rel-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "relay-108-VCC",
            "endHoleId": "bb1:+pwr-bot-16",
            "locked": False,
        },
        {
            "id": "jumper-rel-gnd",
            "color": "#111111",
            "endStyle": "precut",
            "startHoleId": "relay-108-GND",
            "endHoleId": "bb1:-pwr-bot-16",
            "locked": False,
        },
        {
            "id": "jumper-rel-sig",
            "color": "#33cc33",
            "endStyle": "precut",
            "startHoleId": "relay-108-SIG",
            "endHoleId": "mcu-3",
            "locked": False,
        },
        # Motor DC / Solenoide da Fechadura (Baia Frontal Direita 2, x=12.2 -> Dreno MOSFET mn3:e14 & +5V mn3:e16)
        {
            "id": "jumper-mot-vcc",
            "color": "#e63300",
            "endStyle": "precut",
            "startHoleId": "motor-323-VCC",
            "endHoleId": "mn3:e16",
            "locked": False,
        },
        {
            "id": "jumper-mot-drain",
            "color": "#0088ff",
            "endStyle": "precut",
            "startHoleId": "motor-323-GND",
            "endHoleId": "mn3:e14",
            "locked": False,
        },
    ]

    gpio = {
        "A0": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "A1": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "A2": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A3": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A4": {"enabled": True, "mode": "LOW", "direction": "input", "pull": "down"},
        "A5": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "A6": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A7": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A8": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A11": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "A12": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "A15": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B0": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B1": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "B3": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B4": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "B5": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B6": {"enabled": True, "mode": "HIGH", "direction": "output", "pull": None},
        "B7": {"enabled": True, "mode": "HIGH", "direction": "output", "pull": None},
        "B8": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B12": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B13": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
        "B14": {"enabled": True, "mode": "LOW", "direction": "input", "pull": None},
        "B15": {"enabled": True, "mode": "LOW", "direction": "output", "pull": None},
    }

    notes = (
        "# AEGIS-MECH S3 v3.0 Ultra — Bancada 3D Multi-Protoboard (5 Protoboards · 7 Zonas · 40 Componentes · 99 Jumpers)\n"
        "- **Zona 1 (Protoboard Principal 400 pts)**: Botao Trigger `A4` + Pull-down `10k`, Divisor HC-SR04 `1.2k/2.2k` (`B4`), Termistor `NTC 10k B3950` + Divisor `10k` (`A0`), Potenciometro ToF (`B1`), `TMP36` Bulbo Seco (`A5`), `LDR` 2FA + Divisor `10k` (`A1`).\n"
        "- **Zona 2 (Protoboard Extra 400 pts `bb1:`)**: Hub I2C (`col 1`), Registrador `74HC595` (`A6/A7/B0`) acionando `LED Bar 10-Segmentos`, `Buzzer Piezo` 100% silencioso em regime (`A12`), Semaforo FSM 3 LEDs (`Verde B13`, `Amarelo A8`, `Vermelho B12`) + 3x `220Ω`.\n"
        "- **Zona 3 (Mini Protoboard 1 `mn1:`)**: `DHT11` Digital (`A11`), Sensor Optico `TCRT5000` (`B14`), Sensor de Forca `FSR 402` + Divisor `10k`.\n"
        "- **Zona 4 (Mini Protoboard 2 `mn2:`)**: `Character LCD 16x2 I2C` (`0x27`) dedicado sem sobreposicao.\n"
        "- **Zona 5 (Mini Protoboard 3 `mn3:`)**: Display 4-Digitos 7-Segmentos `TM1637` (`CLK=A2, DIO=A3`) + Regulador LDO `AMS1117-3.3` + Capacitor `100µF` + Power Stage MOSFET `IRLZ44N` (`B15`) + Diodo Flyback `1N4001`.\n"
        "- **Zonas 6 & 7 (Baias Perimetrais Externas sem Sobreposicao)**: `HC-SR04` (`B3/B4`), `MPU6050` (`I2C`), `0.96\" OLED SSD1306` (`I2C`), `Servo SG90` (`B5`), `WS2812B NeoPixel 8x` (`B8`), `Modulo Rele SPDT` (`A15`), `Motor DC / Solenoide`."
    )

    workspace = {
        "dockOpen": False,
        "blocksW": 460,
        "codeView": True,
        "codeLang": "py",
        "consoleOpen": True,
        "notesOpen": False,
        "slowMode": False,
        "highlightEnabled": True,
        "scope": {
            "open": False,
            "mode": "gpio",
            "channels": ["B5", "B15", "B1", "A5"],
            "timebase": 2000,
            "xy": False,
        },
        "display": {
            "tog-grid": True,
            "tog-shadows": True,
            "tog-perf": False,
            "tog-labels": True,
            "tog-lockhl": False,
            "tog-autosnap": False,
            "tog-autosim": False,
            "tog-nodeids": False,
            "tog-pininfo": True,
            "tog-forcemanual": False,
        },
        "sections": ["view-section"],
        "font": "share",
        "vcc": "0",
        "theme": "light",
        "autoRun": True,
        "boardColor": "#f4f1ea",
        "extraLayout": "side",
        "jumperStyle": "precut",
        "view": {
            "pos": [3.5, 25.0, 13.5],
            "target": [3.2, -0.5, 2.2],
        },
        "env": {"light": 65, "temp": 24, "humidity": 45},
        "settings": {
            "hdr": "/hdr/studio_small_01_1k.hdr",
            "casing": True,
            "snap": 0.4,
        },
    }

    return {
        "version": "2.1",
        "components": components,
        "jumpers": jumpers,
        "gpio": gpio,
        "python": AEGIS_MICROPYTHON_CODE.strip(),
        "notes": notes,
        "workspace": workspace,
        "multi_mcu_catalog": MULTI_MCU_ARCHITECTURE_CATALOG,
    }


def encode_xplora3d_share_url(project: dict[str, Any]) -> str:
    serializable = {k: v for k, v in project.items() if k != "multi_mcu_catalog"}
    raw_bytes = json.dumps(serializable, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    compressed = zlib.compress(raw_bytes, level=9)
    b64u = base64.urlsafe_b64encode(compressed).decode("ascii").rstrip("=")
    return f"{XPLORA3D_BASE_URL}{b64u}"


def decode_xplora3d_share_url(url_or_hash: str) -> dict[str, Any]:
    token = url_or_hash.split("#p=")[-1].strip()
    pad = "=" * ((4 - len(token) % 4) % 4)
    compressed = base64.urlsafe_b64decode(token + pad)
    raw = zlib.decompress(compressed).decode("utf-8")
    return json.loads(raw)


def audit_xplora3d_project(project: dict[str, Any]) -> dict[str, Any]:
    findings: list[str] = []

    if project.get("version") != "2.1":
        findings.append(f"Versao invalida: {project.get('version')}")

    py_code = project.get("python", "")
    try:
        ast.parse(py_code)
    except SyntaxError as exc:
        findings.append(f"Erro de sintaxe MicroPython: {exc}")

    if "buz_pin.value(tick" in py_code:
        findings.append("Buzzer acionado em loop continuo por tick (deve ser silencioso em regime).")

    occupied_breadboard_holes: dict[str, str] = {}
    board_types: set[str] = set()
    for comp in project.get("components", []):
        cid = comp["id"]
        ctype = comp.get("type", "")
        if ctype in {"breadboard_400", "breadboard_mini"}:
            board_types.add(cid)
        for lead in comp.get("leads", []):
            lname = lead["name"]
            hid = lead.get("holeId")
            if hid is not None:
                if hid in occupied_breadboard_holes:
                    findings.append(
                        f"Colisao de furo na protoboard: {hid} usado por {occupied_breadboard_holes[hid]} e {cid}:{lname}"
                    )
                occupied_breadboard_holes[hid] = f"{cid}:{lname}"

    if len(board_types) < 4:
        findings.append(f"Esperadas 4 protoboards extras acopladas (bb1 + mn1..mn3), encontradas: {sorted(board_types)}")

    pwr_pos_nodes = {"mcu-2", "mcu-10", "mcu-21"}
    pwr_gnd_nodes = {"mcu-1", "mcu-7", "mcu-13", "mcu-20", "mcu-32"}

    for jmp in project.get("jumpers", []):
        jid = jmp["id"]
        s_id = jmp["startHoleId"]
        e_id = jmp["endHoleId"]
        if s_id == e_id:
            findings.append(f"Jumper degenerado {jid}: {s_id} -> {e_id}")

        s_bare = s_id.split(":")[-1]
        e_bare = e_id.split(":")[-1]
        s_is_pos = s_id in pwr_pos_nodes or s_bare.startswith("+pwr-")
        e_is_pos = e_id in pwr_pos_nodes or e_bare.startswith("+pwr-")
        s_is_gnd = s_id in pwr_gnd_nodes or s_bare.startswith("-pwr-")
        e_is_gnd = e_id in pwr_gnd_nodes or e_bare.startswith("-pwr-")

        if (s_is_pos and e_is_gnd) or (s_is_gnd and e_is_pos):
            findings.append(f"Curto-circuito direto VCC-GND no jumper {jid}: {s_id} <-> {e_id}")

    return {
        "passed": len(findings) == 0,
        "component_count": len(project.get("components", [])),
        "jumper_count": len(project.get("jumpers", [])),
        "extra_breadboards_count": len(board_types),
        "total_breadboards_count": 1 + len(board_types),
        "gpio_active_count": len(project.get("gpio", {})),
        "findings": findings,
    }


def export_aegis_xplora3d_bundle(
    output_dir: Path | str | None = None,
    stage_upload_path: Path | str | None = None,
) -> dict[str, Any]:
    repo_root = Path(__file__).resolve().parents[2]
    out_dir = Path(output_dir) if output_dir else (repo_root / "docs" / "xplora3d")
    out_dir.mkdir(parents=True, exist_ok=True)

    project = build_aegis_xplora3d_project()
    audit = audit_xplora3d_project(project)
    if not audit["passed"]:
        raise ValueError(f"Falha na auditoria do projeto Xplora3D: {audit['findings']}")

    share_url = encode_xplora3d_share_url(project)
    x3d_path = out_dir / "aegis_mech_s3_v3.x3d"
    url_path = out_dir / "aegis_mech_s3_v3_share_url.txt"
    readme_path = out_dir / "README_XPLORA3D.md"

    x3d_path.write_text(json.dumps(project, indent=2, ensure_ascii=False), encoding="utf-8")
    url_path.write_text(share_url + "\n", encoding="utf-8")

    mcu_rows = "\n".join(
        f"| **{item['family']}** | `{item['board_id']}` | {item['role']} | `{item['firmware']}` |"
        for item in MULTI_MCU_ARCHITECTURE_CATALOG
    )

    readme_md = f"""# Gemeo Digital 3D Multi-Protoboard — AEGIS-MECH S3 v3.0 Ultra (`Xplora3D`)

Este diretorio contem o circuito 3D interativo completo (`version 2.1`) organizado em **5 Protoboards Acopladas (1.110 furos em 7 Zonas Funcionais)** e o firmware MicroPython embarcado do **AEGIS-MECH S3 v3.0 Ultra** (integrando os **20 repositorios `.zip`**) para execucao imediata no **Xplora3D** (`https://xplora.phinalabs.com/3d/#`).

## Estatisticas Auditadas
- **Protoboards Acopladas**: `{audit['total_breadboards_count']}` (`Main 400 pts` + `Extra 400 pts bb1:` + `3x Mini Protoboards 170 pts mn1:/mn2:/mn3:`)
- **Componentes 3D Montados**: `{audit['component_count']}` distribuidos em 7 zonas funcionais com zero sobreposicao
- **Jumpers Roteados (`Pre-Cut` Alinhados Verticalmente)**: `{audit['jumper_count']}` (0 curtos-circuitos, 0 colisoes de furo)
- **Portas GPIO/ADC/PWM/I2C Ativas**: `{audit['gpio_active_count']}/24`
- **Auditoria Eletrica**: `PASSED = {audit['passed']}`

## Matriz Multi-Microcontrolador Integrada no Projeto (`Arduinos, ESP32, ESP32-S3, ESP32-C3, ESP8266, STM32`)

| Familia / Arquitetura | Board ID | Papel no Ecossistema AEGIS-MECH S3 v3.0 | Firmware / Circuito |
| :--- | :--- | :--- | :--- |
{mcu_rows}

## Arquivos Gerados
- [`aegis_mech_s3_v3.x3d`](./aegis_mech_s3_v3.x3d) — Arquivo nativo para `File -> Open...` (`Ctrl+O`) no Xplora3D.
- [`aegis_mech_s3_v3_share_url.txt`](./aegis_mech_s3_v3_share_url.txt) — Link completo `#p=` autocontido (5 protoboards + 40 componentes + 99 fios + codigo MicroPython + notas).
"""
    readme_path.write_text(readme_md, encoding="utf-8")

    if stage_upload_path is not None:
        stage_path = Path(stage_upload_path)
        stage_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(x3d_path, stage_path)

    return {
        "x3d_path": str(x3d_path),
        "url_path": str(url_path),
        "readme_path": str(readme_path),
        "share_url": share_url,
        "audit": audit,
    }


if __name__ == "__main__":
    res = export_aegis_xplora3d_bundle()
    print(
        json.dumps(
            {
                "x3d_path": res["x3d_path"],
                "share_url_len": len(res["share_url"]),
                "audit": res["audit"],
            },
            indent=2,
        )
    )

