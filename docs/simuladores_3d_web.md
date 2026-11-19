# Bancadas Virtuais & Simulação 3D Web (`Learn Arduino 3D` + `Xplora3D` + `Wokwi CI`)

> **Documentação Verificada em Sessão Real (`18/11/2026`):** As capacidades, restrições, pinagens e listas de componentes abaixo foram validadas diretamente nas interfaces ao vivo das duas plataformas mencionadas pelo usuário.

---

## 1. Comparativo Real das Três Plataformas de Bancada

| Plataforma | Link | Hardware Simulado & Linguagem | Liberdade de Montagem | Aplicação Ideal no `AEGIS-MECH S3` |
| :--- | :--- | :--- | :--- | :--- |
| **Learn Arduino 3D** | [learn-arduino-red.vercel.app/pt/](https://learn-arduino-red.vercel.app/pt/) | **Arduino Uno (`ATmega328P`, `5 V`)**, Arduino C++ (somente leitura c/ parâmetros editáveis no *Inspetor de peças*) | **16 lições guiadas fixas** (sem edição livre de fios/protoboard) | Onboarding visual com fluxo animado de corrente, esquema 2D e parâmetros em `HC-SR04`, `TIP120 + Motor DC`, `TMP36`, `SG90`, `FSR`, `OLED SSD1306` e `Buzzer` (**Labs 01–04**). |
| **Xplora3D (`Phina Labs`)** | [xplora.phinalabs.com/3d/#](https://xplora.phinalabs.com/3d/#) | Placa **`UNDA.Xplora`** (`ESP32-S3` UI + MCU `STM32F411CE`), protoboard `400 pts`, **Blocks & MicroPython** (`from machine import Pin, ADC, PWM`) | **Circuitos customizados livres** com **70 componentes** + `Scope` + `Multimeter` + `Inspect (I)` + `.x3d` / `#p=...` | Montagem e depuração *smoke-free* de divisores (`5V→3,24V`), MOSFET **`IRLZ44N + 1N5819 Schottky`** com motor DC, astável **`LM358`**, `HC-SR04`, `MPU6050 I2C`, `NTC 10k B3950`, `74HC595`, `OLED 0.96"` e `WS2812B` (**Labs 01–09**). |
| **Wokwi CI (`diagram.json`)** | [docs/wokwi/diagram.json](wokwi/diagram.json) | `board-esp32-s3-devkitc-1`, `board-esp32-devkit-v1`, C++ PlatformIO/Arduino | **Netlist JSON completo & CLI** | Validação automática em CI do firmware C++ ESP32-S3 e auditoria pelo `circuit_netlist_auditor.py` (`Z13..Z18`). |

---

## 2. Mapeamento Exato das 16 Lições do **Learn Arduino 3D**

Na plataforma em português (`Básico · 16 lições`), o operador pode clicar em valores numéricos destacados no sketch para abrir o *Inspetor de peças*, ativar **Mostrar fluxo de corrente** e abrir **Mostrar esquema** nas lições diretamente alinhadas ao AEGIS:

| Lição `Learn Arduino 3D` | Hardware da Lição | Conceito Transferido para o `AEGIS-MECH S3` |
| :--- | :--- | :--- |
| `01 Conheça a placa` / `02 Pisque um LED` | Uno + LED + resistor `220 Ω` | Relação tensão/corrente limitadora em LEDs (contraprova de `contador-wokwi 0.22 Ω`). |
| `03 Aperte um botão` / `08 Faça um som` | Botão tátil + `10 kΩ` / Piezo passivo | Pull-up/down anti-flutuação (`SW1`, `BTN_MODE`) e tons em frequência (`BZ1`). |
| `07 Leia um sensor` (`#sensor`) & `16 Monte um projeto` | **`HC-SR04`** + semáforo LED | Trigger `10 µs`, eco `ECHO` (`5 V`) e detecção de proximidade (antes do RMT Ping-Pong em `3,3 V`). |
| `10 Controle um motor` | Motor DC + **`TIP120` Darlington** | Controle PWM unilateral sem retorno direto de corrente na MCU (evoluindo para `DRV8871` + `1N5819`). |
| `11 Sinta o calor` / `12 Mova um servo` / `15 Força` | `TMP36` / `SG90` / `FSR 402` | Aquisição térmica (`DS18B20`/`NTC`), PWM servo `50 Hz` (`2-DOF`) e força/peso (`HX711`). |

---

## 3. Guia Prático na Bancada 3D do **Xplora3D** (`70 Componentes` + MicroPython)

O **Xplora3D** disponibiliza na paleta `COMPONENTS (70)` itens diretamente equivalentes ao núcleo eletrônico do `AEGIS-MECH S3`: `Resistor`, `Capacitor`, `Inductor`, **`MOSFET (TO-220 IRLZ44N)`**, **`Diode (1N4001 / 1N5819 Schottky)`**, `Zener Diode`, `Transistor BJT`, `Op-Amp (LM358)`, `74HC595`, `DC Motor (5V/12V)`, `Servo`, **`Ultrasonic HC-SR04`**, **`MPU6050 Motion (I2C)`**, **`Thermistor (NTC 10k B3950)`**, `0.96" OLED`, **`NeoPixel (WS2812B)`**, `LM7805`, `LM7812`, `AMS1117-3.3`, `Multimeter` e `Scope Probe (P1-P8)`.

> **Nota Importante de Hardware Virtual:** Os pinos visíveis na placa virtual **`UNDA.Xplora`** seguem o mapa **`A0..A15 / B0..B15`** (`I2C: B6/B7`, `UART: A11/A12`, `SPI: B3/A15/B5/B4`), programados em **Blocks** ou **MicroPython** (enquanto componentes exclusivos da PCB final como `PC817` e `VL53L1X` devem ser validados no esquemático `KiCad 8` e em bancada real). Utilize sempre `View -> Inspect wiring (I)` antes de clicar em `Run Sim`!

### 3.1 Scripts MicroPython Prontos para Colar e Testar no `Xplora3D`

#### Experimento A — Segurança `HC-SR04` + Divisor `1,2 kΩ / 2,2 kΩ` + Ambiente (`Target`, `T`, `RH`)
Montagem na protoboard Xplora3D: alimentar `HC-SR04` no barramento `+5V`, ligar `TRIG` ao pino `B8`, ligar `ECHO` ao resistor `1.2 kΩ` → nó central (`B15` + `Multimeter`/`Scope P1`) → `2.2 kΩ` → `GND`. Usar `Environment -> Target` e conferir $V_{\text{nó}} \approx 3{,}235\,\text{V}$ no multímetro!

```python
from machine import Pin
from time import sleep_ms, sleep_us, ticks_us, ticks_diff

trig = Pin('B8', Pin.OUT)
echo = Pin('B15', Pin.IN)

def read_hcsr04_mm(temp_c=22.0):
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    t0 = ticks_us()
    while echo.value() == 0:
        if ticks_diff(ticks_us(), t0) > 20000:
            return None
    t_start = ticks_us()
    while echo.value() == 1:
        if ticks_diff(ticks_us(), t_start) > 38000:  # Timeout oficial 38 ms!
            return None
    dur_us = ticks_diff(ticks_us(), t_start)
    c_t = 331.3 * ((1.0 + temp_c / 273.15) ** 0.5)
    return (dur_us * c_t) / 2000.0

while True:
    d = read_hcsr04_mm(22.0)
    print("US Dist (mm):", "NAN (timeout)" if d is None else f"{d:.1f}")
    sleep_ms(60)  # Intervalo minimo anti-diafonia!
```

#### Experimento B — Soft-Start PWM em `MOSFET IRLZ44N + Diodo Flyback 1N5819 + DC Motor`
Montagem na protoboard Xplora3D: Gate do `MOSFET (IRLZ44N)` em `A5` via resistor `100 Ω` (+ pull-down `10 kΩ` para `GND`); Source em `GND`; Drain no `-` do `DC Motor`; **`Diode (1N5819 Schottky)` com cátodo no `+` (12V/5V) e ânodo no Drain**; `Scope P1` no Drain para observar ausência de pico indutivo destrutivo durante o PWM.

```python
from machine import Pin, PWM
from time import sleep_ms

pwm_motor = PWM(Pin('A5'), freq=2000, duty_u16=0)

def soft_ramp(target_pct, step_ms=25):
    for s in range(0, 21):
        u = s / 20.0
        # Perfil minimo-jerk suave (evita pico de corrente inrush)
        poly = (u ** 3) * (10.0 - 15.0 * u + 6.0 * (u ** 2))
        duty = int((target_pct / 100.0) * poly * 65535)
        pwm_motor.duty_u16(duty)
        sleep_ms(step_ms)

while True:
    soft_ramp(75)
    sleep_ms(1000)
    pwm_motor.duty_u16(0)
    sleep_ms(1000)
```
