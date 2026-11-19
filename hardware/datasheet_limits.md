# Limites Absolutos, Margens Térmicas e Elétricas (`v3.0`)

| Componente | CI / Módulo | Tensão Nominal | V_abs_max | Corrente Pico | Regra Crítica de Projeto (`v3.0`) |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **MCU** | `ESP32-S3-N16R8` | `3,30 V` | `3,60 V` | `580 mA` | Flash **`QIO`** + PSRAM **`OPI`** (`qio_opi`). Pinos `26..37` e strapping `0,3,45,46` livres. Clamp `BAT54S` em ADCs. |
| **ToF Laser** | `VL53L1X` (2×) | `2,8–3,3 V` | `3,60 V` | `40 mA` | Reset sequencial via `XSHUT` (`0x29 -> 0x30/0x31`); Short Mode `20 ms`; ROI `4×4` SPAD; cone mecânico $\ge 30^\circ$. |
| **Ultrassom** | `HC-SR04` (2×) | `5,00 V` | `5,50 V` | `15 mA` | **`ECHO` 5V -> divisor `1,2k/2,2k` (`3,235 V`)**. Ping-Pong `60 ms/eixo`, timeout hardware **`38 ms`**, glitch `<3 µs`. |
| **Termômetro** | `DS18B20` | `3,30 V` | `5,50 V` | `1,5 mA` | Pull-up `4,7 kΩ`; conversão assíncrona (`setWaitForConversion(false)`); compensa $c(T, RH)$ via Magnus-Tetens. |
| **IMU** | `MPU6050` | `3,30 V` | `3,60 V` | `5 mA` | `0x68`; calibração `500` amostras checando variância nos **3 eixos** ($\le 2{,}0$); fallback NVS. |
| **Encoder** | `AS5600` | `3,30 V` | `3,60 V` | `15 mA` | `0x36`; checa bit `MD` (`STATUS 0x0B & 0x20`); fator cremalheira `m=1,25 Z=16`: **`0,17453 mm/deg`**. |
| **Monitor V/I** | `INA219` + `0,1 Ω` | `3,30 V` (bus `26 V`) | `26 V` | `3,27 A` | Ligação **Kelvin 4-fios**; `Cal = 4096 (0x1000)` (`100 µA/bit`); `P_max shunt = 0,90 W < 2,0 W`. |
| **Ponte-H** | `DRV8871DDAR` | `6,5–45 V` | `45 V` | `2,133 A` | `R_ILIM = 30,0 kΩ 1%` ($64\text{kV}/R$); pull-downs `10 kΩ` em `IN1/IN2`; PWM Slow-Decay `20 kHz`. `L298N/TB6612` banidos. |
| **Solenoide** | `SOL1 12V/0,7A` | `12 V` (min `7 V`) | `15 V` | `0,8 A` | `PC817` (`330 Ω`) + `IRLZ44N` (Zener `12 V` no gate) + **`1N5819` flyback** junto à bobina ($V = -L\,di/dt$). |
| **Buck 5V** | `MP1584EN` | `7,0–28 V → 5,00 V` | `28 V` | `3,0 A` | `C_BST = 100 nF`, `R_FREQ = 100 kΩ (850 kHz)`, `L1 = 4,7 µH (Isat >= 4,5 A)`, `R_FB = 52,3k/10k` Kelvin longe de `SW`. |
| **DCDB/ATS/E-Stop** | `F1 + SMBJ15A + SW_ESTOP` | `7,4–12,0 V` | `15 V` | `5,0 A` | Fusível/PPTC `5 A`; `BAT54S` nos ADCs; cogumelo NF corta `+12V_VM_SW` e avisa `GPIO 42`. |
