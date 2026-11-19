# Mapeamento Integral de Pinos (`v3.0`) — `ESP32-S3-WROOM-1-N16R8` (41 Pinos Físicos)

| Pino Módulo | GPIO / Sinal Oficial | Net `AEGIS v3.0` | Direção | Circuito de Interface / Regra Inviolável |
| :---: | :--- | :--- | :---: | :--- |
| `1, 2, 40, 41` | `GND / EPAD` | `AGND` | PWR | Desacoplado junto ao módulo; união estrela em `NT1` com `PGND`. |
| `2` | `3V3` | `+3V3_SYS` | PWR | Saída `TLV1117-33` + `10 µF X7R + 100 nF`. |
| `3` | `EN (CHIP_PU)` | `ESP_EN` | RST | Pull-up `10 kΩ -> 3V3` + `1 µF -> AGND` (`10 ms` RC) + botão `SW_RST`. |
| `39` | **`GPIO 1` (`ADC1_CH0`)** | `V_MAINS_SENSE` | Entrada ADC | Divisor `39,0 kΩ / 10,0 kΩ` + `100 nF` + clamp `BAT54S` (lê `12 V` pré-SS54). |
| `38` | **`GPIO 2` (`ADC1_CH1`)** | `V_UPS_BATT_SENSE` | Entrada ADC | Divisor `22,0 kΩ / 10,0 kΩ` + `100 nF` + clamp `BAT54S` (lê `2S 18650`). |
| `4` | `GPIO 4` | `US_X_TRIG` | Saída RMT | `TRIG` do `HC-SR04 X`. |
| `5` | `GPIO 5` | `US_X_ECHO_3V3` | Entrada RMT | **Divisor `1,2 kΩ / 2,2 kΩ`** do `ECHO X` (`3,235 V` máx). |
| `6` | `GPIO 6` | `US_Y_TRIG` | Saída RMT | `TRIG` do `HC-SR04 Y`. |
| `7` | `GPIO 7` | `US_Y_ECHO_3V3` | Entrada RMT | **Divisor `1,2 kΩ / 2,2 kΩ`** do `ECHO Y` (`3,235 V` máx). |
| `12` | `GPIO 8` | `I2C_SDA` | Bidirecional | Barramento `400 kHz` c/ pull-up `4,7 kΩ` (`0x30, 0x31, 0x36, 0x3C, 0x40, 0x68`). |
| `17` | `GPIO 9` | `I2C_SCL` | Saída | Barramento `400 kHz` c/ pull-up `4,7 kΩ` + `i2c_bus_recovery()`. |
| `18` | `GPIO 10` | `TOF_X_XSHUT` | Saída | Sequenciamento `VL53L1X X -> 0x30`. |
| `19` | `GPIO 11` | `TOF_Y_XSHUT` | Saída | Sequenciamento `VL53L1X Y -> 0x31`. |
| `20` | `GPIO 12` | `IMU_INT` | Entrada | Interrupção Wake-on-Motion `MPU6050`. *(Seguro no ESP32-S3!)* |
| `21` | `GPIO 13` | `DS18B20_DQ` | Bidirecional | `1-Wire` + pull-up `4,7 kΩ -> 3V3`. |
| `22` | `GPIO 14` | `MOTOR_IN1` | PWM LEDC | Pull-down **`10 kΩ -> GND`**; `DRV8871 IN1`. |
| `8` | `GPIO 15` | `MOTOR_IN2` | PWM LEDC | Pull-down **`10 kΩ -> GND`**; `DRV8871 IN2`. |
| `9` | `GPIO 16` | `SOLENOID_CTRL` | Saída | Pull-down `10 kΩ` + resistor `330 Ω -> PC817 (ISO1)`. |
| `10` | `GPIO 17` | `LIMIT_OPEN_SW` | Entrada | Pull-up `10 kΩ` + filtro RC `1 kΩ / 100 nF` -> Fim de curso `OPEN`. |
| `11` | `GPIO 18` | `LIMIT_CLOSE_SW` | Entrada | Pull-up `10 kΩ` + filtro RC `1 kΩ / 100 nF` -> Fim de curso `CLOSE`. |
| `13, 14` | `GPIO 19, 20` | `USB_D_N / USB_D_P` | USB-OTG | Reservado USB-C nativo (`90 Ω` dif. + `USBLC6-2SC6`). |
| `23` | `GPIO 21` | `BTN_MODE` | Entrada | Botão frontal de páginas OLED c/ `INPUT_PULLUP` + `100 nF`. |
| `31` | `GPIO 38` | `WS2812_RING` | Saída RMT | Resistor `330 Ω` série -> `DIN` Anel `WS2812B`. |
| `32` | `GPIO 39` | `BUZZER_PWM` | Saída LEDC | `1 kΩ` -> Base NPN `2N2222` (`BZ1`). |
| `33, 34` | **`GPIO 40, 41`** | `AUX_UART1_TX/RX` | UART1 Matrix | Remapeado para `ESP32-CAM 2FA`. |
| `35` | **`GPIO 42`** | `ESTOP_SENSE_IN` | Entrada (`PULLUP`) | Opto `ISO2 (PC817B)` após cogumelo NF (`HIGH` = E-Stop ativo!). |
| `36, 37` | `GPIO 44, 43` | `UART0_RX/TX` | Debug/Bridge | Console binário `AEGIS-PROTO v1` @ `921600 bps`. |
| `24` | **`GPIO 47`** | `AUX_SERVO_PWM` | PWM/Aux | Servo bandeja 2-DOF (`50 Hz`) ou `HX711_SCK` / `74HC595`. |
| `25` | `GPIO 48` | `ONBOARD_RGB` | Saída RMT | LED RGB onboard ESP32-S3. |
| `15, 16, 26, 27` | **`GPIO 0, 3, 45, 46`** | Strapping | **PROIBIDO** | Apenas botão `BOOT` em `GPIO 0`; deixar `3, 45, 46` desconectados. |
| `28..30` | **`GPIO 26..37`** | **Flash / PSRAM Octal** | **PROIBIDO** | Conectados internamente à memória do `N16R8` (validado em `static_assert`). |
