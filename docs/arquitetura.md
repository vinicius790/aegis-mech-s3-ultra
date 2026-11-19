# Arquitetura Técnica do Sistema AEGIS-MECH S3 (`v3.0`)

O **AEGIS-MECH S3** é uma plataforma mecatrônica de alta segurança e controle autônomo com Inteligência Artificial multimodal embarcada em microcontrolador **ESP32-S3-WROOM-1-N16R8** (Dual Xtensa LX7 @ 240 MHz + SIMD PIE, `16 MB` Flash Quad SPI + `8 MB` Octal PSRAM — modo `qio_opi`).

Consulte a especificação integral em [Plano de desenvolvimento.md](../../Plano%20de%20desenvolvimento.md).

---

## 1. Topologia de Hardware e Sensoriamento Redundante

```mermaid
flowchart TD
    subgraph PWR["Entrada Protegida DCDB + ATS + E-Stop Hardwired"]
        P1["Fonte 12V 5A -> Fusivel F1 5A + TVS SMBJ15A + SS54 (GPIO1 ADC)"]
        P2["UPS 2S 18650 BMS 10A + TP5100 + SS54 (GPIO2 ADC)"]
        P3["Chave Cogumelo E-STOP NF -> Corta +12V_VM_SW + PC817 -> GPIO42"]
        P4["Buck MP1584EN (850kHz, 5V) -> LDO TLV1117-33 (3.3V) | Star-Ground NT1"]
    end

    subgraph SENS["Sensoriamento Acustico-Optico, Inercial & Potencia"]
        S1["2x HC-SR04 (Divisor 1.2k/2.2k + RMT IDF4.4 Ping-Pong 60ms/eixo)"]
        S2["2x VL53L1X (I2C XSHUT 0x30/0x31, 20ms Short, ROI 4x4)"]
        S3["MPU6050 0x68 (3-axis calib + 100Hz Tamper) + AS5600 0x36 (0.17453 mm/deg)"]
        S4["INA219 0x40 (Shunt 0.1R Kelvin Cal=4096) + DS18B20 1-Wire (Magnus-Tetens)"]
    end

    subgraph MCU["ESP32-S3 Dual-Core FreeRTOS + esp_task_wdt"]
        Core1["Core 1 (100Hz): RMT, I2C Mutex+Recovery, EKF Joseph, PF32, LockController (Med-D + SCurve)"]
        Core0["Core 0 (50/30Hz): Tiny1DCNNBio 4879p + OOD + SecurityFSM + OLED/WS2812 + SerialCRC"]
    end

    subgraph ACT["Atuacao Eletromecanica"]
        A1["Ponte H DRV8871 (R_ILIM=30k -> 2.13A) + Cremalheira M1.25 Z=16"]
        A2["Opto PC817 + MOSFET IRLZ44N + Zener 12V + Flyback 1N5819 (Solenoide SOL1)"]
        A3["Buzzer Passivo BZ1 (2N2222) + Anel 12-LED WS2812B + OLED SSD1306"]
    end

    PWR --> SENS --> MCU --> ACT
```

---

## 2. Pipeline de Controle, Fusão e TinyML Bit-Exact

1. **Fusão Sensorial Bayesiana:** EKF 2D na **Forma de Joseph** com gating de Mahalanobis ($d_M^2 \le 6{,}63$), Filtro de Partículas com `xorshift32` + reamostragem sistemática ($N_{\text{eff}} < 16$), e compensação acústica termohigrométrica **Magnus-Tetens** $c(T, RH)$.
2. **Controle Híbrido Verificado Espectralmente:** Perfil quíntico C² `SCurveProfile`, controlador Fuzzy-PID `3×3` com **derivada exclusivamente na medição** (zero *derivative kick*), anti-windup *back-calculation*, feedforward `ActuatorNetMLP` calibrada em `data/actuator_logs/actuator_logs.csv`, guarda temporal `MAX_AGE = 150 ms` e estabilidade discreta validada por **Faddeev-LeVerrier + Durand-Kerner** (`python/aegis_ai/control_stability.py`).
3. **TinyML Bit-Exact:** `Tiny1DCNNBio` (`4.879` parâmetros após folding BN→Conv1d / `5.023` pré-fold), quantização simétrica estrita `[-127, +127]`, perda biométrica `SupCon` cosseno com `GroupKFold`, POST real no boot verificando **`20/20` vetores dourados** (`max |Δ logits| == 0`).

---

## 3. Referências Cruzadas
- [Protocolo Binário AEGIS-PROTO v1](protocolo_comunicacao.md)
- [Simuladores 3D Web & Bancadas (`Learn Arduino 3D`, `Xplora3D`, `Wokwi`)](simuladores_3d_web.md)
- [Plano de Testes e Matriz V&V](plano_de_testes.md)
- [Integração dos 20 Repositórios](INTEGRACAO_REPOSITORIOS.md)
