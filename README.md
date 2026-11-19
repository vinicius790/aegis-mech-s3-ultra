# AEGIS-MECH S3 (`v3.0 Ultra`) — Plataforma Mecatrônica, Eletrônica & IA Embarcada

Sistema ciberfísico autônomo em **ESP32-S3-WROOM-1-N16R8** (`Dual LX7 240 MHz`, `16 MB QIO Flash + 8 MB OPI PSRAM`), sensoriamento bayesiano redundante (`2× HC-SR04 RMT Ping-Pong + 2× VL53L1X + DS18B20 Magnus-Tetens + MPU6050 3-eixos + AS5600 + INA219 Kelvin + ATS ADC + E-Stop NF`), controle híbrido discreto estável (`SCurve` quíntico + Medição-D sem *derivative kick* + `ActuatorNet` + verificador `Faddeev-LeVerrier`), TinyML bit-exact (`Tiny1DCNNBio` 4.879 params folded + OOD + TamperNet), Visão 2FA OpenCV real (`ArUco + HSV + DangerDetector IoU + MediaPipe/DogTracker`), Digital Twin vetorizado estilo NVIDIA Isaac Lab (`aegis_sim/`) e Cockpit SCADA Live SSE.

Consulte o plano mestre completo em [Plano de desenvolvimento.md](../Plano%20de%20desenvolvimento.md).

## Estrutura Principal
- `hardware/` — `BOM v3.0`, limites, pinout `41 pinos`, chicote `harness_wireviz.yml`, projeto `KiCad 8` 4 camadas e netlist auditável.
- `cad/` — `7` modelos `OpenSCAD v3.0` imprimíveis (`painel_sensores` `56 mm`, `arena_base` com cones `VL53L1X`, `trava_cremalheira` `m=1.25 Z=16 AS5600`, `caixa_cofre` c/ furo E-Stop Ø22 e `3× mounts`).
- `firmware/` — FreeRTOS Dual-Core (`espressif32@^6.9.0` IDF 4.4 `<driver/rmt.h>` + `qio_opi`), partições `16 MB`, POST real `20/20`, `AEGIS-PROTO v1` (`0x01..0x08`) e `tools/bringup/01..04`.
- `python/aegis_ai/` — IA Bit-Exact, `aegis_sim/` vetorizado, `control_stability.py` (`Faddeev-LeVerrier` + `Durand-Kerner`), `psychrometry.py` (`Magnus-Tetens`) e `vision_2fa/` completo OpenCV.
- `demo/` — `app_web.py` (SSE 30 Hz + 5 painéis), `circuit_netlist_auditor.py`, `diagnostic_agent.py` (ROSA Read-Only + AST Guard), `rerun_logger.py`, `lerobot_arm_bridge.py` e `serial_recorder.py`.
- `docs/` — **12 Laboratórios Executáveis (`lab_01..12.md`)**, especificação `protocolo_comunicacao.md`, integração real dos **20 ZIPs**, `wokwi/diagram.json`, `web_flasher/` (`boot_app0.bin @ 0xe000`) e [simuladores_3d_web.md](docs/simuladores_3d_web.md).

## Comandos Rápidos
```powershell
py -m pytest -v
py demo/circuit_netlist_auditor.py --netlist hardware/kicad/aegis_mech_s3_netlist.json --strict
py demo/circuit_netlist_auditor.py --netlist docs/wokwi/diagram.json --strict
py demo/diagnostic_agent.py
py demo/app_web.py
```
