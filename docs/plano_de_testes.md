# Plano Oficial de Testes, Verificação e Validação (`V&V Pipeline v3.0`)

Este documento formaliza os **7 Níveis de Verificação e Validação (V&V)** do projeto **AEGIS-MECH S3**, alinhando testes automatizados em Python, suíte C++ Nativa (`Unity`), auditoria estática de circuitos, simulação 3D e ensaios de bancada HIL.

## 1. Execução Automatizada em Linha de Comando

```powershell
# Nivel 0: Auditoria Estatica de Circuitos (AEGIS Netlist + Wokwi CI)
py demo/circuit_netlist_auditor.py --netlist hardware/kicad/aegis_mech_s3_netlist.json --strict
py demo/circuit_netlist_auditor.py --netlist docs/wokwi/diagram.json --strict

# Niveis 1, 2, 5 e 6 (Pytest: Sim-to-Real, TinyML, Faddeev-LeVerrier, Visao 2FA, Cockpit SSE, ROSA)
py -m pytest -v

# Niveis 2 e 4 (Quando PlatformIO CLI estiver instalado na estacao de compilacao)
pio test -e native
pio run -e esp32s3_n16r8
```

## 2. Cobertura Obrigatória por Camada

| Nível | Alvo Testado | Arquivo de Teste / Ferramenta | Critério Mínimo de Aprovação |
| :-: | :--- | :--- | :--- |
| **0** | Netlists KiCad / Wokwi & CAD `.scad` | `demo/circuit_netlist_auditor.py`, `tests/test_auditor_and_cad.py` | `passed == True`; 100% detecção em bugs `A37E8C2F`, `EAFA7E0D`, `0.22Ω`, `GPIO12`, `Exp12/16`. |
| **1** | Simulador Vetorizado `aegis_sim/` | `tests/test_aegis_sim_v3.py`, `tests/test_domain_rand_sim2sim.py` | `robust_eval_7_axes()` $\ge 98\%$; `Stewart6DOF FK` converge `< 1e-3 mm`; `DriveBam` reduz $V_{\text{eff}}$ sob carga. |
| **2** | Paridade `vx/vy`, BN Fold, SupCon, OOD | `tests/test_quantizacao_paridade.py`, `tests/test_tinyml_v3.py` | **`max |logits_QCNN - logits_Py| == 0`** em `20/20` vetores; `4.879 / 5.023` params; `SHA-256` real no manifesto. |
| **3** | Bring-Up Hardware `01..04` | `firmware/tools/bringup/01..04_*.cpp` + Osciloscópio | Ripple `< 50 mVpp`; I2C ACK completo; calibração IMU em 3 eixos. |
| **4** | RTOS, `driver/rmt.h`, `AEGIS-PROTO v1` | `tests/test_protocol_crc16.py`, `firmware/test/test_main.cpp` | CRC `"123456789" -> 0x29B1`; codecs `0x01..0x08`; boot `qio_opi` sem `GPIO 26..37`. |
| **5** | `Faddeev-LeVerrier` & `LockController` | `tests/test_control_stability.py`, `tests/test_actuator_net_and_pid.py` | Todos os 9 pontos Fuzzy com $\rho < 1{,}0$; zero *derivative kick*; `MAX_AGE = 150 ms`. |
| **6** | Visão 2FA OpenCV, Cockpit SSE & ROSA | `tests/test_vision_2fa.py`, `tests/test_cockpit_and_rosa_v3.py` | Rejeição 100% de ArUco P&B; alerta `DangerDetector IoU`; Cockpit SSE/POST ativo; ROSA sem escrita. |
