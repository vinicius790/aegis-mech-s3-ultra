# Lab 09 — Anti-Tamper `100 Hz` (`MPU6050`), OOD *Open-Set* & Persistência Segura na NVS

- **Origens Integradas:** `00EB28A4`, `D0D236D9`, `COMET-AIR-MOUSE`, [sensor_security_power.cpp](../../firmware/src/sensors/sensor_security_power.cpp), [data/tamper/tamper.csv](../../data/tamper/tamper.csv).
- **Apoio `Xplora3D`:** Conectar `MPU6050 Motion (I2C)` aos pinos `B6/B7` do `UNDA.Xplora` e validar variação de aceleração/giroscópio em tempo real no `Console` / `Scope`.

## Bugs Reais Auditados
1. `calibrate_gyro_500()` no `v2.0` testava variância apenas em `var_x`, ignorando vibrações em Y e Z.
2. `SecurityFSM::handle_tamper_lockdown` no `v2.0` expirava após `30 s` destravando sozinho!

## Roteiro Executivo de Bancada
1. Gravar [03_imu_2stage_calibrator.cpp](../../firmware/tools/bringup/03_imu_2stage_calibrator.cpp) e aplicar vibração exclusivamente no **eixo Y** no boot, confirmando rejeição e fallback para NVS.
2. Treinar `TamperNetIMU` (`797` params) sobre `tamper.csv` e `GestureOODAutoencoder` (`13.288` params, limiar percentil `98`).
3. Simular choque `> 3 G` e desligar/religar a energia confirmando retenção permanente da flag **`NVS_TAMPER_LATCH`** em [security_fsm.cpp](../../firmware/src/mechatronics/security_fsm.cpp).

## Quality Gate Numérico
- Latência de disparo `TAMPER_LOCKDOWN` `< 10 ms`; falso alarme em operação normal `< 0,5%`; retenção `100%` pós-*power-cycle*; rejeição OOD $\ge 98\%$.
