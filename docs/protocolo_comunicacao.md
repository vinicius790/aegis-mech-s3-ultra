# Especificação Binária Integral — Protocolo `AEGIS-PROTO v1`

Taxa serial padrão: **`921600 bps`** (`8N1`), Little-Endian nos payloads (`#pragma pack(1)`), **`CRC-16/CCITT-FALSE`** em Big-Endian ao final de cada quadro.

---

## 1. Estrutura Geral do Quadro

| Offset | Campo | Tamanho | Descrição |
| :-: | :--- | :-: | :--- |
| `0..1` | `SYNC` | `2 B` | Constante fixa `0xAA55` (`0x55`, `0xAA` na linha UART LE). |
| `2` | `VERSION` | `1 B` | Versão do protocolo (`0x01`). **Início do cálculo do CRC.** |
| `3` | `MSG_TYPE` | `1 B` | Identificador da mensagem (`0x01..0x08`). |
| `4..5` | `SEQ_NUM` | `2 B` | Contador monotônico `uint16_t` de pacotes transmitidos. |
| `6..13` | `T_CAP_US` | `8 B` | Timestamp de captura hardware (`esp_timer_get_time()`). |
| `14` | `PAYLOAD_LEN` | `1 B` | Tamanho $N \in [0..255]$ do bloco de carga útil. |
| `15..15+N-1` | `PAYLOAD` | `N B` | Estrutura binária empacotada conforme tabela abaixo. |
| `15+N..16+N` | `CRC16_BE` | `2 B` | `CRC-16/CCITT-FALSE` cobrindo bytes `[2 .. 15+N-1]` (`poly=0x1021`, `init=0xFFFF`). Vetor `"123456789"` $\to$ **`0x29B1`**. |

---

## 2. Catálogo Completo de Mensagens (`0x01..0x08`)

| `MSG_TYPE` | Nome Canônico | Direção | Tamanho Payload | Campos Empacotados (`#pragma pack(1)`) |
| :-: | :--- | :-: | :-: | :--- |
| **`0x01`** | `SENSOR_50HZ` | MCU → Host | `38 B` | `us_x, us_y, tof_x, tof_y, kx, ky, kvx, kvy, temp_c (9×float)`, `mahalanobis_flags (u8)`, `particle_spread_mm (u8)` |
| **`0x02`** | `GESTURE_RESULT` | MCU → Host | `166 B` | `pred_class (u8)`, `logits[7] (7×i32)`, `margin_int32 (i32)`, `bio_cos_sim (f32)`, `ood_recon_mse (f32)`, `vec_120[120] (120×i8)`, `infer_us (u32)`, `accepted (u8)` |
| **`0x03`** | `ACTUATOR_100HZ` | MCU → Host | `22 B` | `target_mm, pos_mm, vel_mm_s, current_ma (4×f32)`, `pwm_permille (i16)`, `ff_actuatornet (i16)`, `fsm_state (u8)`, `fault_flags (u8)` |
| **`0x04`** | `TAMPER_ALERT` | MCU → Host | `24 B` | `tamper_class (u8)`, `score (f32)`, `peak_accel_g (f32)`, `peak_gyro_dps (f32)`, `rms_fft_10_40hz (f32)`, `latched_nvs (u8)`, `pad[6]` |
| **`0x05`** | `HIL_INJECT_FRAME` | Host → MCU | `24 B` | `fake_us_x_mm, fake_us_y_mm, fake_tof_x_mm, fake_tof_y_mm, fake_imu_g (5×f32)`, `override_mask (u32)` |
| **`0x06`** | `POST_BOOT_REPORT` | MCU → Host | `16 B` | `vectors_passed (u8)`, `vectors_total (u8)`, `i2c_ack_mask (u16)`, `gyro_var_max (f32)`, `flash_mode_ok (u8)`, `psram_kb (u16)`, `fw_crc32 (u32)` |
| **`0x07`** | `VISION_2FA_SYNC` | Host → MCU | `20 B` | `aruco_id (i32)`, `badge_color_hsv (u8)`, `yolo_person_count (u8)`, `tailgating_alert (u8)`, `pose_dist_mm (f32)`, `valid_token (u8)`, `nonce (u64)` |
| **`0x08`** | `HEALTH_EXTENDED` | MCU → Host | `28 B` | `v_mains_v, v_batt_v, v_bus_v, v_5v_est, temp_c, rh_pct (6×f32)`, `ats_source (u8)`, `fault_code (u8)`, `stack_hwm_bytes (u16)` |

Implementações sincronizadas: [firmware/src/protocol_frames.h](../firmware/src/protocol_frames.h), [firmware/src/comms/serial_crc_bridge.cpp](../firmware/src/comms/serial_crc_bridge.cpp) e [python/aegis_ai/contratos.py](../python/aegis_ai/contratos.py).
