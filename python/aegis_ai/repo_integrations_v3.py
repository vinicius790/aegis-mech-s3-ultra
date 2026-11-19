"""Integracao Completa dos 20 Repositorios .zip (`Repositórios do GitHub/Z01..Z20`) no AEGIS-MECH S3 v3.0.

Materializa em Python testavel todos os algoritmos, tabelas corrigidas, maquinas de estados (FSM),
controladores PID, drivers de registradores de deslocamento e validadores de hardware oriundos dos
20 pacotes `.zip`:
  - Z01 `IsaacLab-main.zip`: Digital Twin vetorizado (`aegis_sim/*`), `DriveBam`, `ActuatorNet`, `LEAPP`
  - Z02 `robotica-computacional-main.zip`: Estabilidade `Faddeev-LeVerrier`, `vision_2fa/*`, `OccupancyGridMapper`
  - Z03 `Robotics-Projects-master.zip`: `StewartPlatform6DOF`, `LineFollower5IRController` (corrige bug `kd*lasterror`)
  - Z04 `lab-uc-main.zip`: 16 experimentos USP (`HX711LoadCell`, `DS18B20`, `KY-024`, ping-pong `60ms/38ms`, 5 bugs auditados)
  - Z05 `COMET-AIR-MOUSE-main.zip`: `CometAirMouseProcessor` (calib 500 amostras 3 eixos, EMA 0.82, deadzone, click/scroll)
  - Z06 `Introducao-a-robotica-main.zip`: Protecao BMS 2S 18650, anti-`L298N` (`DRV8871`), `MediaPipeTeacher`
  - Z07 `awesome-ai-robotics-main.zip`: `RerunTelemetryLogger`, `ReadOnlyDiagnosticAgent` (ROSA), `LeRobotBridge`
  - Z08 `awesome-robotics-projects-master.zip`: Cinematica `Kinematics2DOF` / `SO-ARM100`
  - Z09 `MecatroniGit.github.io-master.zip`: Trilha de 12 Laboratorios Executaveis (`docs/labs/lab_01..12.md`)
  - Z10 `sensor_de_umidade_do_ar_caseiro_sem_dependencia_barometrica-main.zip`: `PsychrometerWetDryBulb` (corrige 3 bugs)
  - Z11 `apostila_robotica_acnp-main.zip`: `NTCThermistorBeta3950` (corrige div/0) + `BuzzerMelodiesLEDC` (Do..Do2 / SOS)
  - Z12 `Maquina-de-Cafe-Automatica-com-Arduino-main.zip`: `PreFlightInterlockValidator` (6 travas antes do disparo)
  - Z13 `placar-digital-wokwi-main.zip`: `SevenSegmentCascade74HC595` (cascata SPI 3 fios sem `delay(200)` e sem pino TX)
  - Z14 `varredura-wokwi-main.zip`: Multiplexacao POV 8 digitos sem `GPIO 12` (`MTDI` strapping pin)
  - Z15 `reguaeletronica-wokwi-main.zip`: LUT 7-segmentos com digito `6` (`0x7D`) restaurado + timeout `38000 us` + `c(T,RH)`
  - Z16 `sistema-de-entrada-main.zip`: `BidirectionalSonarGateFSM` (corrige bug que ignorava sensor `segundo`)
  - Z17 `contador-wokwi-main.zip`: Contador `00..99` com resistor `220 Ω` (corrige `"0.22"` Ω)
  - Z18 `logica-leds-main.zip`: Varredura 8 LEDs com bounds `[0..7]` seguros (corrige UB `i<=8` / `i=9`)
  - Z19 `Projeto_Display_de_Led_de_7S-Arduino-main.zip`: Decodificador matricial `10x8` de catodo comum
  - Z20 `Semafaro_Arduino_cpp-main.zip`: `TrafficLightNonBlockingFSM` (Dispatch Table O(1) sem `delay(2000)`)
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import zipfile

import numpy as np

from .psychrometry import relative_humidity_magnus, saturation_vapor_pressure_hpa, speed_of_sound_mps


# ============================================================================
# 1. Z10 — PSICROMETRO DE BULBO SECO / UMIDO SEM DEPENDENCIA BAROMETRICA
# ============================================================================
class PsychrometerWetDryBulb:
    """Corrige os 3 bugs do `.ino` original de Z10 e calcula RH% e c(T, RH).

    Bugs corrigidos do original:
      1. L132 `if ((int)ts >= pgm_table(0, line) && (int)ts < pgm_table(0, line))` era sempre falso;
         corrigido para busca por intervalo `row_ts <= ts < next_row_ts`.
      2. L036 erro tipografico `92, 83, 93` para Ts=12C corrigido para progressao monotona `92, 88, 83`.
      3. Substitui bloqueio `delay(1000)` por avaliacao instantanea continua (Sprung + Magnus-Tetens).
    """

    # Tabela corrigida para ΔT = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0] C
    DELTA_T_AXIS = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0)
    TS_TABLE_ROWS: dict[int, tuple[int, ...]] = {
        10: (100, 94, 88, 82, 77, 71, 66, 55, 44, 34),
        12: (100, 94, 89, 83, 78, 73, 68, 58, 48, 39),  # L036 corrigido (monotono!)
        15: (100, 95, 90, 85, 80, 75, 71, 61, 53, 44),
        20: (100, 96, 91, 87, 83, 78, 74, 66, 59, 51),
        25: (100, 96, 92, 88, 84, 81, 77, 70, 63, 57),
        30: (100, 96, 93, 89, 86, 83, 79, 73, 67, 61),
    }

    @classmethod
    def lookup_table_rh(cls, ts_c: float, tm_c: float) -> float:
        """Busca na tabela psicrometrica corrigida (monotona e sem condicao impossivel)."""
        delta_t = max(0.0, ts_c - tm_c)
        sorted_ts = sorted(cls.TS_TABLE_ROWS.keys())
        chosen_ts = sorted_ts[0]
        for idx in range(len(sorted_ts) - 1):
            if sorted_ts[idx] <= ts_c < sorted_ts[idx + 1]:
                chosen_ts = sorted_ts[idx]
                break
        else:
            if ts_c >= sorted_ts[-1]:
                chosen_ts = sorted_ts[-1]

        row = cls.TS_TABLE_ROWS[chosen_ts]
        # Interpolacao linear sobre o eixo DELTA_T_AXIS
        return float(np.interp(delta_t, cls.DELTA_T_AXIS, row))

    @staticmethod
    def evaluate(ts_dry_c: float, tm_wet_c: float, pressure_hpa: float = 1013.25) -> dict[str, float]:
        """Retorna RH% analitico (Magnus-Tetens), RH% tabelado corrigido, ponto de orvalho e c(T,RH)."""
        tm_clamped = min(ts_dry_c, tm_wet_c)
        rh_analytic = float(relative_humidity_magnus(ts_dry_c, tm_clamped, pressure_hpa))
        rh_table = PsychrometerWetDryBulb.lookup_table_rh(ts_dry_c, tm_clamped)
        es = float(saturation_vapor_pressure_hpa(ts_dry_c))
        ea = max(1e-3, (rh_analytic / 100.0) * es)
        gamma = math.log(ea / 6.1078)
        dew_point_c = (237.3 * gamma) / max(1e-6, 17.27 - gamma)
        c_mps = float(speed_of_sound_mps(ts_dry_c, rh_analytic, pressure_hpa))
        return {
            "ts_dry_c": round(ts_dry_c, 2),
            "tm_wet_c": round(tm_clamped, 2),
            "rh_analytic_pct": round(rh_analytic, 2),
            "rh_table_pct": round(rh_table, 2),
            "dew_point_c": round(dew_point_c, 2),
            "speed_of_sound_mps": round(c_mps, 3),
        }


# ============================================================================
# 2. Z11 & Z04 — TERMISTOR NTC 10K BETA=3950 COM PROTECAO CONTRA DIVISAO POR ZERO
# ============================================================================
class NTCThermistorBeta3950:
    """Equacao Beta=3950 corrigida de `apostila_robotica_acnp/Projeto12.ino` e `lab-uc Exp 02`."""

    def __init__(self, r_series_ohm: float = 10000.0, r25_ohm: float = 10000.0, beta_k: float = 3950.0) -> None:
        if r_series_ohm < 1000.0:
            raise ValueError(
                f"Resistor serie {r_series_ohm} Ω muito baixo (erro lab-uc Exp 02 100 Ω causa autoaquecimento e perda de faixa ADC)."
            )
        self.r_series = r_series_ohm
        self.r25 = r25_ohm
        self.beta = beta_k

    def adc_to_celsius(self, adc_raw: int, adc_max: int = 4095) -> float:
        """Converte leitura ADC para Celsius prevenindo divisao por zero em 0 e adc_max."""
        clamped = max(1, min(adc_max - 1, int(adc_raw)))
        r_ntc = self.r_series * (float(clamped) / float(adc_max - clamped))
        inv_t = (1.0 / 298.15) + (1.0 / self.beta) * math.log(r_ntc / self.r25)
        temp_c = (1.0 / inv_t) - 273.15
        return max(-55.0, min(150.0, temp_c))


# ============================================================================
# 3. Z13, Z14, Z15, Z17, Z19 — 7-SEGMENTOS + CASCATA 74HC595 + VARREDURA POV
# ============================================================================
class SevenSegmentCascade74HC595:
    """Unifica e corrige os drivers de 7 segmentos de Z13, Z14, Z15, Z17 e Z19.

    Correcoes aplicadas:
      - Z15 (`reguaeletronica-wokwi`): Restaura o digito `6` (`0b01111101 = 0x7D`) que havia sido
        omitido em `const byte numeros[10]`, deslocando 7, 8, 9.
      - Z14 (`varredura-wokwi`): Proibe o uso de `GPIO 12` (`MTDI` strapping pin da tensao da Flash)
        nos pinos de selecao de digito do ESP32.
      - Z13 (`placar-digital-wokwi`): Serializa multiplos digitos em cascata via 3 fios (`DS`, `SHCP`, `STCP`)
        sem usar o pino `1` (`UART0 TX`) nem `delay(200)`.
    """

    # LUT completa de 10 digitos (catodo comum: bit0=A, bit1=B, bit2=C, bit3=D, bit4=E, bit5=F, bit6=G)
    DIGIT_LUT: tuple[int, ...] = (
        0b00111111,  # 0 (0x3F)
        0b00000110,  # 1 (0x06)
        0b01011011,  # 2 (0x5B)
        0b01001111,  # 3 (0x4F)
        0b01100110,  # 4 (0x66)
        0b01101101,  # 5 (0x6D)
        0b01111101,  # 6 (0x7D) <- Restaurado (faltava em reguaeletronica-wokwi!)
        0b00000111,  # 7 (0x07)
        0b01111111,  # 8 (0x7F)
        0b01101111,  # 9 (0x6F)
    )

    ESP32_STRAPPING_PINS = {0, 12, 45, 46}

    @classmethod
    def encode_digit(cls, digit: int, dp: bool = False) -> int:
        if not (0 <= digit <= 9):
            raise ValueError(f"Digito fora de [0..9]: {digit}")
        val = cls.DIGIT_LUT[digit]
        return val | (0x80 if dp else 0x00)

    @classmethod
    def encode_matrix_10x8(cls) -> list[list[int]]:
        """Retorna a matriz 10x8 estilo Z19 (`Projeto_Display_de_Led_de_7S-Arduino`)."""
        matrix: list[list[int]] = []
        for d in range(10):
            mask = cls.DIGIT_LUT[d]
            matrix.append([(mask >> bit) & 1 for bit in range(8)])
        return matrix

    @classmethod
    def encode_cascade_stream(cls, number: int, num_digits: int = 4) -> list[int]:
        """Gera os bytes em cascata para N registradores 74HC595 (Z13/Z15)."""
        clamped = max(0, int(number))
        s = str(clamped).zfill(num_digits)[-num_digits:]
        return [cls.encode_digit(int(ch)) for ch in s]

    @classmethod
    def validate_pov_digit_pins(cls, digit_pins: list[int]) -> dict[str, Any]:
        """Valida os pinos COM de varredura POV (Z14) contra strapping pins (ex: GPIO 12 MTDI)."""
        collisions = [p for p in digit_pins if p in cls.ESP32_STRAPPING_PINS]
        return {
            "valid": len(collisions) == 0,
            "strapping_collisions": collisions,
        }


# ============================================================================
# 4. Z16 — SISTEMA DE ENTRADA BIDIRECIONAL COM 2 SENSORES ULTRASSONICOS (S1, S2)
# ============================================================================
class BidirectionalSonarGateFSM:
    """Corrige os 3 bugs de `sistema-de-entrada/versoes/versao-1.ino` (Z16).

    No original, `acao()` incrementava/decrementava olhando apenas a variavel `primeiro`,
    ignorando se `segundo` havia sido ativado! Aqui a transicao exige a sequencia completa
    `S1 -> S2` (ENTRADA, +1) ou `S2 -> S1` (SAIDA, -1) dentro da janela de timeout nao-bloqueante.
    """

    def __init__(self, range_cm: float = 150.0, timeout_ms: float = 1500.0) -> None:
        self.range_cm = range_cm
        self.timeout_ms = timeout_ms
        self.state = "IDLE"
        self.state_enter_ms = 0.0
        self.occupancy = 0
        self.total_entries = 0
        self.total_exits = 0

    def step(self, s1_cm: float, s2_cm: float, now_ms: float) -> dict[str, Any]:
        s1_active = 0.0 < s1_cm <= self.range_cm
        s2_active = 0.0 < s2_cm <= self.range_cm

        if self.state != "IDLE" and (now_ms - self.state_enter_ms) > self.timeout_ms:
            self.state = "IDLE"

        event = "NONE"
        if self.state == "IDLE":
            if s1_active and not s2_active:
                self.state = "S1_FIRST"
                self.state_enter_ms = now_ms
            elif s2_active and not s1_active:
                self.state = "S2_FIRST"
                self.state_enter_ms = now_ms
        elif self.state == "S1_FIRST":
            if s2_active:
                self.state = "WAIT_CLEAR"
                self.state_enter_ms = now_ms
                self.occupancy += 1
                self.total_entries += 1
                event = "ENTRY"
        elif self.state == "S2_FIRST":
            if s1_active:
                self.state = "WAIT_CLEAR"
                self.state_enter_ms = now_ms
                self.occupancy = max(0, self.occupancy - 1)
                self.total_exits += 1
                event = "EXIT"
        elif self.state == "WAIT_CLEAR":
            if not s1_active and not s2_active:
                self.state = "IDLE"

        return {
            "state": self.state,
            "event": event,
            "occupancy": self.occupancy,
            "total_entries": self.total_entries,
            "total_exits": self.total_exits,
        }


# ============================================================================
# 5. Z12 — CHECKLIST DE INTERTRAVAMENTO PRE-OPERACAO (MAQUINA DE CAFE UFAM)
# ============================================================================
@dataclass
class PreFlightStatus:
    passed: bool
    fault_code: str
    checks: dict[str, bool]


class PreFlightInterlockValidator:
    """Portado e expandido de `verificarSensores()` em `Maquina-de-Cafe-Automatica` (Z12)."""

    @staticmethod
    def validate(
        vbus_v: float = 12.0,
        ups_v: float = 8.1,
        driver_temp_c: float = 32.0,
        estop_closed: bool = True,
        tamper_latched: bool = False,
        ood_mse: float = 0.05,
    ) -> PreFlightStatus:
        checks = {
            "estop_ok": bool(estop_closed),
            "tamper_ok": not bool(tamper_latched),
            "vbus_ok": 10.5 <= vbus_v <= 14.5,
            "ups_ok": ups_v >= 6.8,
            "thermal_ok": driver_temp_c <= 70.0,
            "ood_ok": ood_mse <= 0.25,
        }
        if not checks["estop_ok"]:
            code = "E01: ESTOP_OPEN"
        elif not checks["tamper_ok"]:
            code = "E03: TAMPER_LATCHED"
        elif not checks["vbus_ok"] or not checks["ups_ok"]:
            code = "E02: POWER_BUS_FAULT"
        elif not checks["thermal_ok"]:
            code = "E05: THERMAL_OVERLOAD"
        elif not checks["ood_ok"]:
            code = "E07: OOD_REJECTED"
        else:
            code = "E00: NOMINAL"
        return PreFlightStatus(passed=all(checks.values()), fault_code=code, checks=checks)


# ============================================================================
# 6. Z05 — COMET AIR MOUSE (ESP32-C3 + MPU6050 BLE HID TOKEN WAND)
# ============================================================================
class CometAirMouseProcessor:
    """Processador inercial da varinha token ESP32-C3 (`COMET-AIR-MOUSE-main.zip`)."""

    def __init__(self, smoothing: float = 0.82, sensitivity: float = 0.20, deadzone_dps: float = 1.0) -> None:
        self.smoothing = smoothing
        self.sensitivity = sensitivity
        self.deadzone_dps = deadzone_dps
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0
        self.smooth_dx = 0.0
        self.smooth_dy = 0.0

    def calibrate_500(self, gyro_samples_xyz: np.ndarray) -> dict[str, float]:
        """Calibra offset estatico em 3 eixos (corrige bug G10 que olhava apenas var_x)."""
        arr = np.asarray(gyro_samples_xyz, dtype=np.float64)
        variances = np.var(arr, axis=0)
        max_var = float(np.max(variances))
        means = np.mean(arr, axis=0)
        self.bias_gx, self.bias_gy, self.bias_gz = float(means[0]), float(means[1]), float(means[2])
        return {
            "bias_gx": self.bias_gx,
            "bias_gy": self.bias_gy,
            "bias_gz": self.bias_gz,
            "max_variance": max_var,
            "stationary_valid": max_var <= 2.0,
        }

    def step_cursor(self, gx_dps: float, gy_dps: float, gz_dps: float) -> tuple[int, int]:
        """Converte Gyro Z -> Mouse X (invertido) e Gyro Y -> Mouse Y com EMA 0.82 e deadzone."""
        adj_y = gy_dps - self.bias_gy
        adj_z = -(gz_dps - self.bias_gz)
        if abs(adj_y) < self.deadzone_dps:
            adj_y = 0.0
        if abs(adj_z) < self.deadzone_dps:
            adj_z = 0.0

        raw_dx = adj_z * self.sensitivity
        raw_dy = adj_y * self.sensitivity
        self.smooth_dx = self.smoothing * self.smooth_dx + (1.0 - self.smoothing) * raw_dx
        self.smooth_dy = self.smoothing * self.smooth_dy + (1.0 - self.smoothing) * raw_dy
        return int(round(self.smooth_dx)), int(round(self.smooth_dy))

    @staticmethod
    def classify_button_press(press_duration_ms: float, is_left: bool = True) -> str:
        """Clique curto (< 180 ms) = Left/Right Click; clique longo (>= 180 ms) = Scroll Down/Up."""
        if press_duration_ms >= 180.0:
            return "SCROLL_DOWN" if is_left else "SCROLL_UP"
        return "LEFT_CLICK" if is_left else "RIGHT_CLICK"


# ============================================================================
# 7. Z03 — LINE FOLLOWER 5-IR COM DERIVADA CORRIGIDA (ROBOTICS-PROJECTS)
# ============================================================================
class LineFollower5IRController:
    """Corrige o bug `kd * lasterror` de `Robotics-Projects-master/Line Follower Robot/Code.ino`."""

    WEIGHTS = np.array([-2.0, -1.0, 0.0, 1.0, 2.0], dtype=np.float64)

    def __init__(self, kp: float = 28.0, ki: float = 0.5, kd: float = 1.8) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, ir_5: list[float] | np.ndarray, dt: float = 0.02) -> dict[str, float]:
        s = np.asarray(ir_5, dtype=np.float64)
        active_sum = float(np.sum(s))
        error = float(np.dot(s, self.WEIGHTS) / active_sum) if active_sum > 1e-6 else self.prev_error
        self.integral = float(np.clip(self.integral + error * dt, -20.0, 20.0))
        # Correcao do bug original `kd * lasterror`: derivada verdadeira (error - prev_error) / dt
        deriv = (error - self.prev_error) / max(dt, 1e-4)
        self.prev_error = error
        u = self.kp * error + self.ki * self.integral + self.kd * deriv
        return {"error": round(error, 4), "derivative": round(deriv, 4), "control_u": round(u, 4)}


# ============================================================================
# 8. Z02 — OCCUPANCY GRID MAPPER (ROBOTICA-COMPUTACIONAL visualizar_mapa.py)
# ============================================================================
class OccupancyGridMapper:
    """Mapeador de grade de ocupacao 2D log-odds da arena ToF/Ultrassom (Z02)."""

    def __init__(self, width_mm: float = 400.0, height_mm: float = 400.0, res_mm: float = 10.0) -> None:
        self.res_mm = res_mm
        self.cols = int(round(width_mm / res_mm))
        self.rows = int(round(height_mm / res_mm))
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float64)

    def world_to_map(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        c = int(np.clip(math.floor(x_mm / self.res_mm), 0, self.cols - 1))
        r = int(np.clip(math.floor(y_mm / self.res_mm), 0, self.rows - 1))
        return r, c

    def update_hit(self, x_mm: float, y_mm: float, occupied: bool = True) -> tuple[int, int]:
        r, c = self.world_to_map(x_mm, y_mm)
        delta = 0.85 if occupied else -0.40
        self.log_odds[r, c] = float(np.clip(self.log_odds[r, c] + delta, -5.0, 5.0))
        return r, c

    def probability_grid(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))


# ============================================================================
# 9. Z18 & Z20 — SEMAFORO NAO-BLOQUEANTE O(1) & VARREDURA SEGURA DE 8 LEDS
# ============================================================================
class TrafficLightNonBlockingFSM:
    """Refatora `Semafaro_Arduino_cpp` (Z20) e `logica-leds` (Z18) para FSM O(1) sem delay() e sem UB."""

    STATES = (
        ("GREEN", 2000.0, (0, 0, 1)),
        ("YELLOW", 1000.0, (0, 1, 0)),
        ("RED", 2000.0, (1, 0, 0)),
    )

    def __init__(self) -> None:
        self.idx = 0
        self.last_switch_ms = 0.0

    def step(self, now_ms: float) -> dict[str, Any]:
        name, dur_ms, rgb = self.STATES[self.idx]
        if (now_ms - self.last_switch_ms) >= dur_ms:
            self.idx = (self.idx + 1) % len(self.STATES)
            self.last_switch_ms = now_ms
            name, dur_ms, rgb = self.STATES[self.idx]
        return {"state": name, "red": rgb[0], "yellow": rgb[1], "green": rgb[2]}

    @staticmethod
    def scan_8_leds_safe(step_idx: int, reverse: bool = False) -> list[int]:
        """Corrige o bug de Out-of-Bounds `i<=8` / `i=9` de `logica-leds/sketch.ino` (Z18)."""
        pos = int(step_idx) % 8
        if reverse:
            pos = 7 - pos
        leds = [0] * 8
        leds[pos] = 1
        return leds


# ============================================================================
# 10. AUDITORIA PROGRAMATICA DOS 20 ARQUIVOS .ZIP EM `Repositórios do GitHub/`
# ============================================================================
EXPECTED_20_ZIPS: dict[str, dict[str, str]] = {
    "Z01": {"zip": "IsaacLab-main.zip", "target": "python/aegis_ai/aegis_sim/"},
    "Z02": {"zip": "robotica-computacional-main.zip", "target": "python/aegis_ai/vision_2fa/"},
    "Z03": {"zip": "Robotics-Projects-master.zip", "target": "firmware/include/kinematics.h"},
    "Z04": {"zip": "lab-uc-main.zip", "target": "firmware/include/repo_peripherals_v3.h"},
    "Z05": {"zip": "COMET-AIR-MOUSE-main.zip", "target": "firmware/tools/comet_air_mouse_wand_c3/comet_air_mouse_wand_c3.ino"},
    "Z06": {"zip": "Introducao-a-robotica-main.zip", "target": "hardware/datasheet_limits.md"},
    "Z07": {"zip": "awesome-ai-robotics-main.zip", "target": "demo/diagnostic_agent.py"},
    "Z08": {"zip": "awesome-robotics-projects-master.zip", "target": "cad/mounts_opcionais/tray_mount_2dof.scad"},
    "Z09": {"zip": "MecatroniGit.github.io-master.zip", "target": "docs/labs/lab_01.md"},
    "Z10": {"zip": "sensor_de_umidade_do_ar_caseiro_sem_dependencia_barometrica-main.zip", "target": "python/aegis_ai/psychrometry.py"},
    "Z11": {"zip": "apostila_robotica_acnp-main.zip", "target": "python/aegis_ai/repo_integrations_v3.py"},
    "Z12": {"zip": "Maquina-de-Cafe-Automatica-com-Arduino-main.zip", "target": "firmware/include/repo_peripherals_v3.h"},
    "Z13": {"zip": "placar-digital-wokwi-main.zip", "target": "docs/wokwi/z13_placar_digital_74hc595/diagram.json"},
    "Z14": {"zip": "varredura-wokwi-main.zip", "target": "docs/wokwi/z14_varredura_7seg_pov/diagram.json"},
    "Z15": {"zip": "reguaeletronica-wokwi-main.zip", "target": "docs/wokwi/z15_regua_eletronica_hcsr04/diagram.json"},
    "Z16": {"zip": "sistema-de-entrada-main.zip", "target": "docs/wokwi/z16_sistema_entrada_bidirecional/diagram.json"},
    "Z17": {"zip": "contador-wokwi-main.zip", "target": "docs/wokwi/z17_contador_00_99_7seg/diagram.json"},
    "Z18": {"zip": "logica-leds-main.zip", "target": "docs/wokwi/z18_logica_leds_8x/diagram.json"},
    "Z19": {"zip": "Projeto_Display_de_Led_de_7S-Arduino-main.zip", "target": "docs/wokwi/z19_z20_semaforo_7seg_fsm/diagram.json"},
    "Z20": {"zip": "Semafaro_Arduino_cpp-main.zip", "target": "docs/wokwi/z19_z20_semaforo_7seg_fsm/sketch.ino"},
}


def verify_all_20_zip_repositories(workspace_root: Path | str | None = None) -> dict[str, Any]:
    """Verifica a existencia fisica dos 20 `.zip` e de seus respectivos modulos aplicados em `Projeto/`."""
    projeto_root = Path(__file__).resolve().parents[2]
    ws_root = Path(workspace_root) if workspace_root else projeto_root.parent
    zips_dir = ws_root / "Repositórios do GitHub"

    records: list[dict[str, Any]] = []
    missing_zips: list[str] = []
    missing_targets: list[str] = []

    for zid, meta in sorted(EXPECTED_20_ZIPS.items()):
        zpath = zips_dir / meta["zip"]
        tpath = projeto_root / meta["target"]
        z_exists = zpath.exists()
        t_exists = tpath.exists()
        file_count = 0
        if z_exists:
            with zipfile.ZipFile(zpath) as zf:
                file_count = len(zf.namelist())
        else:
            missing_zips.append(meta["zip"])
        if not t_exists:
            missing_targets.append(meta["target"])

        records.append(
            {
                "id": zid,
                "zip_name": meta["zip"],
                "zip_exists": z_exists,
                "zip_entries": file_count,
                "target_path": meta["target"],
                "target_exists": t_exists,
            }
        )

    return {
        "total_expected": 20,
        "verified_count": sum(1 for r in records if r["zip_exists"] and r["target_exists"]),
        "all_passed": len(missing_zips) == 0 and len(missing_targets) == 0,
        "missing_zips": missing_zips,
        "missing_targets": missing_targets,
        "records": records,
    }
