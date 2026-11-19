# -*- coding: utf-8 -*-
"""Pydantic v2 Data Contracts, Binary AEGIS-PROTO v1 Packet Codec & Schemas."""

import struct
from enum import IntEnum
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Tuple, Union


class MsgType(IntEnum):
    SENSOR_50HZ = 0x01
    GESTURE_RESULT = 0x02
    ACTUATOR_100HZ = 0x03
    TAMPER_ALERT = 0x04
    HIL_INJECT_FRAME = 0x05
    POST_BOOT_REPORT = 0x06
    VISION_2FA_SYNC = 0x07
    HEALTH_EXTENDED = 0x08


class AmostraCSV(BaseModel):
    amostra: int
    forma: str
    sessao: int
    t_ms: float
    x_mm: Optional[float] = None
    y_mm: Optional[float] = None
    x_tof_mm: Optional[float] = None
    y_tof_mm: Optional[float] = None
    temp_c: Optional[float] = 25.0


class TelemetriaFrame(BaseModel):
    sync: int = 0xAA55
    version: int = 0x01
    msg_type: int
    seq_num: int
    t_cap_us: int
    us_x_mm: float
    us_y_mm: float
    tof_x_mm: float
    tof_y_mm: float
    kalman_x_mm: float
    kalman_y_mm: float
    kalman_vx: float
    kalman_vy: float
    temp_c: float
    mahalanobis_flags: int = 0
    particle_spread_mm: int = 0


class GestureInferResult(BaseModel):
    pred_class: int
    class_name: str
    logits: List[int]
    margin_int32: int
    bio_cos_sim: float
    ood_recon_mse: float
    infer_us: int
    accepted: bool


class TamperAlertFrame(BaseModel):
    tamper_class: int
    score: float
    peak_accel_g: float
    peak_gyro_dps: float
    rms_fft_10_40hz: float
    latched_nvs: int


class HilInjectFrame(BaseModel):
    fake_us_x_mm: float
    fake_us_y_mm: float
    fake_tof_x_mm: float
    fake_tof_y_mm: float
    fake_imu_g: float
    override_mask: int


class PostBootReport(BaseModel):
    vectors_passed: int
    vectors_total: int
    i2c_ack_mask: int
    gyro_var_max: float
    flash_mode_ok: int
    psram_kb: int
    fw_crc32: int


class Vision2FASyncFrame(BaseModel):
    aruco_id: int
    badge_color_hsv: int
    yolo_person_count: int
    tailgating_alert: int
    pose_dist_mm: float
    valid_token: int
    nonce: int


class HealthExtendedFrame(BaseModel):
    v_mains_v: float
    v_batt_v: float
    v_bus_v: float
    v_5v_est: float
    temp_c: float
    rh_pct: float
    ats_source: int
    fault_code: int
    stack_hwm_bytes: int


class ManifestoModelo(BaseModel):
    version: str
    model_sha256: str
    firmware_sha256: Optional[str] = None
    se_scale: float = 0.015625
    s1_shift: int
    input_dim: int = 120
    classes: List[str]
    num_params: int
    num_params_unfolded: Optional[int] = None
    ood_params: Optional[int] = None
    tamper_params: Optional[int] = None
    offsets: Dict[str, str] = Field(
        default_factory=lambda: {
            "bootloader": "0x0",
            "partitions": "0x8000",
            "boot_app0": "0xe000",
            "app": "0x10000",
        }
    )


HEADER_FMT = "<HBBHQB"  # 15 bytes: sync(2), version(1), msg_type(1), seq_num(2), t_cap_us(8), payload_len(1)
PAYLOAD_FMTS: Dict[int, str] = {
    MsgType.SENSOR_50HZ: "<9fBB",          # 38 bytes
    MsgType.GESTURE_RESULT: "<B7ii2f120bIB", # 166 bytes
    MsgType.ACTUATOR_100HZ: "<4f2hBB",     # 22 bytes
    MsgType.TAMPER_ALERT: "<B4fB6x",       # 24 bytes
    MsgType.HIL_INJECT_FRAME: "<5fI",      # 24 bytes
    MsgType.POST_BOOT_REPORT: "<BBHfBHIx", # 16 bytes (15 + 1 reserved)
    MsgType.VISION_2FA_SYNC: "<i3BfBQ",    # 20 bytes
    MsgType.HEALTH_EXTENDED: "<6f2BH",     # 28 bytes (padded cleanly to exact C struct)
}


def calculate_crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, vetor '123456789' -> 0x29B1)."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def pack_aegis_frame(msg_type: Union[int, MsgType], seq_num: int, t_cap_us: int, payload: bytes) -> bytes:
    """Empacota quadro binario AEGIS-PROTO v1 com header 15 B + payload + CRC16 big-endian."""
    mtype = int(msg_type)
    if len(payload) > 255:
        raise ValueError("Payload excede 255 bytes.")
    hdr = struct.pack(HEADER_FMT, 0xAA55, 0x01, mtype, seq_num & 0xFFFF, int(t_cap_us), len(payload))
    crc = calculate_crc16_ccitt(hdr[2:] + payload)
    return hdr + payload + struct.pack(">H", crc)


def unpack_aegis_frame(raw: bytes) -> Tuple[int, int, int, bytes]:
    """Valida SYNC, versao e CRC-16 de quadro AEGIS-PROTO v1."""
    hdr_sz = struct.calcsize(HEADER_FMT)
    if len(raw) < hdr_sz + 2:
        raise ValueError("Quadro menor que cabecalho minimo.")
    sync, version, msg_type, seq_num, t_cap_us, plen = struct.unpack(HEADER_FMT, raw[:hdr_sz])
    if sync != 0xAA55 or version != 0x01:
        raise ValueError(f"Sync/version invalidos: sync={sync:#x}, ver={version}")
    total_sz = hdr_sz + plen + 2
    if len(raw) < total_sz:
        raise ValueError("Quadro truncado.")
    payload = raw[hdr_sz : hdr_sz + plen]
    rx_crc = struct.unpack(">H", raw[hdr_sz + plen : total_sz])[0]
    calc_crc = calculate_crc16_ccitt(raw[2 : hdr_sz + plen])
    if rx_crc != calc_crc:
        raise ValueError(f"CRC invalido: esperado {calc_crc:#06x}, recebido {rx_crc:#06x}")
    return msg_type, seq_num, t_cap_us, payload
