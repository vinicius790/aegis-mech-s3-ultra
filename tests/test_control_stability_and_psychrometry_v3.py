# -*- coding: utf-8 -*-
"""Testes Abrangentes v3.0 — Estabilidade Espectral Faddeev-LeVerrier, Psicrometria & AEGIS-PROTO 0x01..08."""

import struct
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

from aegis_ai.contratos import (
    HEADER_FMT,
    MsgType,
    PAYLOAD_FMTS,
    calculate_crc16_ccitt,
    pack_aegis_frame,
    unpack_aegis_frame,
)
from aegis_ai.control_stability import auditar_tabela_fuzzy_3x3, avaliar_estabilidade_pid
from aegis_ai.psychrometry import relative_humidity_magnus, speed_of_sound_mps


def test_faddeev_leverrier_and_fuzzy_table_stability():
    rep = auditar_tabela_fuzzy_3x3(h=0.01, tau=0.08)
    assert len(rep["points"]) == 9
    assert rep["all_stable"] is True
    assert rep["max_spectral_radius"] < 1.0

    unstable = avaliar_estabilidade_pid(kp=4500.0, ki=50.0, kd=50.0, h=0.01, tau=0.08)
    assert unstable["is_stable"] is False
    assert unstable["regime"] == "instavel"


def test_magnus_tetens_psychrometry_and_sound_speed():
    assert relative_humidity_magnus(25.0, 25.0) == 100.0
    rh = relative_humidity_magnus(25.0, 18.0)
    assert 40.0 < rh < 60.0
    c_cold = speed_of_sound_mps(5.0, 20.0)
    c_hot = speed_of_sound_mps(40.0, 90.0)
    assert 330.0 < c_cold < 340.0
    assert c_hot > c_cold + 18.0


def test_aegis_proto_v1_all_8_messages_and_crc_errors():
    assert calculate_crc16_ccitt(b"123456789") == 0x29B1
    assert struct.calcsize(HEADER_FMT) == 15

    expected_sizes = {
        MsgType.SENSOR_50HZ: 38,
        MsgType.GESTURE_RESULT: 166,
        MsgType.ACTUATOR_100HZ: 22,
        MsgType.TAMPER_ALERT: 24,
        MsgType.HIL_INJECT_FRAME: 24,
        MsgType.POST_BOOT_REPORT: 16,
        MsgType.VISION_2FA_SYNC: 20,
        MsgType.HEALTH_EXTENDED: 28,
    }
    for mtype, sz in expected_sizes.items():
        assert struct.calcsize(PAYLOAD_FMTS[mtype]) == sz
        payload = bytes((i % 251 for i in range(sz)))
        pkt = pack_aegis_frame(mtype, seq_num=17, t_cap_us=987654321, payload=payload)
        got_type, got_seq, got_t, got_payload = unpack_aegis_frame(pkt)
        assert got_type == int(mtype)
        assert got_seq == 17
        assert got_t == 987654321
        assert got_payload == payload

        # Corrupcao de 1 bit no payload deve falhar na verificacao CRC!
        corrupted = bytearray(pkt)
        corrupted[16] ^= 0x01
        try:
            unpack_aegis_frame(bytes(corrupted))
            raise AssertionError("Pacote corrompido deveria lancar ValueError")
        except ValueError:
            pass
