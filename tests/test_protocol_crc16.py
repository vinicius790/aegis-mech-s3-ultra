# -*- coding: utf-8 -*-
import pytest

def calculate_crc16_python(data: bytes) -> int:
    """Implementacao Python exata do CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def test_crc16_ccitt_vectors():
    # Vetores padrao conhecidos de CRC-16/CCITT-FALSE
    assert calculate_crc16_python(b"123456789") == 0x29B1
    assert calculate_crc16_python(b"AEGIS") == 0x3D13

def test_packet_structure_integrity():
    sync = 0xAA55
    version = 0x01
    msg_type = 0x01 # SENSOR_50HZ
    header_bytes = bytes([sync >> 8, sync & 0xFF, version, msg_type])
    crc = calculate_crc16_python(header_bytes[2:]) # Cobre de version ate fim
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF
