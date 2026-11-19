# -*- coding: utf-8 -*-
import pytest
import sys
from pathlib import Path

# Adiciona o diretorio python ao sys.path
sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from aegis_ai.contratos import AmostraCSV, TelemetriaFrame, ManifestoModelo
from aegis_ai.config import S_E, MAHALANOBIS_THRESHOLD_SQ

def test_pydantic_contratos():
    amostra = AmostraCSV(
        amostra=1,
        forma="I_v",
        sessao=1,
        t_ms=120.0,
        x_mm=220.5,
        y_mm=180.2
    )
    assert amostra.forma == "I_v"
    assert amostra.x_mm == 220.5

def test_telemetria_header():
    frame = TelemetriaFrame(
        msg_type=1,
        seq_num=42,
        t_cap_us=1000200,
        us_x_mm=210.0,
        us_y_mm=210.0,
        tof_x_mm=210.5,
        tof_y_mm=210.2,
        kalman_x_mm=210.2,
        kalman_y_mm=210.1,
        kalman_vx=0.0,
        kalman_vy=0.0,
        temp_c=25.0
    )
    assert frame.sync == 0xAA55
    assert frame.version == 0x01
    assert frame.seq_num == 42
