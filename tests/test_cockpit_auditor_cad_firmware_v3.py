# -*- coding: utf-8 -*-
"""Testes Abrangentes v3.0 — Auditor Anti-Alucinacao, Cockpit HTTP Live, ROSA AST, CAD & Contratos FW."""

import json
import threading
import urllib.request
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "demo"))

from app_web import create_cockpit_server
from circuit_netlist_auditor import CircuitNetlistAuditor
from diagnostic_agent import ReadOnlyDiagnosticAgent, verify_safe_query_ast


def test_auditor_approves_aegis_netlist_and_wokwi():
    aud = CircuitNetlistAuditor()
    aegis_nl = json.loads((ROOT / "hardware" / "kicad" / "aegis_mech_s3_netlist.json").read_text(encoding="utf-8"))
    wokwi_nl = json.loads((ROOT / "docs" / "wokwi" / "diagram.json").read_text(encoding="utf-8"))
    assert aud.audit(aegis_nl)["passed"] is True
    assert aud.audit(wokwi_nl)["passed"] is True


def test_auditor_catches_all_6_real_reference_bugs():
    aud = CircuitNetlistAuditor()

    # 1. A37E8C2F: LM358 astavel sem histerese positiva
    r1 = aud.audit({"parts": [{"id": "U1", "type": "opamp_astable", "has_positive_feedback": False}], "nets": []})
    assert any(f["rule_id"] == "RULE_OSCILLATOR_FEEDBACK" for f in r1["findings"]) and not r1["passed"]

    # 2. EAFA7E0D: Contator usado como caminho de surto SPD
    r2 = aud.audit({"parts": [{"id": "K1", "type": "contactor_coil", "connected_to_spd_surge": True}], "nets": []})
    assert any(f["rule_id"] == "RULE_NO_SURGE_COIL" for f in r2["findings"]) and not r2["passed"]

    # 3. lab-uc Exp 16: carga indutiva sem flyback
    r3 = aud.audit({"parts": [{"id": "M1", "type": "dc_motor_discrete"}], "nets": []})
    assert any(f["rule_id"] == "RULE_NO_FLYBACK" for f in r3["findings"]) and not r3["passed"]

    # 4. lab-uc Exp 12: sinal 5V direto em GPIO 3.3V sem divisor
    r4 = aud.audit({
        "mcu": "ESP32-S3", "mcu_vcc": 3.3, "parts": [],
        "nets": [{"name": "ECHO_RAW", "driver_voltage": 5.0, "connects_to_mcu_gpio": True, "has_voltage_divider": False, "mcu_gpio": 5}]
    })
    assert any(f["rule_id"] == "RULE_LOGIC_LEVEL_5V_ON_3V3" for f in r4["findings"]) and not r4["passed"]

    # 5. lab-uc Exp 05 (GPIO 35 input-only no ESP32) + N16R8 PSRAM colisao
    r5 = aud.audit({
        "mcu": "ESP32", "mcu_vcc": 3.3, "parts": [],
        "nets": [{"name": "HX711_SCK", "mcu_gpio": 35, "direction": "OUTPUT"}]
    })
    assert any(f["rule_id"] == "RULE_ESP32_PIN_CAPS" for f in r5["findings"]) and not r5["passed"]

    # 6. contador-wokwi r2 = 0.22 ohm em diagram.json
    r6 = aud.audit({
        "parts": [{"id": "esp", "type": "board-esp32-s3-devkitc-1"}, {"id": "r2", "type": "wokwi-resistor", "attrs": {"value": "0.22"}}],
        "connections": [["esp:35", "r2:1", "red", []]]
    })
    rule_ids = {f["rule_id"] for f in r6["findings"]}
    assert "RULE_CURRENT_LIMIT_RESISTOR" in rule_ids
    assert "RULE_ESP32_PIN_CAPS" in rule_ids


def test_live_cockpit_http_server_endpoints():
    srv = create_cockpit_server("127.0.0.1", 0)
    port = srv.server_address[1]
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=3) as resp:
            html = resp.read().decode("utf-8")
            assert resp.status == 200 and "Tiny Logic Zoom" in html and "Faddeev-LeVerrier" in html

        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=3) as resp:
            st = json.loads(resp.read().decode("utf-8"))
            assert st["fsm_state"] == "LOCKED_IDLE" and st["fault_code"] == "E00: NOMINAL"

        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/stream", timeout=3) as resp:
            first_evt = resp.readline().decode("utf-8")
            assert "data:" in first_evt and "LOCKED_IDLE" in first_evt

        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/api/stability",
            data=json.dumps({"kp": 65.0, "ki": 2.0, "kd": 2.4}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=3) as resp:
            stab = json.loads(resp.read().decode("utf-8"))
            assert stab["is_stable"] is True and stab["spectral_radius"] < 1.0
    finally:
        srv.shutdown()
        srv.server_close()


def test_demo_bridges_and_recorder_roundtrip(tmp_path):
    from lerobot_arm_bridge import LeRobotBridge
    from rerun_logger import RerunTelemetryLogger
    from serial_recorder import SerialRecorder
    from aegis_ai.kalman_particle import KalmanFilter2D

    bridge = LeRobotBridge(ema_alpha=0.5)
    th1, th2 = bridge.execute_policy({"target_x_mm": 150.0, "target_z_mm": 40.0}, obs_age_ms=10.0)
    assert th1 != 0.0 or th2 != 0.0
    # Comando atrasado > 150ms deve manter ultima pose segura!
    frozen = bridge.execute_policy({"target_x_mm": 210.0, "target_z_mm": 10.0}, obs_age_ms=200.0)
    assert frozen == (th1, th2)

    rlog = RerunTelemetryLogger()
    rlog.log_frame(20.0, 210.0, 215.0, 210.0, 215.0, 210.0, 215.0, current_ma=120.0, bolt_mm=14.0)
    assert len(rlog.offline_frames) == 1

    rec = SerialRecorder()
    rec.push_sample({"t_ms": 0.0, "x_mm": 220.0, "y_mm": 220.0})
    out_csv = tmp_path / "bench_session.csv"
    assert rec.record_session(str(out_csv)) == 1
    assert out_csv.exists()

    kf2 = KalmanFilter2D()
    kf2.predict(0.02)
    assert kf2.update_x(204.0, is_tof=True) is True
    assert kf2.update_y(900.0, is_tof=False) is False


def test_rosa_ast_firewall_and_live_tools():
    assert verify_safe_query_ast("get_system_health()") is True
    assert verify_safe_query_ast("__import__('os').system('calc')") is False
    assert verify_safe_query_ast("unlock()") is False

    agent = ReadOnlyDiagnosticAgent()
    assert agent.get_kalman_anomaly_report()["samples_processed"] > 50
    assert agent.get_actuator_wear_metrics()["peak_current_ma"] >= 320.0
    rep = agent.audit_netlist_file("hardware/kicad/aegis_mech_s3_netlist.json")
    assert rep["passed"] is True


def test_cad_firmware_and_docs_structural_integrity():
    # 1. CAD v3.0
    painel = (ROOT / "cad" / "painel_sensores.scad").read_text(encoding="utf-8")
    arena = (ROOT / "cad" / "arena_base.scad").read_text(encoding="utf-8")
    trava = (ROOT / "cad" / "trava_cremalheira.scad").read_text(encoding="utf-8")
    caixa = (ROOT / "cad" / "caixa_cofre.scad").read_text(encoding="utf-8")
    assert "bloco_w = 56.0" in painel
    assert "janela_sensores_parede()" in arena
    assert "mod_m      = 1.25" in trava and "Z_pinhao   = 16" in trava
    assert "11.15" in caixa  # E-Stop Ø22.3mm

    # 2. Firmware IDF 4.4 + qio_opi + WebFlasher boot_app0 + 12 Labs
    pio = (ROOT / "firmware" / "platformio.ini").read_text(encoding="utf-8")
    rmt_cpp = (ROOT / "firmware" / "src" / "sensors" / "sensor_ultrasonic_rmt.cpp").read_text(encoding="utf-8")
    rmt_h = (ROOT / "firmware" / "src" / "sensors" / "sensor_ultrasonic_rmt.h").read_text(encoding="utf-8")
    wf = json.loads((ROOT / "docs" / "web_flasher" / "manifest.json").read_text(encoding="utf-8"))

    assert "flash_mode = qio" in pio and "qio_opi" in pio and "espressif32@^6.9.0" in pio
    assert "<driver/rmt.h>" in rmt_h and "rmt_tx.h" not in rmt_cpp and "rmt_tx.h" not in rmt_h
    assert any(p["offset"] == 57344 for p in wf["builds"][0]["parts"])
    for idx in range(1, 13):
        assert (ROOT / "docs" / "labs" / f"lab_{idx:02d}.md").exists()
