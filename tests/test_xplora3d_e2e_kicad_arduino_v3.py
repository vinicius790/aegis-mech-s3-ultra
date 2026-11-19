"""Suite de testes de profundidade industrial v3.0:
- Exportador & Auditor 3D Xplora3D (`aegis_mech_s3_v3.x3d` com 25 componentes, 49 jumpers, buzzer silencioso e `#p=` roundtrip)
- Integracao completa dos 20 Repositorios `.zip` (`Z01..Z20`) em `repo_integrations_v3.py`, `repo_peripherals_v3.h` e `docs/wokwi/z13..z20`
- Pipeline E2E sobre CSVs reais (`amostras.csv`, `fluxo_e2e.csv`, `fluxo_e2e_gabarito.csv`, `senha_e2e.csv`)
- Esquematico e Layout PCB KiCad 8 (`aegis_mech_s3.kicad_sch` e `aegis_mech_s3.kicad_pcb`)
- Cinematica 2-DOF/Stewart C++ (`kinematics.h`), Biometria NVS (`main.cpp`), BLE HID (`wireless_dual.cpp`), Varinha C3 (`comet_air_mouse_wand_c3.ino`) e Sketch `.ino`
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
import urllib.request

import numpy as np

from aegis_ai.e2e_pipeline import run_e2e_evaluation
from aegis_ai.repo_integrations_v3 import (
    BidirectionalSonarGateFSM,
    CometAirMouseProcessor,
    LineFollower5IRController,
    NTCThermistorBeta3950,
    OccupancyGridMapper,
    PreFlightInterlockValidator,
    PsychrometerWetDryBulb,
    SevenSegmentCascade74HC595,
    TrafficLightNonBlockingFSM,
    verify_all_20_zip_repositories,
)
from aegis_ai.wokwi_suite_generator import generate_all_wokwi_reference_projects
from aegis_ai.xplora3d_exporter import (
    audit_xplora3d_project,
    build_aegis_xplora3d_project,
    decode_xplora3d_share_url,
    encode_xplora3d_share_url,
    export_aegis_xplora3d_bundle,
)
from demo.app_web import create_server
from demo.circuit_netlist_auditor import CircuitNetlistAuditor


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_xplora3d_exporter_25_components_silent_buzzer_and_roundtrip(tmp_path: Path) -> None:
    proj = build_aegis_xplora3d_project()
    audit = audit_xplora3d_project(proj)
    assert audit["passed"] is True, f"Falhas de auditoria Xplora3D: {audit['findings']}"
    assert audit["component_count"] == 40
    assert audit["jumper_count"] == 99
    assert audit["total_breadboards_count"] == 5
    assert audit["gpio_active_count"] == 24
    assert len(proj["multi_mcu_catalog"]) == 7
    assert "buz_pin.value(tick" not in proj["python"]
    assert "read_ntc_celsius" in proj["python"]
    assert "compute_psychrometry_and_sound" in proj["python"]
    assert "shift_out_74hc595" in proj["python"]

    share_url = encode_xplora3d_share_url(proj)
    assert share_url.startswith("https://xplora.phinalabs.com/3d/#p=")
    decoded = decode_xplora3d_share_url(share_url)
    assert decoded["version"] == "2.1"
    assert len(decoded["components"]) == 40
    assert len(decoded["jumpers"]) == 99

    bundle = export_aegis_xplora3d_bundle(output_dir=tmp_path)
    assert Path(bundle["x3d_path"]).exists()
    assert Path(bundle["url_path"]).exists()


def test_all_20_github_zip_repositories_integrated_and_verified() -> None:
    # Gera a suite Wokwi Z13..Z20 e audita cada diagram.json com o CircuitNetlistAuditor
    diagram_paths = generate_all_wokwi_reference_projects()
    assert len(diagram_paths) == 7
    auditor = CircuitNetlistAuditor()
    for dpath in diagram_paths:
        diag = json.loads(Path(dpath).read_text(encoding="utf-8"))
        rep = auditor.audit(diag)
        assert rep["passed"] is True, f"Falha na auditoria Wokwi de {dpath}: {rep['findings']}"

    # Verifica os 20 arquivos .zip e seus 20 alvos em Projeto/
    catalog = verify_all_20_zip_repositories()
    assert catalog["total_expected"] == 20
    assert catalog["verified_count"] == 20, f"Faltando: {catalog['missing_zips']} / {catalog['missing_targets']}"
    assert catalog["all_passed"] is True

    # Z10: Psicrometro bulbo seco/umido corrigido
    psy = PsychrometerWetDryBulb.evaluate(25.0, 20.0)
    assert 55.0 <= psy["rh_analytic_pct"] <= 70.0
    assert 60.0 <= psy["rh_table_pct"] <= 75.0
    assert psy["speed_of_sound_mps"] > 346.0

    # Z11 + Z04: Termistor NTC 10k Beta=3950 com guarda nos extremos ADC 0 e 4095
    ntc = NTCThermistorBeta3950(r_series_ohm=10000.0)
    assert abs(ntc.adc_to_celsius(2048, 4096) - 25.0) < 0.5
    assert -60.0 < ntc.adc_to_celsius(0, 4095) < 200.0
    assert -60.0 < ntc.adc_to_celsius(4095, 4095) < 200.0

    # Z13 + Z14 + Z15 + Z17 + Z19: LUT 7-segmentos com digito 6 = 0x7D e checagem GPIO 12 MTDI
    assert SevenSegmentCascade74HC595.encode_digit(6) == 0x7D
    assert len(SevenSegmentCascade74HC595.encode_matrix_10x8()) == 10
    assert SevenSegmentCascade74HC595.encode_cascade_stream(6789, 4) == [0x7D, 0x07, 0x7F, 0x6F]
    assert SevenSegmentCascade74HC595.validate_pov_digit_pins([18, 19, 20, 21])["valid"] is True
    assert SevenSegmentCascade74HC595.validate_pov_digit_pins([12, 13, 14, 15])["valid"] is False

    # Z16: Contador bidirecional S1 <-> S2 corrigido
    gate = BidirectionalSonarGateFSM(range_cm=150.0)
    gate.step(80.0, 250.0, 100.0)  # S1 primeiro
    r_in = gate.step(80.0, 90.0, 250.0)  # S2 segundo -> ENTRADA!
    assert r_in["event"] == "ENTRY" and r_in["occupancy"] == 1
    gate.step(250.0, 250.0, 300.0)  # Libera corredor
    gate.step(250.0, 75.0, 400.0)  # S2 primeiro
    r_out = gate.step(70.0, 75.0, 520.0)  # S1 segundo -> SAIDA!
    assert r_out["event"] == "EXIT" and r_out["occupancy"] == 0

    # Z12: Pre-Flight Checklist
    pf_ok = PreFlightInterlockValidator.validate()
    assert pf_ok.passed is True and pf_ok.fault_code == "E00: NOMINAL"
    pf_err = PreFlightInterlockValidator.validate(estop_closed=False)
    assert pf_err.passed is False and pf_err.fault_code == "E01: ESTOP_OPEN"

    # Z05: COMET Air Mouse 3-axis calib + click/scroll
    mouse = CometAirMouseProcessor()
    cal = mouse.calibrate_500(np.zeros((500, 3)))
    assert cal["stationary_valid"] is True
    dx, dy = mouse.step_cursor(0.0, 20.0, -30.0)
    assert dx != 0 or dy != 0
    assert mouse.classify_button_press(80.0, is_left=True) == "LEFT_CLICK"
    assert mouse.classify_button_press(220.0, is_left=True) == "SCROLL_DOWN"

    # Z03 + Z02 + Z18 + Z20: LineFollower PID, OccupancyGrid, TrafficLight FSM e Scan 8 LEDs
    lf = LineFollower5IRController()
    out_lf = lf.step([0.0, 0.0, 0.2, 0.8, 1.0], dt=0.02)
    assert out_lf["error"] > 0.0 and out_lf["control_u"] > 0.0

    og = OccupancyGridMapper()
    r, c = og.update_hit(200.0, 150.0, occupied=True)
    assert og.probability_grid()[r, c] > 0.5

    tl = TrafficLightNonBlockingFSM()
    assert tl.step(100.0)["state"] == "GREEN"
    assert tl.step(2200.0)["state"] == "YELLOW"
    assert sum(TrafficLightNonBlockingFSM.scan_8_leds_safe(15)) == 1


def test_e2e_pipeline_on_real_csv_datasets() -> None:
    report = run_e2e_evaluation()
    assert report.sample_count == 84
    assert report.sample_accuracy == 1.0
    assert report.roc_auc >= 0.95
    assert report.eer <= 0.10
    assert report.expected_password == ["I_v", "O", "L"]
    assert report.detected_sequence == ["I_v", "O", "L"]
    assert report.sequence_unlocked is True
    assert len(report.windows) == 3
    assert len(report.stream_frames) == 130


def test_kicad8_schematic_and_pcb_depth() -> None:
    sch_text = (REPO_ROOT / "hardware" / "kicad" / "aegis_mech_s3.kicad_sch").read_text(encoding="utf-8")
    pcb_text = (REPO_ROOT / "hardware" / "kicad" / "aegis_mech_s3.kicad_pcb").read_text(encoding="utf-8")

    assert sch_text.count("(") == sch_text.count(")")
    assert pcb_text.count("(") == pcb_text.count(")")

    for token in ("ESP32-S3-WROOM-1-N16R8", "VL53L1X", "HC-SR04", "MPU6050", "IRLZ44N", "1N5819", "(wire", "(junction"):
        assert token in sch_text, f"Token ausente no esquematico KiCad: {token}"

    for token in ("(footprint", "(segment", "(via", "(zone", "ECHO_3V3", "NET_GATE", "NET_DRAIN"):
        assert token in pcb_text, f"Token ausente no PCB KiCad: {token}"


def test_firmware_kinematics_biometrics_ble_peripherals_and_sketches() -> None:
    kin_h = (REPO_ROOT / "firmware" / "include" / "kinematics.h").read_text(encoding="utf-8")
    periph_h = (REPO_ROOT / "firmware" / "include" / "repo_peripherals_v3.h").read_text(encoding="utf-8")
    main_cpp = (REPO_ROOT / "firmware" / "src" / "main.cpp").read_text(encoding="utf-8")
    wireless_cpp = (REPO_ROOT / "firmware" / "src" / "comms" / "wireless_dual.cpp").read_text(encoding="utf-8")
    ino_sketch = (
        REPO_ROOT / "firmware" / "arduino_aegis_s3_all_in_one" / "arduino_aegis_s3_all_in_one.ino"
    ).read_text(encoding="utf-8")
    wand_sketch = (
        REPO_ROOT / "firmware" / "tools" / "comet_air_mouse_wand_c3" / "comet_air_mouse_wand_c3.ino"
    ).read_text(encoding="utf-8")

    assert "class Kinematics2DOF" in kin_h
    assert "damped_least_squares_step" in kin_h
    assert "class StewartPlatform6DOF" in kin_h

    assert "kSevenSegLUT[10]" in periph_h and "0x7D" in periph_h
    assert "class HX711LoadCell" in periph_h
    assert "struct PreFlightSafetyChecklist" in periph_h
    assert "class BidirectionalPassageDetector" in periph_h
    assert "class LineFollowerPID5IR" in periph_h
    assert "class ShiftRegister74HC595" in periph_h
    assert "class TM1637Display4Dig" in periph_h
    assert "class FSR402ForceSensor" in periph_h
    assert "class TrafficLightFSM3LED" in periph_h
    assert "class SilentBuzzerController" in periph_h

    assert "res.bio_cos_sim = 0.92f" not in main_cpp
    assert "s_nvs_bio_template" in main_cpp
    assert "BleHidKeyboardReport" in wireless_cpp
    assert "computeSpeedOfSoundMps" in ino_sketch
    assert "shiftOut74HC595" in ino_sketch
    assert "PIN_RELAY_LOCK" in ino_sketch
    assert "PIN_TM1637_CLK" in ino_sketch
    assert "PIN_DHT11_DATA" in ino_sketch
    assert "calibrateGyro500ThreeAxis" in wand_sketch

    bom_txt = (REPO_ROOT / "hardware" / "bom.csv").read_text(encoding="utf-8")
    net_txt = (REPO_ROOT / "hardware" / "kicad" / "aegis_mech_s3_netlist.json").read_text(encoding="utf-8")
    for token in ("74HC595", "TM1637", "DHT11", "TCRT5000", "FSR", "AMS1117"):
        assert token in bom_txt, f"Faltando {token} em hardware/bom.csv"
        assert token in net_txt, f"Faltando {token} em hardware/kicad/aegis_mech_s3_netlist.json"


def test_web_cockpit_e2e_and_xplora3d_endpoints() -> None:
    srv = create_server("127.0.0.1", 0)
    port = srv.server_address[1]
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/e2e_report", timeout=10) as resp:
            e2e_data = json.loads(resp.read().decode("utf-8"))
        assert e2e_data["sequence_unlocked"] is True
        assert e2e_data["detected_sequence"] == ["I_v", "O", "L"]

        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/xplora3d", timeout=5) as resp:
            xp_data = json.loads(resp.read().decode("utf-8"))
        assert xp_data["version"] == "2.1"
        assert xp_data["audit"]["passed"] is True
        assert xp_data["share_url"].startswith("https://xplora.phinalabs.com/3d/#p=")
    finally:
        srv.shutdown()
        srv.server_close()
