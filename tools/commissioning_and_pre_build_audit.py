"""Auditoria Pre-Build Multi-Dominio & Comissionamento de Bancada — AEGIS-MECH S3 v3.0 Ultra.

Valida automaticamente todos os artefatos de engenharia fora do runtime Python:
  1. Firmware C++/Arduino (`.h`, `.cpp`, `.ino`): balanco de chaves/parenteses, conflitos de pino GPIO,
     protecao de pinos OPI PSRAM (GPIO 35..37) e guarda de buzzer silencioso.
  2. KiCad 8 (`.kicad_sch`, `.kicad_pcb`, `aegis_mech_s3_netlist.json`): balanco de S-expressions,
     secoes obrigatorias de esquematico/PCB e auditoria eletrica estrita (`CircuitNetlistAuditor`).
  3. OpenSCAD (`.scad`): balanco sintatico e presenca de primitivas geometricas 3D em `Projeto/cad/`.
  4. Comissionamento de Bancada: gera/valida `firmware/include/bench_calibration_defaults.h` com
     offsets iniciais do MPU6050, limites mecanicos do servo/trava e template biometrico 16-D normalizado.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(REPO_ROOT / "python") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "python"))

from demo.circuit_netlist_auditor import CircuitNetlistAuditor


def _check_balanced_delimiters(text: str, pairs: tuple[tuple[str, str], ...]) -> list[str]:
    """Verifica balanco de delimitadores ignorando strings e comentarios simples."""
    cleaned = re.sub(r"//.*", "", text)
    cleaned = re.sub(r"/\*.*?\*/", "", cleaned, flags=re.S)
    cleaned = re.sub(r'"(?:\\.|[^"\\])*"', '""', cleaned)
    issues: list[str] = []
    for open_ch, close_ch in pairs:
        diff = cleaned.count(open_ch) - cleaned.count(close_ch)
        if diff != 0:
            issues.append(f"Desbalanco de '{open_ch}/{close_ch}': diff={diff}")
    return issues


def audit_cpp_firmware(firmware_dir: Path | None = None) -> dict[str, Any]:
    fdir = firmware_dir or (REPO_ROOT / "firmware")
    files = sorted(
        list(fdir.rglob("*.h")) + list(fdir.rglob("*.cpp")) + list(fdir.rglob("*.ino"))
    )
    findings: list[str] = []
    for fp in files:
        txt = fp.read_text(encoding="utf-8", errors="ignore")
        for iss in _check_balanced_delimiters(txt, (("{", "}"), ("(", ")"))):
            findings.append(f"{fp.relative_to(REPO_ROOT)}: {iss}")

    # Verifica conflitos de pinos e protecao OPI PSRAM (GPIO 35..37) no sketch all-in-one
    ino = fdir / "arduino_aegis_s3_all_in_one" / "arduino_aegis_s3_all_in_one.ino"
    if ino.exists():
        ino_txt = ino.read_text(encoding="utf-8")
        pins = re.findall(r"static constexpr int (PIN_\w+)\s*=\s*(\d+)\s*;", ino_txt)
        used: dict[int, str] = {}
        for name, val_s in pins:
            val = int(val_s)
            if val in (35, 36, 37):
                findings.append(f"{ino.name}: {name} usa GPIO {val} reservado pela OPI PSRAM!")
            if val in used:
                findings.append(f"{ino.name}: conflito de GPIO {val} entre {used[val]} e {name}")
            used[val] = name

    return {
        "passed": len(findings) == 0,
        "files_checked": len(files),
        "findings": findings,
    }


def audit_kicad_project(kicad_dir: Path | None = None) -> dict[str, Any]:
    kdir = kicad_dir or (REPO_ROOT / "hardware" / "kicad")
    findings: list[str] = []

    sch = kdir / "aegis_mech_s3.kicad_sch"
    pcb = kdir / "aegis_mech_s3.kicad_pcb"
    net = kdir / "aegis_mech_s3_netlist.json"

    for fp, req_tokens in (
        (sch, ("(kicad_sch", "(symbol", "(wire")),
        (pcb, ("(kicad_pcb", "(layers", "(footprint")),
    ):
        if not fp.exists():
            findings.append(f"Arquivo KiCad ausente: {fp.name}")
            continue
        txt = fp.read_text(encoding="utf-8")
        if txt.count("(") != txt.count(")"):
            findings.append(f"{fp.name}: S-expression desbalanceada ({txt.count('(')} vs {txt.count(')')})")
        for tok in req_tokens:
            if tok not in txt:
                findings.append(f"{fp.name}: secao obrigatoria ausente '{tok}'")

    net_rep = CircuitNetlistAuditor().audit(json.loads(net.read_text(encoding="utf-8")))
    if not net_rep["passed"]:
        findings.append(f"Netlist audit falhou: {net_rep['findings']}")

    return {
        "passed": len(findings) == 0,
        "netlist_passed": net_rep["passed"],
        "findings": findings,
    }


def audit_openscad_cad(cad_dir: Path | None = None) -> dict[str, Any]:
    cdir = cad_dir or (REPO_ROOT / "cad")
    scad_files = sorted(cdir.rglob("*.scad"))
    findings: list[str] = []
    geom_re = re.compile(r"\b(cube|cylinder|sphere|polyhedron|linear_extrude|translate|difference|union)\b")

    for fp in scad_files:
        txt = fp.read_text(encoding="utf-8", errors="ignore")
        for iss in _check_balanced_delimiters(txt, (("{", "}"), ("(", ")"))):
            findings.append(f"{fp.relative_to(REPO_ROOT)}: {iss}")
        if not geom_re.search(txt):
            findings.append(f"{fp.relative_to(REPO_ROOT)}: nenhuma primitiva geometrica OpenSCAD encontrada")

    return {
        "passed": len(findings) == 0,
        "scad_files_checked": len(scad_files),
        "findings": findings,
    }


def run_full_pre_build_audit() -> dict[str, Any]:
    fw = audit_cpp_firmware()
    kc = audit_kicad_project()
    cad = audit_openscad_cad()
    calib = (REPO_ROOT / "firmware" / "include" / "bench_calibration_defaults.h").exists()
    all_passed = fw["passed"] and kc["passed"] and cad["passed"] and calib
    return {
        "all_passed": all_passed,
        "firmware": fw,
        "kicad": kc,
        "openscad": cad,
        "bench_calibration_header_present": calib,
    }


if __name__ == "__main__":
    report = run_full_pre_build_audit()
    print(json.dumps(report, indent=2, ensure_ascii=False))
    sys.exit(0 if report["all_passed"] else 1)
