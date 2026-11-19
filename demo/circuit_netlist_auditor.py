# -*- coding: utf-8 -*-
"""Auditor Estatico Anti-Alucinacao de Circuitos & Netlists (KiCad JSON / Wokwi diagram.json).

Previne sistematicamente bugs eletricos e alucinacoes GenAI catalogadas nas referencias:
- A37E8C2F (LM358 sem realimentacao positiva)
- EAFA7E0D (bobina de contator tratada como via de surto SPD)
- lab-uc Exp 16 / 3F2B4AAE (carga indutiva sem diodo flyback)
- lab-uc Exp 12 (sinal 5V / pull-up direto em GPIO 3.3V sem divisor)
- lab-uc Exp 05 & varredura-wokwi (GPIO 34..39 input-only ESP32 / GPIO 26..37 PSRAM S3 / Strapping GPIO12)
- contador-wokwi (resistor limitador de LED 0.22 ohm < 100 ohm)
"""

import argparse
import json
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Dict, Any


@dataclass
class AuditFinding:
    rule_id: str
    severity: str   # "CRITICAL" | "HIGH" | "WARNING"
    component: str
    message: str


class CircuitNetlistAuditor:
    ESP32_INPUT_ONLY = {34, 35, 36, 39}
    ESP32S3_PSRAM_FORBIDDEN = set(range(26, 38))
    ESP32_CLASSIC_STRAPPING_WARN = {0, 2, 5, 12, 15}
    ESP32S3_STRAPPING_WARN = {0, 3, 45, 46}

    def audit(self, schematic: Dict[str, Any]) -> Dict[str, Any]:
        # Detecta automaticamente Wokwi diagram.json bruto se contiver "connections" e sem "nets"
        if "connections" in schematic and "nets" not in schematic:
            return self.audit_wokwi_diagram(schematic)

        findings: List[AuditFinding] = []
        parts = {p["id"]: p for p in schematic.get("parts", [])}
        nets = schematic.get("nets", [])
        mcu_arch = schematic.get("mcu", "ESP32-S3")
        mcu_vcc = float(schematic.get("mcu_vcc", 3.3))

        # 1. RULE_NO_FLYBACK: Carga indutiva discreta DEVE ter diodo antiparalelo (+V <-> dreno/coletor)
        inductive_types = {"solenoid", "relay", "dc_motor_discrete"}
        diodes = [p for p in parts.values() if p.get("type") in {"diode", "schottky"} and p.get("role") == "flyback"]
        for pid, p in parts.items():
            if p.get("type") in inductive_types:
                if not any(pid in d.get("protects", []) for d in diodes):
                    findings.append(
                        AuditFinding(
                            "RULE_NO_FLYBACK",
                            "CRITICAL",
                            pid,
                            f"Carga indutiva '{pid}' acionada sem diodo de roda livre em antiparalelo (-L*di/dt).",
                        )
                    )

        # 2. RULE_LOGIC_LEVEL_5V_ON_3V3: Sinal >= 4.5V ligado a GPIO 3.3V sem divisor seguro
        if mcu_vcc <= 3.6:
            for net in nets:
                drv_v = float(net.get("driver_voltage", mcu_vcc))
                if drv_v >= 4.5 and net.get("connects_to_mcu_gpio", False):
                    has_div = bool(net.get("has_voltage_divider", False))
                    r1 = float(net.get("r1_ohm", 0.0))
                    r2 = float(net.get("r2_ohm", 1.0))
                    v_div = drv_v * (r2 / max(r1 + r2, 1.0)) if has_div else drv_v
                    if (not has_div) or (v_div > 3.45):
                        findings.append(
                            AuditFinding(
                                "RULE_LOGIC_LEVEL_5V_ON_3V3",
                                "CRITICAL",
                                net.get("name", "?"),
                                f"Sinal {drv_v:.1f}V conectado a GPIO {mcu_vcc:.1f}V sem divisor seguro (V_calc={v_div:.2f}V > 3.45V).",
                            )
                        )

        # 3. RULE_ESP32_PIN_CAPS: Input-only / Octal PSRAM N16R8 / Strapping pins
        strapping_set = self.ESP32S3_STRAPPING_WARN if "S3" in mcu_arch else self.ESP32_CLASSIC_STRAPPING_WARN
        for net in nets:
            gpio = net.get("mcu_gpio")
            if gpio is None:
                continue
            gpio = int(gpio)
            direction = str(net.get("direction", "INPUT")).upper()
            if mcu_arch == "ESP32" and gpio in self.ESP32_INPUT_ONLY and direction == "OUTPUT":
                findings.append(
                    AuditFinding(
                        "RULE_ESP32_PIN_CAPS",
                        "CRITICAL",
                        f"GPIO{gpio}",
                        f"GPIO{gpio} e SOMENTE ENTRADA no ESP32 classico e nao pode acionar '{net.get('name')}'.",
                    )
                )
            if "S3" in mcu_arch and gpio in self.ESP32S3_PSRAM_FORBIDDEN:
                findings.append(
                    AuditFinding(
                        "RULE_ESP32_PIN_CAPS",
                        "CRITICAL",
                        f"GPIO{gpio}",
                        f"GPIO{gpio} reservado internamente para Flash/PSRAM Octal SPI no ESP32-S3-N16R8.",
                    )
                )
            if gpio in strapping_set and net.get("heavy_load_or_pull", False):
                findings.append(
                    AuditFinding(
                        "RULE_ESP32_PIN_CAPS",
                        "HIGH",
                        f"GPIO{gpio}",
                        f"GPIO{gpio} e pino de strapping no boot; proibido conectar carga que altere nivel no reset.",
                    )
                )

        # 4. RULE_CURRENT_LIMIT_RESISTOR: LEDs com R < 100 ohm (contador-wokwi r2 = 0.22 ohm)
        for pid, p in parts.items():
            if p.get("type") in {"resistor", "wokwi-resistor"} and p.get("role") == "led_limit":
                val = float(p.get("value_ohm", 0.0))
                if val < 100.0:
                    findings.append(
                        AuditFinding(
                            "RULE_CURRENT_LIMIT_RESISTOR",
                            "CRITICAL",
                            pid,
                            f"Resistor limitador de LED '{pid}' = {val} ohm (< 100 ohm) provoca sobrecorrente.",
                        )
                    )

        # 5. RULE_OSCILLATOR_FEEDBACK, RULE_NO_SURGE_COIL & RULE_MISSING_BUCK_BOOTSTRAP
        for pid, p in parts.items():
            ptype = p.get("type")
            if ptype == "opamp_astable" and not p.get("has_positive_feedback", False):
                findings.append(
                    AuditFinding(
                        "RULE_OSCILLATOR_FEEDBACK",
                        "CRITICAL",
                        pid,
                        f"Oscilador '{pid}' sem malha de histerese positiva; o circuito nao oscilara.",
                    )
                )
            if ptype == "contactor_coil" and p.get("connected_to_spd_surge", False):
                findings.append(
                    AuditFinding(
                        "RULE_NO_SURGE_COIL",
                        "CRITICAL",
                        pid,
                        f"Proibido atribuir desvio de surto SPD atraves da bobina do contator '{pid}'.",
                    )
                )
            if ptype == "buck_converter" and not p.get("has_bootstrap_cap", True):
                findings.append(
                    AuditFinding(
                        "RULE_MISSING_BUCK_BOOTSTRAP",
                        "CRITICAL",
                        pid,
                        f"Regulador Buck '{pid}' sem capacitor bootstrap C_BST (pino BST->SW); chave high-side nao dispara.",
                    )
                )

        passed = not any(f.severity == "CRITICAL" for f in findings)
        return {
            "passed": passed,
            "num_findings": len(findings),
            "findings": [asdict(f) for f in findings],
        }

    def audit_wokwi_diagram(self, diagram: Dict[str, Any]) -> Dict[str, Any]:
        """Analisa diretamente arquivos Wokwi diagram.json quanto a resistores baixos e colisoes GPIO."""
        findings: List[AuditFinding] = []
        parts = diagram.get("parts", [])
        connections = diagram.get("connections", [])
        part_map = {p["id"]: p for p in parts}

        mcu_type = "UNO"
        for p in parts:
            t = p.get("type", "")
            if "esp32-s3" in t:
                mcu_type = "ESP32-S3"
            elif "esp32" in t:
                mcu_type = "ESP32"

        # Verifica resistores Wokwi < 100 ohm ligados a LEDs / displays
        for p in parts:
            if p.get("type") == "wokwi-resistor":
                raw_val = str(p.get("attrs", {}).get("value", "0"))
                try:
                    ohm = float(raw_val)
                except ValueError:
                    ohm = 220.0
                if ohm < 100.0:
                    pid = p["id"]
                    findings.append(
                        AuditFinding(
                            "RULE_CURRENT_LIMIT_RESISTOR",
                            "CRITICAL",
                            pid,
                            f"Resistor Wokwi '{pid}' com valor {ohm} ohm (< 100 ohm) em risco de sobrecorrente.",
                        )
                    )

        # Verifica colisoes de pinos nas conexoes Wokwi
        for c in connections:
            if len(c) < 2:
                continue
            ep_a, ep_b = str(c[0]), str(c[1])
            for ep in (ep_a, ep_b):
                if ":" not in ep:
                    continue
                comp_id, pin_name = ep.split(":", 1)
                ctype = part_map.get(comp_id, {}).get("type", "")
                if "esp32-s3" in ctype and pin_name.isdigit():
                    gpio = int(pin_name)
                    if gpio in self.ESP32S3_PSRAM_FORBIDDEN:
                        findings.append(
                            AuditFinding(
                                "RULE_ESP32_PIN_CAPS",
                                "CRITICAL",
                                f"{comp_id}:{gpio}",
                                f"GPIO{gpio} proibido no Wokwi ESP32-S3-N16R8 (PSRAM/Flash Octal).",
                            )
                        )
                if mcu_type == "ESP32" and "esp32" in ctype and pin_name in {"12", "D12"}:
                    findings.append(
                        AuditFinding(
                            "RULE_ESP32_PIN_CAPS",
                            "HIGH",
                            f"{comp_id}:{pin_name}",
                            "GPIO12 (MTDI) e strapping critico de tensao da Flash no ESP32 classico.",
                        )
                    )

        passed = not any(f.severity == "CRITICAL" for f in findings)
        return {"passed": passed, "num_findings": len(findings), "findings": [asdict(f) for f in findings]}


def main():
    parser = argparse.ArgumentParser(description="Auditor Anti-Alucinacao de Circuitos AEGIS v3.0")
    parser.add_argument("--netlist", type=Path, required=True, help="Caminho para JSON de netlist ou Wokwi diagram.json")
    parser.add_argument("--strict", action="store_true", help="Retorna exit code 1 em caso de achado CRITICAL")
    args = parser.parse_args()

    data = json.loads(args.netlist.read_text(encoding="utf-8"))
    report = CircuitNetlistAuditor().audit(data)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if args.strict and not report["passed"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
