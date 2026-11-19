# -*- coding: utf-8 -*-
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))
sys.path.insert(0, str(Path(__file__).parent.parent / "demo"))

from diagnostic_agent import ReadOnlyDiagnosticAgent

def test_agent_read_only_tools():
    agent = ReadOnlyDiagnosticAgent()
    health = agent.get_system_health()
    assert health["i2c_ok"] is True
    assert health["vbus_12v"] > 11.0
    
    parity = agent.verify_golden_parity()
    assert parity["passed"] is True
    assert parity["max_abs_diff_int32"] == 0

def test_agent_no_actuator_mutation():
    agent = ReadOnlyDiagnosticAgent()
    # Invariante 5 e FMEA F14: Agente jamais possui metodos de escrita ou atuacao
    proibidos = ["unlock", "lock", "write", "set_pwm", "force_open"]
    for p in proibidos:
        assert not hasattr(agent, p), f"VULNERABILIDADE: Metodo mutavel {p} encontrado no agente!"
