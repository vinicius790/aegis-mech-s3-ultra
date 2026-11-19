# Lab 12 — Integração Completa, Cockpit SCADA SSE, Agente ROSA Auditor Anti-Alucinação & WebFlasher

- **Origens Integradas:** `awesome-ai-robotics`, `COMET` Flasher, `lab-uc CP210x`, `A37E8C2F`, `EAFA7E0D`, `B84DC409`, `BA21951D`, `35806826`.

## Desafio Obrigatório Anti-Alucinação GenAI
Submeter a [circuit_netlist_auditor.py](../../demo/circuit_netlist_auditor.py) **os 5 esquemáticos defeituosos reais**:
1. `A37E8C2F`: astável `LM358` sem malha de realimentação positiva → bloqueado por `RULE_OSCILLATOR_FEEDBACK`;
2. `EAFA7E0D`: bobina de contator rotulada como caminho SPD → bloqueada por `RULE_NO_SURGE_COIL`;
3. `lab-uc Exp 16`: carga indutiva sem diodo *flyback* → bloqueada por `RULE_NO_FLYBACK`;
4. `lab-uc Exp 12`: `ECHO 5V` / pull-up direto em pino `3,3 V` → bloqueada por `RULE_LOGIC_LEVEL_5V_ON_3V3`;
5. `contador-wokwi`: resistor `0,22 Ω` em display LED → bloqueado por `RULE_CURRENT_LIMIT_RESISTOR`.

## Roteiro Integrado Ponta-a-Ponta
1. Conferir `data/sintetico/modelo/manifest.json` (`num_params = 4879`, `SHA-256` verdadeiro) e [docs/web_flasher/manifest.json](../web_flasher/manifest.json) (`boot_app0.bin @ 0xe000`).
2. Abrir `py demo/app_web.py` e validar todos os **5 painéis** (Arena XAI, SCADA live SSE 30 Hz, Chicote SVG, Círculo Unitário `Faddeev-LeVerrier`, *Tiny Logic Zoom*).
3. Executar `py demo/diagnostic_agent.py` verificando AST firewall *read-only* e zero métodos de atuação.
4. Concluir matriz completa `py -m pytest -v` aprovando os **7 Níveis de V&V**.

## Quality Gate Final (`Certificação v3.0`)
- 100% dos testes `pytest` passando sem falhas nem regressões; Auditor retorna `passed == True` em `aegis_mech_s3_netlist.json` e `docs/wokwi/diagram.json`; bloqueio `100% CRITICAL` nos 5 circuitos defeituosos.
