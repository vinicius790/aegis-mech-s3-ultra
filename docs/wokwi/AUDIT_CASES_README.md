# Casos de Auditoria Wokwi & Correções Reais (`Z13..Z20`)

Este diretório contém o circuito base de CI (`diagram.json`) para `AEGIS-MECH S3` e documenta os **5 bugs críticos reais extraídos dos repositórios Wokwi/Arduino** em `Repositórios do GitHub/`, auditáveis pelo `demo/circuit_netlist_auditor.py`:

1. **`contador-wokwi` (`Z17`):** Resistor `r2` declarado como `"0.22"` Ω (< 100 Ω) causando sobrecorrente severa + ocupação de pinos `0/1` (`RX/TX`) → corrigido para `220 Ω` e GPIO livre (`RULE_CURRENT_LIMIT_RESISTOR`).
2. **`varredura-wokwi` (`Z14`):** Uso de **`GPIO 12` (`MTDI` strapping)** na varredura + drenagem de 7 segmentos num único pino sem driver → removido `GPIO 12` e substituído `delay(2)` por timer (`RULE_ESP32_PIN_CAPS`).
3. **`reguaeletronica-wokwi` (`Z15`):** Tabela `const byte numeros[10]` omitindo dígito `6` (`B01111101`) e `pulseIn` sem timeout → corrigido índice `6` + timeout `38000UL` (`Lab 03`).
4. **`logica-leds` (`Z18`):** Loops `i <= 8` e `i = 9` em vetor de tamanho `8` (*Out-of-Bounds Memory Access*) + 7 LEDs no diagrama → corrigido para `i < sizeof(leds)` + 8º LED (`Lab 02`).
5. **`placar-digital-wokwi` (`Z13`):** Cascata `74HC595` com `delay(200)` bloqueante e uso do pino `TX(1)` → debounce não-bloqueante via `millis()` (`Lab 02 / Lab 05`).
