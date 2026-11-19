// Z13 Placar Digital 74HC595 Corrigido (Arduino Uno R3 - Sem usar pino 1 TX e debounce millis() nao-bloqueante)
const int PIN_DS = 8, PIN_STCP = 9, PIN_SHCP = 10, PIN_BTN = 7;
unsigned long lastDebounceMs = 0;
int score = 0;
void setup() { pinMode(PIN_DS, OUTPUT); pinMode(PIN_STCP, OUTPUT); pinMode(PIN_SHCP, OUTPUT); pinMode(PIN_BTN, INPUT_PULLUP); }
void loop() {
  if (digitalRead(PIN_BTN) == LOW && (millis() - lastDebounceMs) > 40) {
    lastDebounceMs = millis();
    score = (score + 1) % 100;
  }
}
