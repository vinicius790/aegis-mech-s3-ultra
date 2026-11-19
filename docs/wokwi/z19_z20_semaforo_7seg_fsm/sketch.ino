// Z19 + Z20 Semaforo FSM O(1) Nao-Bloqueante + Matriz 7-Segmentos
const int LED_G = 11, LED_Y = 12, LED_R = 13;
unsigned long stateStartMs = 0;
int state = 0;
void setup() { pinMode(LED_G, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_R, OUTPUT); }
void loop() {
  const unsigned long dur = (state == 1) ? 1000UL : 2000UL;
  if (millis() - stateStartMs >= dur) { stateStartMs = millis(); state = (state + 1) % 3; }
  digitalWrite(LED_G, state == 0); digitalWrite(LED_Y, state == 1); digitalWrite(LED_R, state == 2);
}
