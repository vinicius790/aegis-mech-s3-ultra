// Z15 Regua Eletronica HC-SR04 Corrigida (ESP32 Classic): LUT com digito 6 (0x7D) + pulseIn timeout 38000UL + c(T,RH)
const uint8_t numeros[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
const int PIN_TRIG = 4, PIN_ECHO = 5;
void setup() { Serial.begin(115200); pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT); }
void loop() {
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 38000UL);
  float dist_cm = (dur * 0.03463f) * 0.5f;
  Serial.println(dist_cm);
}
