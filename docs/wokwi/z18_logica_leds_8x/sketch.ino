// Z18 Logica de 8 LEDs + Z10 Psicrometria IoT em ESP8266 NodeMCU: indices estritos 0..7 (sem i<=8 nem i=9 Out-of-Bounds!)
const uint8_t leds[8] = {4, 5, 12, 13, 14, 15, 16, 2};
void setup() { for (int i = 0; i < 8; ++i) pinMode(leds[i], OUTPUT); }
void loop() {
  for (int i = 0; i < 8; ++i) { digitalWrite(leds[i], HIGH); }
  for (int i = 7; i >= 0; --i) { digitalWrite(leds[i], LOW); }
}
