// Z14 Varredura POV 7-Seg Corrigida (ESP32-S3: GPIO 12 MTDI substituido por GPIO 18 seguro + timer micros())
const uint8_t kSegPins[7] = {4, 5, 6, 7, 8, 9, 10};
const uint8_t kDigPins[4] = {18, 19, 20, 21}; // Sem GPIO 12 (MTDI)!
unsigned long lastScanUs = 0;
uint8_t curDig = 0;
void setup() { for(int i=0;i<7;++i) pinMode(kSegPins[i], OUTPUT); for(int i=0;i<4;++i) pinMode(kDigPins[i], OUTPUT); }
void loop() {
  if (micros() - lastScanUs >= 2000UL) {
    lastScanUs = micros();
    digitalWrite(kDigPins[curDig], HIGH);
    curDig = (curDig + 1) & 3;
    digitalWrite(kDigPins[curDig], LOW);
  }
}
