// SparkFun Sound Detector test — Nucleo-F411RE
//
// Wiring — power it from 3V3, NOT 5V (see the note in the explanation):
//   Sound Detector VCC      -> 3V3
//   Sound Detector GND      -> GND
//   Sound Detector GATE     -> D2   (PA10)  digital: HIGH while sound is present
//   Sound Detector ENVELOPE -> A1   (PA1)   analog:  sound amplitude
//   Sound Detector AUDIO    -> not connected yet (raw waveform, for later)
//
// Turn the on-board potentiometer to set the GATE threshold.
// Open the serial monitor at 115200 baud. Clap, talk, or tap the microphone.

#include <Arduino.h>

const uint8_t ENV_PIN  = A1;    // PA1  -> ENVELOPE
const uint8_t GATE_PIN = PA10;  // D2   -> GATE

uint32_t peak = 0;              // loudest envelope seen since reset

void setup() {
  Serial.begin(115200);
  pinMode(GATE_PIN, INPUT);
  Serial.println("Sound detector test — clap or talk, watch 'env' and 'gate'");
  Serial.println("env\tvolts\tgate\tpeak");
}

void loop() {
  // Sample fast, so we do not miss the peaks of the envelope...
  int raw = analogRead(ENV_PIN);
  if ((uint32_t)raw > peak) peak = raw;

  // ...but report slowly, so it is readable.
  static uint32_t nextPrint = 0;
  if (millis() >= nextPrint) {
    nextPrint = millis() + 100;
    Serial.print(raw);
    Serial.print('\t');
    Serial.print(raw * (3.3f / 4095.0f), 3);
    Serial.print(" V\t");
    Serial.print(digitalRead(GATE_PIN) ? "HIGH" : "low ");
    Serial.print("\t");
    Serial.println(peak);
  }
}
