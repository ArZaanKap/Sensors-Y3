// LM35 temperature test — Nucleo-F411RE
//
// Wiring:  LM35 VCC -> 5V,  LM35 GND -> GND,  LM35 OUT -> A0 (PA0)
//
// The LM35 outputs 10 mV per °C, and 0 V at 0 °C.
// Open the serial monitor at 115200 baud and pinch the sensor to warm it.

#include <Arduino.h>

const uint8_t LM35_PIN = A0;  // PA0

void setup() {
  Serial.begin(115200);
  Serial.println("LM35 test");
}

void loop() {
  int raw = analogRead(LM35_PIN);             // 0..4095 on the 12-bit ADC
  float volts = raw * (3.3f / 4095.0f);
  float celsius = volts * 100.0f;             // 1 V = 100 °C
  Serial.print("raw=");
  Serial.print(raw);
  Serial.print("\t");
  Serial.print(volts, 3);
  Serial.print(" V\t");
  Serial.print(celsius, 1);
  Serial.println(" C");
  delay(200);
}
