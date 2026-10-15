// Hall sensor magnet-pass counter — Nucleo-F411RE
//
// Wiring:  sensor VCC -> 5V   (open-collector switches need >= 4.5 V)
//          sensor GND -> GND
//          sensor OUT -> A0   (PA0, interrupt-capable)
//
// Idle = HIGH (internal pull-up). A magnet arriving pulls the pin LOW,
// so each magnet pass = one FALLING edge = one count.
//
// Open the serial monitor at 115200 baud and pass a magnet past the sensor.
// Press the board's black RESET button to start the count from zero again.

#include <Arduino.h>

const uint8_t  HALL_PIN    = A0;  // PA0
const uint16_t DEBOUNCE_MS = 5;   // raise this if one pass counts twice

volatile uint32_t count    = 0;   // written inside the ISR -> must be volatile
volatile uint32_t lastEdge = 0;

void onMagnet() {
  uint32_t now = millis();
  if (now - lastEdge < DEBOUNCE_MS) return;  // ignore ringing/contact chatter
  lastEdge = now;
  count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onMagnet, FALLING);
  Serial.println("Hall counter: pass a magnet, watch the count");
}

void loop() {
  noInterrupts();               // read count atomically (not mid-increment)
  uint32_t c = count;
  interrupts();

  Serial.print("count=");
  Serial.print(c);
  Serial.print("\tlevel=");
  Serial.println(digitalRead(HALL_PIN) == LOW ? "LOW (magnet)" : "HIGH (idle)");
  delay(200);
}
