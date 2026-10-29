// Hall sensor frequency meter — Nucleo-F411RE
//
// 3-blade rotor, one magnet per blade -> 3 pulses per revolution.
//
// Wiring:  sensor VCC -> 5V   (open-collector switches need >= 4.5 V)
//          sensor GND -> GND
//          sensor OUT -> A0   (PA0, interrupt-capable)
//
// Instead of counting pulses into a window, we timestamp every edge with
// micros() and subtract consecutive timestamps. The last MAGNETS_PER_REV
// periods span exactly one revolution, so averaging them cancels out
// uneven magnet spacing.
//
// Open the serial monitor at 115200 baud.

#include <Arduino.h>

const uint8_t  HALL_PIN        = A0;       // PA0
const uint8_t  MAGNETS_PER_REV = 3;        // one magnet on each of 3 blades
const uint32_t MIN_PERIOD_US   = 100;      // reject edges closer than this (10 kHz cap)
const uint32_t TIMEOUT_US      = 500000;   // 0.5 s with no pulse -> report 0

volatile uint32_t periods[MAGNETS_PER_REV] = {0};  // ring of the last N periods
volatile uint8_t  pIdx   = 0;    // slot the next period goes into
volatile uint32_t sumUs  = 0;    // running sum of those N periods
volatile uint8_t  filled = 0;    // how many slots hold real data
volatile uint32_t tEdge  = 0;    // micros() timestamp of the last edge
volatile uint32_t edges  = 0;    // total edges, for sanity checking
volatile bool     primed = false;          // false until the first edge arrives
volatile uint32_t periodUs = 0;  // revolution-averaged period; 0 = no data yet

void onEdge() {
  uint32_t now = micros();
  uint32_t dt  = now - tEdge;   // unsigned maths -> survives micros() rollover
  tEdge = now;
  edges++;

  if (!primed) {                // first edge has no predecessor to subtract
    primed = true;
    return;
  }
  if (dt < MIN_PERIOD_US) return;  // too fast to be real: a noise glitch

  sumUs -= periods[pIdx];       // drop the oldest period from the running sum
  periods[pIdx] = dt;           // insert the new one
  sumUs += dt;
  pIdx = (pIdx + 1) % MAGNETS_PER_REV;

  if (filled < MAGNETS_PER_REV) filled++;
  if (filled == MAGNETS_PER_REV) {          // a full revolution is available
    periodUs = sumUs / MAGNETS_PER_REV;     // averaged period per magnet pass
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onEdge, FALLING);
  Serial.println("Hall frequency meter — 3 magnets, one per blade");
}

void loop() {
  uint32_t p = periodUs;

  // No pulse yet, or the rotor has stalled: don't report a stale speed.
  if (p == 0 || (micros() - tEdge) > TIMEOUT_US) {
    Serial.println("f = 0.00 Hz  (no pulses)");
    delay(200);
    return;
  }

  float hz      = 1000000.0f / p;      // 1e6 microseconds per second
  float revPerS = hz / MAGNETS_PER_REV;
  float rpm     = revPerS * 60.0f;
  // float mps  = ...;                 // calibration goes here once you have
                                       // ground-truth data: v = a*revPerS + b

  Serial.print("period=");      Serial.print(p);
  Serial.print(" us\tf=");      Serial.print(hz, 2);
  Serial.print(" Hz\trev/s=");  Serial.print(revPerS, 2);
  Serial.print("\trpm=");       Serial.print(rpm, 1);
  Serial.print("\tedges=");     Serial.println(edges);

  delay(200);   // display rate only — measurement runs in the ISR
}
