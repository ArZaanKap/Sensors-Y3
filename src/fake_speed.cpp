// Fake airspeed generator — test wind-tunnel-logger without a real sensor.
//
// Streams one random speed in m/s per line at 200 Hz over 115200 baud,
// exactly in the format wind-tunnel-logger expects: a plain number per line.
//
// Build & flash:
//   ~/.platformio/penv/bin/pio run -e fake_speed -t upload
//
// Then run wind-tunnel-logger, pick this board's port, and Connect.
// Use Ground truth: Simulated (bench test) if you have no tunnel hardware.

#include <Arduino.h>

static const uint32_t SAMPLE_US = 5000;     // 200 Hz

void setup() {
  Serial.begin(115200);
  // Seed from a floating ADC pin plus micros(), so the sequence is never identical.
  randomSeed(analogRead(A0) ^ (uint32_t)micros());
  delay(200);                               // give the host a moment to attach
}

void loop() {
  // --- exact 200 Hz scheduling (same pattern as logger.cpp) ----------------
  static uint32_t next = 0;
  if (next == 0) next = micros() + SAMPLE_US;
  if ((int32_t)(micros() - next) < 0) return;      // not time yet
  next += SAMPLE_US;
  if ((int32_t)(micros() - next) > (int32_t)SAMPLE_US) {
    next = micros() + SAMPLE_US;                   // fell behind: resync
  }

  // Random speed in the assessed range: 2.000 .. 10.000 m/s
  float v = 2.0f + (random(0, 8001) / 1000.0f);

  // One plain number per line, e.g. "5.432" + CRLF.
  // Use Serial.print(float) rather than snprintf("%.3f"): float printf
  // pulls in newlib's _write() and fails to link on this core.
  Serial.print(v, 3);
  Serial.print("\r\n");
}
