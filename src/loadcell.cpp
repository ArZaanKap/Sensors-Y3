// loadcell — 200 Hz NAU7802 load-cell logger (load cell only).
//
// Stripped-down logger3.cpp: same I2C wiring and 200 Hz clock, but no mic and
// no AS5048B. The load cell runs at its fastest rate (320 SPS), so every line
// carries a fresh reading. Capture with:
//   ~/.platformio/penv/bin/python tools/log_calibration.py
// (log_serial.py won't work: it only accepts 4-column lines.)
//
// Output (header once, then one line every 5 ms):
//   t_us,load_raw
//   t_us     Nucleo microseconds since boot (gaps > 5000 = missed samples)
//   load_raw NAU7802 24-bit raw counts, or -99999 if the load cell isn't found
//
// With no AS5048B to detect a stuck bus, a stall is detected as "no new
// conversion for 100 ms" (expect one every ~3 ms). The bus is then reset and the
// NAU7802 re-initialised; load_raw is -99999 until it recovers.
//
// Wiring (same as 3_sensor_test.cpp):
//   NAU7802 VIN -> 3V3, GND -> GND, SDA -> D14 (PB9), SCL -> D15 (PB8)
//   AV and DRDY unconnected. Fixed address 0x2A.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

static const uint32_t I2C_SDA = PB9;   // D14
static const uint32_t I2C_SCL = PB8;   // D15

static const int32_t LOAD_ERR = -99999;

static const uint32_t SAMPLE_US = 5000;     // 200 Hz
static const uint32_t STALL_MS  = 100;      // no conversion this long = bus stuck

NAU7802 scale;
bool scaleOK = false;

static inline void enableInternalPullups() {
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));  // PB9 SDA
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));  // PB8 SCL
}

// Configure the NAU7802 for 320 SPS. Returns false if it doesn't answer.
static bool initScale() {
  if (!scale.begin(Wire)) return false;
  scale.setSampleRate(NAU7802_SPS_320);
  scale.calibrateAFE();                      // required after changing the rate
  return true;
}

// Re-initialise the I2C peripheral and the NAU7802.
// Wire.begin() itself clocks SCL to free a slave that is holding SDA low.
static uint32_t busResets = 0;
static void resetBus() {
  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL);
  enableInternalPullups();
  scaleOK = initScale();
  busResets++;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  enableInternalPullups();
  delay(100);

  Serial.println("# loadcell: 200 Hz NAU7802");

  scaleOK = initScale();
  if (scaleOK) {
    // Self-check: count conversions in 1 s (expect ~320).
    uint32_t n = 0, t0 = millis();
    while (millis() - t0 < 1000) {
      if (scale.available()) { scale.getReading(); n++; }
    }
    Serial.print("# NAU7802 OK, measured rate ");
    Serial.print(n);
    Serial.println(" /s (expect ~320)");
  } else {
    Serial.println("# NAU7802 NOT FOUND (0x2A) - check 3V3/GND/SDA(D14)/SCL(D15)");
  }

  Serial.println("t_us,load_raw");
}

void loop() {
  static uint32_t next = 0;
  if (next == 0) next = micros() + SAMPLE_US;
  if ((int32_t)(micros() - next) < 0) return;
  next += SAMPLE_US;
  if ((int32_t)(micros() - next) > (int32_t)SAMPLE_US) next = micros() + SAMPLE_US;

  uint32_t t = micros();

  static int32_t load = LOAD_ERR;
  static uint32_t lastFresh = millis();
  if (scaleOK && scale.available()) {
    load = scale.getReading();
    lastFresh = millis();
  } else if (millis() - lastFresh > STALL_MS) {
    // No conversion for 100 ms: bus stuck or NAU7802 dropped out. Retry
    // at most every STALL_MS (also covers "not found at boot" -> hot-plug).
    load = LOAD_ERR;
    resetBus();
    lastFresh = millis();
  }

  // Report bus resets as a comment line (ignored by the logger's CSV).
  static uint32_t reported = 0;
  if (busResets != reported) {
    reported = busResets;
    Serial.print(scaleOK ? "# i2c bus reset " : "# i2c bus reset (NAU7802 not found) ");
    Serial.println(busResets);
  }

  char buf[32];
  int len = snprintf(buf, sizeof(buf), "%lu,%ld\n", (unsigned long)t, (long)load);
  Serial.write((const uint8_t *)buf, len);
}
