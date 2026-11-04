// logger3 — 200 Hz calibration logger: mic + AS5048B + NAU7802 load cell.
//
// Same wiring as 3_sensor_test.cpp, but sampled on a fixed 200 Hz clock (like
// logger.cpp) with the load cell at its fastest rate (320 SPS), so every line
// carries a fresh reading from every sensor. Capture with:
//   ~/.platformio/penv/bin/python tools/log_calibration.py --giga
//
// Output (header once, then one line every 5 ms):
//   t_us,mic,gate,load_raw,ang_raw
//   t_us     Nucleo microseconds since boot (gaps > 5000 = missed samples)
//   mic      microphone envelope, raw 0..4095
//   gate     0 or 1
//   load_raw NAU7802 24-bit raw counts, or -99999 if the load cell isn't found
//   ang_raw  14-bit AS5048B angle 0..16383, or -1 if the I2C read failed
//
// Timing budget per 5 ms sample at 100 kHz I2C: angle read ~0.5 ms, load-cell
// check + read ~1 ms, serial line (~32 bytes) sent by interrupt in ~2.8 ms.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

static const uint32_t I2C_SDA = PB9;   // D14
static const uint32_t I2C_SCL = PB8;   // D15
static const uint8_t MIC_PIN  = A1;    // PA1  -> ENVELOPE
static const uint8_t GATE_PIN = PA10;  // D2   -> GATE

static const uint8_t AS5048B_ADDR = 0x40;
enum { REG_ANG_MSB = 0xFE };
static const uint16_t ANGLE_ERR = 0xFFFF;
static const int32_t LOAD_ERR = -99999;

static const uint32_t SAMPLE_US = 5000;     // 200 Hz

NAU7802 scale;
bool scaleOK = false;

static inline void enableInternalPullups() {
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));  // PB9 SDA
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));  // PB8 SCL
}

static uint16_t readAngle() {
  Wire.beginTransmission(AS5048B_ADDR);
  Wire.write(REG_ANG_MSB);
  if (Wire.endTransmission(false) != 0) return ANGLE_ERR;
  if (Wire.requestFrom(AS5048B_ADDR, (uint8_t)2) != 2) return ANGLE_ERR;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 6) | (lsb & 0x3F);
}

void setup() {
  Serial.begin(115200);
  pinMode(GATE_PIN, INPUT);
  Wire.begin(I2C_SDA, I2C_SCL);
  enableInternalPullups();
  delay(100);

  Serial.println("# logger3: 200 Hz mic + AS5048B + NAU7802");
  Serial.println(readAngle() == ANGLE_ERR ? "# AS5048B NOT FOUND (0x40)" : "# AS5048B OK");

  scaleOK = scale.begin(Wire);
  if (scaleOK) {
    scale.setSampleRate(NAU7802_SPS_320);
    scale.calibrateAFE();                    // required after changing the rate
    // Self-check: count conversions in 1 s (expect ~320).
    uint32_t n = 0, t0 = millis();
    while (millis() - t0 < 1000) {
      if (scale.available()) { scale.getReading(); n++; }
    }
    Serial.print("# NAU7802 OK, measured rate ");
    Serial.print(n);
    Serial.println(" /s (expect ~320)");
  } else {
    Serial.println("# NAU7802 NOT FOUND (0x2A)");
  }

  Serial.println("t_us,mic,gate,load_raw,ang_raw");
}

void loop() {
  static uint32_t next = 0;
  if (next == 0) next = micros() + SAMPLE_US;
  if ((int32_t)(micros() - next) < 0) return;
  next += SAMPLE_US;
  if ((int32_t)(micros() - next) > (int32_t)SAMPLE_US) next = micros() + SAMPLE_US;

  uint32_t t = micros();
  int mic  = analogRead(MIC_PIN);
  int gate = digitalRead(GATE_PIN) ? 1 : 0;
  uint16_t ang = readAngle();

  static int32_t load = LOAD_ERR;
  if (scaleOK && scale.available()) load = scale.getReading();

  char buf[56];
  int len = snprintf(buf, sizeof(buf), "%lu,%d,%d,%ld,%d\n",
                     (unsigned long)t, mic, gate, (long)load,
                     (ang == ANGLE_ERR) ? -1 : (int)ang);
  Serial.write((const uint8_t *)buf, len);
}
