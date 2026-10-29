// Multi-sensor data logger -- Nucleo-F411RE
//
// Logs two sensors at a fixed 200 Hz and streams one compact CSV line per
// sample over USB serial (115200 baud). The host script tools/log_serial.py
// captures the stream to a timestamped .csv file.
//
// A ground-truth sensor (Dwyer + pitot) will be added later; extend the
// CSV line below when it is wired in.
//
// --- Wiring ----------------------------------------------------------------
//   Mic ENVELOPE   -> A1  (PA1)     analog: sound amplitude
//   Mic GATE       -> D2  (PA10)    digital: HIGH while sound exceeds threshold
//   AS5048B SDA    -> D14 (PB9)  ┐ I2C1
//   AS5048B SCL    -> D15 (PB8)  ┘  (fit 4.7k pull-ups to 3V3)
//   AS5048B: power from 3V3, GND, A1+A2 to GND (address 0x40)
//
// --- Output line -----------------------------------------------------------
//   t_us,mic,gate,ang_raw\n
//     t_us     microseconds since boot (exact sample spacing)
//     mic      envelope, raw 0..4095
//     gate     0 or 1
//     ang_raw  14-bit AS5048B angle 0..16383, or -1 if the I2C read failed
//   The host script adds a wall-clock time column and converts ang_raw to deg.

#include <Arduino.h>
#include <Wire.h>

static const uint32_t I2C_SDA = PB9;   // Arduino D14
static const uint32_t I2C_SCL = PB8;   // Arduino D15

static const uint8_t AS5048B_ADDR = 0x40;   // default (A1=A2=GND)
static const uint8_t MIC_PIN  = A1;         // PA1  -> envelope
static const uint8_t GATE_PIN = PA10;       // D2   -> gate

static const uint32_t SAMPLE_US = 5000;     // 200 Hz

// AS5048B readout registers (auto-increment).
enum { REG_ANG_MSB = 0xFE, REG_ANG_LSB = 0xFF };

static const uint16_t ANGLE_ERR = 0xFFFF;   // sentinel for a failed read

// The Nucleo-F411RE I2C pins have no pull-ups; enable the weak internal ones
// as a fallback alongside the external 4.7k resistors.
static inline void enableInternalPullups() {
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));  // PB9 SDA
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));  // PB8 SCL
}

// Read the 14-bit angle as one 2-byte burst; ANGLE_ERR if the bus did not answer.
// Angle = (reg[0xFE] << 6) | (reg[0xFF] & 0x3F), per ams AN5048B.
static uint16_t readAngle() {
  Wire.beginTransmission(AS5048B_ADDR);
  Wire.write(REG_ANG_MSB);
  if (Wire.endTransmission(false) != 0) return ANGLE_ERR;          // repeated START
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

  delay(300);                       // give the host time to attach
  Serial.println(F("t_us,mic,gate,ang_raw"));   // header once
}

void loop() {
  // --- exact 200 Hz scheduling -------------------------------------------
  static uint32_t next = 0;
  if (next == 0) next = micros() + SAMPLE_US;
  if ((int32_t)(micros() - next) < 0) return;    // not time yet
  next += SAMPLE_US;
  if ((int32_t)(micros() - next) > (int32_t)SAMPLE_US) {
    next = micros() + SAMPLE_US;                 // fell behind: resync
  }

  // --- sample -------------------------------------------------------------
  uint32_t t  = micros();
  int      mic  = analogRead(MIC_PIN);
  int      gate = digitalRead(GATE_PIN) ? 1 : 0;
  uint16_t ang  = readAngle();

  // --- one serial write per sample (cheap and atomic enough) --------------
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "%lu,%d,%d,%d\n",
                   (unsigned long)t, mic, gate,
                   (ang == ANGLE_ERR) ? -1 : (int)ang);
  Serial.write((const uint8_t *)buf, n);
}
