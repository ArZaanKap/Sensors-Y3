// 3_sensor_test — bring-up test: mic + AS5048B + NAU7802 load cell, all at once.
//
// Reads all three sensors and prints one CSV line at ~20 Hz so you can verify
// the whole wiring works together before writing the real calibration code.
//
// Output (one line per sample, plus a header):
//   mic,gate,load_raw,ang_raw
//   mic      microphone envelope, raw 0..4095
//   gate     0 or 1 (digital "sound present" from the Sound Detector)
//   load_raw NAU7802 24-bit raw counts, or -99999 if the load cell isn't found
//   ang_raw  14-bit AS5048B angle 0..16383, or -1 if the I2C read failed
//
// This same CSV is also capture-able by tools/log_calibration.py.
//
// --- Wiring ----------------------------------------------------------------
//   Sound Detector VCC      -> 3V3        (NOT 5V)
//   Sound Detector GND      -> GND
//   Sound Detector ENVELOPE -> A1   (PA1)
//   Sound Detector GATE     -> D2   (PA10)
//
//   AS5048B VCC (pin 7)     -> 3V3
//   AS5048B GND (pin 1)     -> GND
//   AS5048B SCL (pin 4)     -> D15  (PB8)  I2C1 SCL
//   AS5048B SDA (pin 5)     -> D14  (PB9)  I2C1 SDA
//   AS5048B A1+A2           -> GND         (address 0x40)
//   (fit 4.7k pull-ups SDA/SCL -> 3V3, or the weak internal ones are enabled)
//
//   NAU7802 VIN (power in) -> 3V3      <- this is the board's VCC; use VIN, NOT AV
//   NAU7802 GND             -> GND
//   NAU7802 SDA             -> D14  (PB9)  shares the I2C1 bus
//   NAU7802 SCL             -> D15  (PB8)  (fixed address 0x2A)
//   NAU7802 AV  (AVDD)      -> leave UNCONNECTED (load-cell excitation output)
//   NAU7802 DRDY            -> leave UNCONNECTED (optional data-ready; code polls it)

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

// I2C1 pins (shared by AS5048B and NAU7802)
static const uint32_t I2C_SDA = PB9;   // Arduino D14
static const uint32_t I2C_SCL = PB8;   // Arduino D15

// Microphone (SparkFun Sound Detector)
static const uint8_t MIC_PIN  = A1;    // PA1  -> ENVELOPE
static const uint8_t GATE_PIN = PA10;  // D2   -> GATE

static const uint8_t AS5048B_ADDR = 0x40;   // default (A1=A2=GND)
enum { REG_ANG_MSB = 0xFE, REG_ANG_LSB = 0xFF };
static const uint16_t ANGLE_ERR = 0xFFFF;

static const int32_t LOAD_ERR = -99999;     // sentinel: NAU7802 not present

NAU7802 scale;          // the load-cell ADC (library holds address 0x2A)
bool scaleOK = false;

// Weak internal pull-ups on PB9/PB8 as a fallback next to external 4.7k ones.
static inline void enableInternalPullups() {
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));  // PB9 SDA
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));  // PB8 SCL
}

// Print every I2C address that ACKs, so wiring faults are obvious.
static void scanI2C() {
  int found = 0;
  for (uint8_t a = 1; a < 0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("#   device at 0x");
      if (a < 0x10) Serial.print('0');
      Serial.println(a, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println("#   none found - check 3V3/GND/SDA(D14)/SCL(D15) and pull-ups");
}

// Read the 14-bit AS5048B angle; ANGLE_ERR if the bus did not answer.
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

  Serial.println("# 3_sensor_test: mic + AS5048B + NAU7802 load cell");
  Serial.println("# I2C scan (expect 0x40 = AS5048B, 0x2A = NAU7802):");
  scanI2C();

  // begin(Wire) does a full init + AFE calibration and returns false if absent.
  scaleOK = scale.begin(Wire);
  if (scaleOK) {
    Serial.println("# NAU7802 OK - gain 128, 80 SPS (raw 24-bit counts below)");
  } else {
    Serial.println("# NAU7802 NOT FOUND - check 3V3/GND/SDA(D14)/SCL(D15) (addr 0x2A)");
  }

  Serial.println("mic,gate,load_raw,ang_raw");   // CSV header (last text line before data)
}

void loop() {
  int mic  = analogRead(MIC_PIN);
  int gate = digitalRead(GATE_PIN) ? 1 : 0;

  uint16_t ang = readAngle();

  // Only read when a fresh conversion is ready; otherwise keep the last value.
  static int32_t load = LOAD_ERR;
  if (scaleOK && scale.available()) {
    load = scale.getReading();
  }

  Serial.print(mic);
  Serial.print(',');
  Serial.print(gate);
  Serial.print(',');
  Serial.print(load);
  Serial.print(',');
  Serial.print((ang == ANGLE_ERR) ? -1 : (int)ang);
  Serial.print("\r\n");

  delay(50);   // ~20 Hz is plenty for a bring-up check (NAU7802 converts at 80 SPS)
}
