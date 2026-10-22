// AS5048B magnetic angle sensor -- Nucleo-F411RE bring-up test
//
// Part: AS5048B-TS_EK_AB adapter board, MPN 990600491 (ams/OSRAM).
// It is a 14-bit magnetic rotary position sensor with an I2C interface.
// A small diametrically-magnetised magnet turns above the chip package and
// this sketch prints the angle it measures.
//
// --- Wiring (I2C, 3.3 V logic) ---------------------------------------------
// The AS5048B is a 3.3 V part. Power the whole adapter board from the
// Nucleo's 3V3 rail and leave the board's 5V pin unconnected. The AB board
// carries NO I2C pull-up resistors (only decoupling caps), so fit a 4.7 k
// resistor from SDA to 3V3 and one from SCL to 3V3. The F411 I2C pins are
// configured open-drain with no internal pull-up, so these are required for
// a reliable bus (short flying leads sometimes work without them, but fit
// them anyway).
//~/.platformio/penv/bin/pio run -e as5048b -t upload
// ~/.platformio/penv/bin/pio device monitor -b 115200             

//   Adapter pin (P1)   Signal     Nucleo-F411RE
//   ----------------------------------------------------------------------
//   1                  GND   -->  GND
//   4                  SCL/SCK --> D15  (PB8, I2C1_SCL)
//   5                  SDA/CSn --> D14  (PB9, I2C1_SDA)
//   7                  3.3V  -->  3V3        (powers the board)
//   8                  5V    -->  not connected
//   2                  A2/MISO -> GND  \  tie both address pins to GND
//   3                  A1/MOSI -> GND  /  -> default I2C address 0x40
//   6                  PWM     --> not connected (optional PWM output)
//
// Match the labels printed on the adapter silkscreen if your pin numbering
// differs. SDA/SCL are 3.3 V only: never feed 5 V into them.
//
// Mechanical: the 2-pole (diametric) magnet must sit centred over the chip
// with a 0.5-2 mm air gap, held by a NON-magnetic holder (brass/alu/plastic).
//
// Open the serial monitor at 115200 baud and turn the magnet by hand.

#include <Arduino.h>
#include <Wire.h>

// I2C pins: STM32F411 I2C1 on the Arduino D14/D15 header = PB9/PB8.
// These are also the Nucleo-F411RE default Wire pins, so the lines below
// just make it explicit. (Change them if you wire the sensor elsewhere.)
static const uint32_t I2C_SDA = PB9;   // Arduino D14
static const uint32_t I2C_SCL = PB8;   // Arduino D15

static const uint8_t AS5048B_ADDR = 0x40;   // default 7-bit address (A1=A2=GND)

// Readout registers, per ams application note "AS5048B - I2C sensor readout".
// They auto-increment, so we can grab all six in one burst read from 0xFA.
enum {
  REG_AGC     = 0xFA,   // 8-bit automatic gain control
  REG_DIAG    = 0xFB,   // diagnostic flags in the low nibble
  REG_MAG_MSB = 0xFC,   // magnitude <13:6>
  REG_MAG_LSB = 0xFD,   // magnitude <5:0>
  REG_ANG_MSB = 0xFE,   // angle     <13:6>
  REG_ANG_LSB = 0xFF,   // angle     <5:0>
  N_REGS      = 6
};

static const float DEG_PER_LSB = 360.0f / 16384.0f;   // 14-bit -> degrees

static uint8_t gAddr = AS5048B_ADDR;   // address actually in use (set by scan)

// The F411 I2C pins and the AB board have no pull-up resistors. External
// 4.7k resistors to 3V3 are the correct fix; as a fallback for a quick test
// with no resistors to hand, enable the MCU's weak (~40k) internal pull-ups.
// This is harmless next to external resistors, so it is on by default.
#define USE_INTERNAL_PULLUPS 1

static inline void enableInternalPullups() {
  // PUPDR field = 01 means pull-up; valid because the I2C pins are open-drain AF.
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));  // PB9 SDA
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));  // PB8 SCL
}

// Burst-read `n` bytes starting at register `start`.
static bool readRegs(uint8_t addr, uint8_t start, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  if (Wire.endTransmission(false) != 0) return false;   // repeated START
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Print every address that answers and remember the first one found.
static uint8_t scanBus() {
  uint8_t found = 0;
  for (uint8_t a = 1; a < 0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      if (found == 0) gAddr = a;
      found++;
      Serial.print("  device at 0x");
      if (a < 0x10) Serial.print('0');
      Serial.println(a, HEX);
    }
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("AS5048B angle sensor test");
  Serial.println("I2C: SDA = PB9 / D14,  SCL = PB8 / D15  (100 kHz)");

  Wire.begin(I2C_SDA, I2C_SCL);
#if USE_INTERNAL_PULLUPS
  enableInternalPullups();
  Serial.println("(weak internal pull-ups enabled; fit 4.7k to 3V3 for reliability)");
#endif

  Serial.println("I2C scan:");
  if (scanBus() == 0) {
    Serial.println("  none found -> check 3V3, GND, SDA(D14), SCL(D15),");
    Serial.println("                and the 4.7k pull-ups on SDA/SCL to 3V3");
  }
}

void loop() {
  uint8_t b[N_REGS];
  if (!readRegs(gAddr, REG_AGC, b, N_REGS)) {
    // Re-scan now and then so a late fix (power, wiring) is picked up.
    static uint32_t nextScan = 0;
    if (millis() >= nextScan) {
      nextScan = millis() + 2000;
      Serial.print("read failed -- rescanning (addr 0x");
      if (gAddr < 0x10) Serial.print('0');
      Serial.print(gAddr, HEX);
      Serial.println("):");
      if (scanBus() == 0) {
        Serial.println("  no device: check 3V3, GND, SDA(D14), SCL(D15), pull-ups, A1+A2 to GND");
      } else {
        Serial.print("  using address 0x");
        if (gAddr < 0x10) Serial.print('0');
        Serial.println(gAddr, HEX);
      }
    }
    delay(200);
    return;
  }

  uint8_t  agc  = b[0];                                        // 0 = strong field, 255 = weak
  uint8_t  diag = b[1] & 0x0F;                                 // OCF/COF + comp flags
  uint16_t mag  = ((uint16_t)b[2] << 6) | (b[3] & 0x3F);       // 14-bit magnitude
  uint16_t raw  = ((uint16_t)b[4] << 6) | (b[5] & 0x3F);       // 14-bit angle
  float    deg  = raw * DEG_PER_LSB;

  bool ocf = diag & 0x01;   // Offset Compensation Finished: should be 1
  bool cof = diag & 0x02;   // CORDIC Overflow: should be 0 (field too weak/strong)

  Serial.print("raw=");        Serial.print(raw);
  Serial.print("\tdeg=");      Serial.print(deg, 2);
  Serial.print("\tmag=");      Serial.print(mag);
  Serial.print("\tagc=");      Serial.print(agc);
  Serial.print("\tdiag=0x");   Serial.print(diag, HEX);
  if (!ocf) Serial.print("  OCF! (no valid angle)");
  if (cof)  Serial.print("  COF! (magnet out of range)");
  Serial.println();

  delay(100);   // display rate only; the sensor itself updates far faster
}
