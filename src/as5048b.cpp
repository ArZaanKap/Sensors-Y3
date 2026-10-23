// AS5048B magnetic angle sensor -- Nucleo-F411RE angle + RPM logger
//
// Part: AS5048B-TS_EK_AB adapter board, MPN 990600491 (ams/OSRAM).
// It is a 14-bit magnetic rotary position sensor with an I2C interface.
// A magnet turns above the chip package; this sketch samples the angle at a
// fixed rate (SAMPLE_HZ), unwraps it into total revolutions, and reports RPM.
//
// --- Build / upload / log (Windows paths; on macOS use penv/bin) -------------
//   ~/.platformio/penv/Scripts/pio run -e as5048b -t upload
//   ~/.platformio/penv/Scripts/python tools/serial_log.py          (CSV in logs/)
//   ~/.platformio/penv/Scripts/pio device monitor -b 921600         (just watch)
//
// --- Serial commands (send a single character) ------------------------------
//   s  summary: one CSV line per report (REPORT_HZ), default
//   r  raw: one CSV line per sample (SAMPLE_HZ) for offline analysis
//   q  quiet: no streaming output (sampling and rev counting continue)
//   z  zero the revolution counter and clock
//   ?  print status and the column headers
//
// --- Wiring (I2C, 3.3 V logic) ---------------------------------------------
// The AS5048B is a 3.3 V part. Power the whole adapter board from the
// Nucleo's 3V3 rail and leave the board's 5V pin unconnected. The AB board
// carries NO I2C pull-up resistors (only decoupling caps), so fit a 4.7 k
// resistor from SDA to 3V3 and one from SCL to 3V3. The F411 I2C pins are
// configured open-drain with no internal pull-up, so these are required for
// a reliable bus (short flying leads sometimes work without them, but fit
// them anyway).

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
// Mechanical: the magnet's N-S axis must lie PARALLEL to the chip surface
// (a diametric disc, or an axial magnet laid on its side), centred over the
// chip, ~0.5-2 mm away, and spun about the vertical axis through the chip.

#include <Arduino.h>
#include <Wire.h>

// --- Configuration ---------------------------------------------------------
#define SAMPLE_HZ   1000     // angle reads per second; must stay > 2x the
                             // rotation rate (rev/s), ideally > 6x
#define REPORT_HZ   10       // summary lines per second
#define I2C_HZ      100000   // 400000 needs real 4.7k pull-ups, not internal
#define SERIAL_BAUD 921600   // raw mode needs ~25 bytes x SAMPLE_HZ per second
#define MAG_MIN     1000     // below this CORDIC magnitude the angle is noise

enum OutMode { OUT_QUIET, OUT_SUMMARY, OUT_RAW };
static OutMode gMode = OUT_SUMMARY;   // power-up output mode

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

static const int32_t COUNTS_PER_REV = 16384;                 // 14-bit angle
static const float   DEG_PER_LSB    = 360.0f / COUNTS_PER_REV;
static const uint32_t SAMPLE_US     = 1000000UL / SAMPLE_HZ;
static const uint32_t REPORT_US     = 1000000UL / REPORT_HZ;

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

// I2C bus recovery. An MCU reset (e.g. re-flashing) does not power-cycle the
// sensor, so a reset mid-read can leave it stuck part-way through a byte.
// It may be holding SDA high or low at that moment, so always clock SCL 9
// times (finishing any byte) and then send a STOP. Run before Wire.begin.
static void recoverBus() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  Serial.print("# bus idle levels: SDA=");  Serial.print(digitalRead(I2C_SDA));
  Serial.print(" SCL=");                    Serial.println(digitalRead(I2C_SCL));
  if (digitalRead(I2C_SCL) == LOW) {
    Serial.println("#   SCL held low -> check wiring / power (cannot recover)");
    return;
  }
  bool sdaWasLow = digitalRead(I2C_SDA) == LOW;

  pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  }
  // STOP: SDA low -> high while SCL is high.
  pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  if (sdaWasLow) {
    Serial.print("#   SDA was stuck low; after 9 clocks + STOP SDA=");
    Serial.println(digitalRead(I2C_SDA));
  }
}

// (Re)start the I2C peripheral from a clean bus. Also used at run time: a
// noise glitch (e.g. from a drill motor) can lock the F4 I2C block up until
// it is re-initialised.
static void initBus() {
  recoverBus();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
#if USE_INTERNAL_PULLUPS
  enableInternalPullups();
#endif
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
      Serial.print("#   device at 0x");
      if (a < 0x10) Serial.print('0');
      Serial.println(a, HEX);
    }
  }
  return found;
}

// --- One sample --------------------------------------------------------------
struct Sample {
  uint32_t t_us;
  uint16_t raw;    // 14-bit angle
  uint16_t mag;    // 14-bit CORDIC magnitude
  uint8_t  agc;    // 0 = strong field, 255 = weak
  uint8_t  diag;   // bit0 OCF, bit1 COF, bit2 comp low, bit3 comp high
};

static bool readSample(Sample &s) {
  uint8_t b[N_REGS];
  s.t_us = micros();
  if (!readRegs(gAddr, REG_AGC, b, N_REGS)) return false;
  s.agc  = b[0];
  s.diag = b[1] & 0x0F;
  s.mag  = ((uint16_t)b[2] << 6) | (b[3] & 0x3F);
  s.raw  = ((uint16_t)b[4] << 6) | (b[5] & 0x3F);
  return true;
}

// The angle is trustworthy once offset compensation is done, the CORDIC has
// not overflowed and there is a real field. This skips the junk readings
// while the magnet is being brought up to the sensor.
static inline bool sampleValid(const Sample &s) {
  return (s.diag & 0x01) && !(s.diag & 0x02) && s.mag >= MAG_MIN;
}

static inline bool sampleWeak(const Sample &s) {
  return s.diag & 0x0C;   // AGC pinned at either end
}

// --- Revolution counting / RPM state -----------------------------------------
static int64_t  gCounts   = 0;       // unwrapped angle since last zero
static bool     gSynced   = false;   // previous sample valid -> can unwrap
static uint16_t gPrevRaw  = 0;
static uint32_t gPrevUs   = 0;
static uint32_t gZeroUs   = 0;       // clock origin for printed timestamps

// Per-report-window accumulators.
static int32_t  wCounts   = 0;       // unwrapped counts over valid sample pairs
static uint32_t wSpanUs   = 0;       // time covered by those pairs
static uint16_t wSamples  = 0, wValid = 0, wWeak = 0, wErrors = 0;
static uint32_t wNextUs   = 0;
static Sample   gLast     = {};      // most recent good read, for the summary

static uint32_t gI2cErrors = 0, gOverruns = 0, gBusResets = 0;

static void zeroCounter() {
  gCounts = 0;
  gZeroUs = micros();
  Serial.println("# zeroed revolutions and clock");
}

static void printHeader() {
  switch (gMode) {
    case OUT_SUMMARY: Serial.println("t_ms,rpm,revs,deg,agc,mag,valid_pct,weak_pct,i2c_err"); break;
    case OUT_RAW:     Serial.println("t_us,raw,mag,agc,diag,valid"); break;
    case OUT_QUIET:   break;
  }
}

static void printStatus() {
  Serial.print("# mode=");
  Serial.print(gMode == OUT_SUMMARY ? "summary" : gMode == OUT_RAW ? "raw" : "quiet");
  Serial.print(" sample_hz=");  Serial.print(SAMPLE_HZ);
  Serial.print(" report_hz=");  Serial.print(REPORT_HZ);
  Serial.print(" i2c_hz=");     Serial.print(I2C_HZ);
  Serial.print(" revs=");       Serial.print((float)gCounts / COUNTS_PER_REV, 3);
  Serial.print(" i2c_errors="); Serial.print(gI2cErrors);
  Serial.print(" overruns=");   Serial.print(gOverruns);
  Serial.print(" bus_resets="); Serial.println(gBusResets);
  printHeader();
}

static void handleCommands() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 's': gMode = OUT_SUMMARY; printStatus(); break;
      case 'r': gMode = OUT_RAW;     printStatus(); break;
      case 'q': gMode = OUT_QUIET;   printStatus(); break;
      case 'z': zeroCounter();                       break;
      case '?': printStatus();                       break;
      default: break;
    }
  }
}

static void processSample(const Sample &s) {
  bool valid = sampleValid(s);
  gLast = s;
  wSamples++;
  if (valid) wValid++;
  if (sampleWeak(s)) wWeak++;

  if (valid) {
    if (gSynced) {
      // Shortest signed step between successive angles, in [-8192, 8191].
      int32_t d = (((int32_t)s.raw - gPrevRaw + COUNTS_PER_REV / 2) & (COUNTS_PER_REV - 1))
                  - COUNTS_PER_REV / 2;
      gCounts += d;
      wCounts += d;
      wSpanUs += s.t_us - gPrevUs;
    }
    gPrevRaw = s.raw;
    gPrevUs  = s.t_us;
  }
  gSynced = valid;

  if (gMode == OUT_RAW) {
    Serial.print(s.t_us - gZeroUs); Serial.print(',');
    Serial.print(s.raw);            Serial.print(',');
    Serial.print(s.mag);            Serial.print(',');
    Serial.print(s.agc);            Serial.print(',');
    Serial.print(s.diag);           Serial.print(',');
    Serial.println(valid ? 1 : 0);
  }
}

// Emit one summary line per report period. Driven by the clock, not by good
// reads, so a log keeps ticking (with i2c_err > 0) if the sensor drops out.
static void reportIfDue(uint32_t now) {
  if ((int32_t)(now - wNextUs) < 0) return;
  wNextUs += REPORT_US;
  if ((int32_t)(now - wNextUs) >= 0) wNextUs = now + REPORT_US;   // after a long stall

  if (gMode == OUT_SUMMARY) {
    // RPM only from windows where most of the time was covered by valid
    // samples; otherwise print nan rather than a misleading number.
    uint16_t n = wSamples ? wSamples : 1;
    Serial.print((now - gZeroUs) / 1000); Serial.print(',');
    if (wSpanUs > REPORT_US / 2) {
      Serial.print(60.0e6f * wCounts / COUNTS_PER_REV / wSpanUs, 2);
    } else {
      Serial.print("nan");
    }
    Serial.print(',');
    Serial.print((float)gCounts / COUNTS_PER_REV, 3); Serial.print(',');
    Serial.print(gLast.raw * DEG_PER_LSB, 2);          Serial.print(',');
    Serial.print(gLast.agc);                           Serial.print(',');
    Serial.print(gLast.mag);                           Serial.print(',');
    Serial.print(100 * wValid / n);                    Serial.print(',');
    Serial.print(100 * wWeak / n);                     Serial.print(',');
    Serial.println(wErrors);
  }
  wCounts = 0; wSpanUs = 0; wSamples = 0; wValid = 0; wWeak = 0; wErrors = 0;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("# AS5048B angle + RPM logger");
  Serial.println("# I2C: SDA = PB9 / D14,  SCL = PB8 / D15");

  initBus();
#if USE_INTERNAL_PULLUPS
  Serial.println("# (weak internal pull-ups enabled; fit 4.7k to 3V3 for reliability)");
#endif

  Serial.println("# I2C scan:");
  if (scanBus() == 0) {
    Serial.println("#   none found -> check 3V3, GND, SDA(D14), SCL(D15),");
    Serial.println("#                 and the 4.7k pull-ups on SDA/SCL to 3V3");
  }

  // Time one burst read so a too-high SAMPLE_HZ is obvious at power-up.
  Sample s;
  uint32_t t0 = micros();
  uint8_t ok = 0;
  for (uint8_t i = 0; i < 50; i++) ok += readSample(s);
  uint32_t perRead = (micros() - t0) / 50;
  Serial.print("# read time "); Serial.print(perRead);
  Serial.print(" us -> max ~"); Serial.print(perRead ? 1000000UL / perRead : 0);
  Serial.print(" Hz ("); Serial.print(ok); Serial.println("/50 ok)");
  if (perRead >= SAMPLE_US) Serial.println("# WARNING: SAMPLE_HZ too high for this I2C speed");

  Serial.println("# commands: s=summary r=raw q=quiet z=zero ?=status");
  zeroCounter();
  wNextUs = micros() + REPORT_US;
  printStatus();
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t failSinceUs = 0;   // start of the current run of failures
  static bool     failing = false;

  handleCommands();

  uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += SAMPLE_US;
  if ((int32_t)(now - nextUs) >= 0) {   // fell a whole period behind
    gOverruns++;
    nextUs = now + SAMPLE_US;
  }

  Sample s;
  bool ok = readSample(s);
  if (ok) {
    failing = false;
    processSample(s);
  } else {
    gI2cErrors++;
    wSamples++;
    wErrors++;
    gSynced = false;   // can't unwrap across a gap of unknown length
    if (!failing) { failing = true; failSinceUs = s.t_us; }
  }
  reportIfDue(micros());
  if (ok || micros() - failSinceUs < 100000UL) return;   // reset after 100 ms of failures

  failing = false;
  gBusResets++;
  Serial.print("# I2C reads failing -- resetting bus (reset #");
  Serial.print(gBusResets);
  Serial.println("); if this repeats, check wiring / fit 4.7k pull-ups");
  Wire.end();
  initBus();
  nextUs = micros();
}
