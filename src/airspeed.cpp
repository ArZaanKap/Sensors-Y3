// airspeed — COMP0219 CW1 demo firmware: drag-plate airspeed at 200 Hz.
//
// Streams one airspeed in m/s per line at 200 Hz, 115200 baud: the format
// WindTunnelLogger expects. All processing runs on the Nucleo:
//
//   NAU7802 load (320 SPS) -> minus zero L0 -> 2-state Kalman (force, force rate)
//   -> v = S * max(f - DEAD, 0)^N -> max(v, FLOOR) -> optional output delay -> serial
//
// Drag on the plate goes as v^2, so N = 0.5 by physics. S is the one number the
// on-the-day calibration sets (tools/demo_day.py). FLOOR matches the tunnel's
// reading at rest: VDAS reports whole Pa minus a non-integer zero offset and the
// logger takes sqrt(2|dp|/rho), so with the fan off it reads ~0.7-1.3 m/s, not 0.
//
// Zeroing (fan OFF, nothing touching the rig): automatically at power-up, and
// again on the 'z' command (tools/demo_day.py sends it) or the blue USER button. 2 s average; output is 0
// while zeroing. Green LED (LD2): blinking = zeroing, on = zeroed and running.
//
// Serial commands (one per line; replies start with '#'):
//   c          print the calibration ("calibration file on demand")
//   z          re-zero now (fan OFF)
//   s <x>      scale S (m/s per count^N)          n <x>  exponent N
//   d <x>      deadband DEAD (counts)              l <x>  output delay (ms, 0-1000)
//   q <x>      Kalman process noise (counts^2/s^3) r <x>  measurement noise (counts^2)
//   f <x>      output floor (m/s): output = max(v, FLOOR)
//   m 0|1      0 = speed only; 1 = raw "t_us,load,ang,f,v" (speed still last)
//   w          save parameters to flash (s/n/d/l/q/r also save themselves)
//   x          restore the built-in defaults (and save)
//   D          dump the raw recorder (see below) as text, then resume
//
// Raw recorder: the last REC_N samples of raw load, rotor angle and mic envelope at 100 Hz
// (~170 s) are kept in RAM all the time, so a tunnel run's raw data can be
// downloaded afterwards ('D', tools/demo_day.py raw) while WindTunnelLogger
// only ever sees the speed stream.
// Parameters live in flash (EEPROM emulation), so a reset keeps the calibration.
//
// Wiring: NAU7802 and AS5048B on I2C1, SDA D14 (PB9), SCL D15 (PB8). The rotor
// (AS5048B, 0x40) is optional: it is only reported in raw mode.

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

static const uint32_t I2C_SDA = PB9;   // D14
static const uint32_t I2C_SCL = PB8;   // D15
static const uint8_t AS5048B_ADDR = 0x40;
static const uint8_t MIC_PIN = A1;           // PA1 <- Sound Detector ENVELOPE (raw recorder only)
static const uint32_t SAMPLE_US = 5000;      // 200 Hz
static const float DT = SAMPLE_US * 1e-6f;
static const uint32_t STALL_MS = 100;        // no load conversion this long = bus stuck
static const uint32_t ZERO_MS = 2000;
static const int MAX_DELAY = 200;            // samples (1 s)
static const int REC_N = 17000;              // raw recorder: 170 s at 100 Hz (~100 KB RAM)
static const int REC_EVERY = 2;              // record every 2nd 200 Hz tick

// ---------------------------------------------------------------- parameters
struct Params {
  uint32_t magic;
  float S, N, dead, q, r, floor;
  uint16_t delayMs;
  uint8_t raw;
};
static const uint32_t MAGIC = 0xA1E50004;
// S from the FAN4 + anemometer fit (v = 0.0509 sqrt(dL)): a placeholder until
// the good tunnel's VDAS sets it.
static const Params DEFAULTS = {MAGIC, 0.0509f, 0.5f, 300.0f, 1e7f, 2.25e6f, 0.0f, 0, 0};
static Params P = DEFAULTS;

static void saveParams() {
  eeprom_buffer_fill();
  const uint8_t *b = (const uint8_t *)&P;
  for (size_t i = 0; i < sizeof(P); i++) eeprom_buffered_write_byte(i, b[i]);
  eeprom_buffer_flush();                      // one sector erase + write (~1 s stall)
}

static void loadParams() {
  eeprom_buffer_fill();
  Params t;
  uint8_t *b = (uint8_t *)&t;
  for (size_t i = 0; i < sizeof(t); i++) b[i] = eeprom_buffered_read_byte(i);
  if (t.magic == MAGIC && t.S > 0 && t.N > 0 && t.N < 2 && t.delayMs <= 1000) P = t;
}

// ---------------------------------------------------------------- sensors
NAU7802 scale;
static bool scaleOK = false;
static uint32_t busResets = 0;

static inline void enableInternalPullups() {
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (9 * 2))) | (1u << (9 * 2));
  GPIOB->PUPDR = (GPIOB->PUPDR & ~(3u << (8 * 2))) | (1u << (8 * 2));
}

static bool initScale() {
  if (!scale.begin(Wire)) return false;
  scale.setSampleRate(NAU7802_SPS_320);
  scale.calibrateAFE();
  return true;
}

static void resetBus() {
  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL);
  enableInternalPullups();
  scaleOK = initScale();
  busResets++;
}

static int readAngle() {                      // 0..16383, or -1
  Wire.beginTransmission(AS5048B_ADDR);
  Wire.write(0xFE);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5048B_ADDR, (uint8_t)2) != 2) return -1;
  uint8_t msb = Wire.read(), lsb = Wire.read();
  return ((int)msb << 6) | (lsb & 0x3F);
}

// ---------------------------------------------------------------- estimator
static float L0 = 0;                          // zero (counts)
static bool zeroed = false;
static bool zeroing = false;
static uint32_t zeroStart = 0;
static double zSum = 0, zSq = 0;
static uint32_t zN = 0;

// 2-state constant-velocity Kalman on force f (counts) and its rate.
static float kx0 = 0, kx1 = 0, p00 = 1e12f, p01 = 0, p11 = 1e12f;

static void kalmanReset(float f) {
  kx0 = f; kx1 = 0; p00 = P.r; p01 = 0; p11 = 1e10f;
}

static float kalmanStep(float z) {
  // predict: x = F x, P = F P F' + Q (white-noise acceleration)
  kx0 += DT * kx1;
  float n00 = p00 + DT * (2 * p01 + DT * p11);
  float n01 = p01 + DT * p11;
  float n11 = p11;
  const float dt2 = DT * DT, dt3 = dt2 * DT;
  n00 += P.q * dt3 / 3; n01 += P.q * dt2 / 2; n11 += P.q * DT;
  // update with z
  float s = n00 + P.r;
  float k0 = n00 / s, k1 = n01 / s;
  float e = z - kx0;
  kx0 += k0 * e; kx1 += k1 * e;
  p00 = (1 - k0) * n00; p01 = (1 - k0) * n01; p11 = n11 - k1 * n01;
  return kx0;
}

static float speedFrom(float f) {
  float x = f - P.dead;
  float v = x > 0 ? P.S * powf(x, P.N) : 0.0f;
  return v > P.floor ? v : P.floor;
}

static float delayBuf[MAX_DELAY + 1];
static int delayHead = 0;

static float delayed(float v) {
  int n = P.delayMs / 5;
  if (n <= 0) return v;
  if (n > MAX_DELAY) n = MAX_DELAY;
  delayBuf[delayHead] = v;
  int i = delayHead - n;
  if (i < 0) i += MAX_DELAY + 1;
  delayHead = (delayHead + 1) % (MAX_DELAY + 1);
  return delayBuf[i];
}

static void startZero() {
  zeroing = true;
  zeroStart = millis();
  zSum = zSq = 0;
  zN = 0;
}

// ---------------------------------------------------------------- output
static void fmtFloat(char *dst, size_t n, float v, int dec) {
  // newlib-nano printf has no %f: format by hand.
  long m = 1;
  for (int i = 0; i < dec; i++) m *= 10;
  long x = lroundf(v * m);
  const char *sign = x < 0 ? "-" : "";
  if (x < 0) x = -x;
  if (dec > 0) snprintf(dst, n, "%s%ld.%0*ld", sign, x / m, dec, x % m);
  else snprintf(dst, n, "%s%ld", sign, x);
}

static void printParam(const char *name, float v, int dec) {
  char b[24];
  fmtFloat(b, sizeof(b), v, dec);
  Serial.print("# "); Serial.print(name); Serial.print(" = "); Serial.println(b);
}

static void printCalibration() {
  Serial.println("# --- airspeed calibration ---");
  Serial.println("# model: v = max(S * max(L - L0 - DEAD, 0)^N, FLOOR), L Kalman-filtered (2-state CV)");
  printParam("S", P.S, 6);
  printParam("N", P.N, 4);
  printParam("DEAD_counts", P.dead, 1);
  printParam("L0_counts", L0, 1);
  printParam("q_counts2_per_s3", P.q, 0);
  printParam("r_counts2", P.r, 0);
  printParam("floor_mps", P.floor, 3);
  printParam("delay_ms", P.delayMs, 0);
  printParam("raw_mode", P.raw, 0);
  printParam("zeroed", zeroed, 0);
  printParam("i2c_bus_resets", busResets, 0);
  Serial.println("# --- end ---");
}

// ---------------------------------------------------------------- raw recorder
// Load is 24-bit (NAU7802), packed into 3 bytes so the mic byte fits in the same RAM.
static const int32_t LOAD_NONE = -8388608;     // 0x800000, the int24 minimum
static uint8_t recLoad[REC_N][3];
static int16_t recAng[REC_N];
static uint8_t recMic[REC_N];                  // mic envelope, 10-bit ADC >> 2 (0..255)
static int recHead = 0, recCount = 0;
static uint32_t recLastUs = 0;

static inline int32_t recLoadAt(int j) {
  int32_t v = (int32_t)recLoad[j][0] | ((int32_t)recLoad[j][1] << 8) | ((int32_t)recLoad[j][2] << 16);
  return (v & 0x800000) ? v - 0x1000000 : v;   // sign-extend 24 -> 32 bits
}

static void recordSample(uint32_t t, int32_t load, int16_t ang, uint8_t mic) {
  if (load < -8388608) load = -8388608;
  if (load > 8388607) load = 8388607;
  recLoad[recHead][0] = load & 0xFF;
  recLoad[recHead][1] = (load >> 8) & 0xFF;
  recLoad[recHead][2] = (load >> 16) & 0xFF;
  recAng[recHead] = ang;
  recMic[recHead] = mic;
  recHead = (recHead + 1) % REC_N;
  if (recCount < REC_N) recCount++;
  recLastUs = t;
}

// Text dump, oldest first: header, one "load,ang,mic" line per sample, footer.
// ~15 s at 115200 baud; the speed stream pauses meanwhile.
static void dumpRecorder() {
  int n = recCount;
  uint32_t lastUs = recLastUs;
  int start = (recHead - n + REC_N) % REC_N;
  Serial.print("# dump n="); Serial.print(n);
  Serial.print(" dt_us="); Serial.print(SAMPLE_US * REC_EVERY);
  Serial.print(" last_us="); Serial.print(lastUs);
  Serial.print(" now_us="); Serial.print(micros());
  Serial.print(" L0="); Serial.println((long)L0);
  char b[32];
  for (int i = 0; i < n; i++) {
    int j = (start + i) % REC_N;
    int32_t ld = recLoadAt(j);
    int len = snprintf(b, sizeof(b), "%ld,%d,%u\n", ld == LOAD_NONE ? -99999L : (long)ld, recAng[j], (unsigned)recMic[j]);
    Serial.write((const uint8_t *)b, len);
  }
  Serial.println("# end");
  Serial.flush();
}

// ---------------------------------------------------------------- commands
static char cmd[40];
static uint8_t cmdLen = 0;

static void handleCommand(char *c) {
  while (*c == ' ') c++;
  char k = c[0];
  float val = atof(c + 1);
  bool hasVal = strlen(c) > 1;
  bool save = false;
  switch (k) {
    case 'c': printCalibration(); return;
    case 'D': dumpRecorder(); return;
    case 'z': startZero(); Serial.println("# zeroing (fan OFF)"); return;
    case 'w': saveParams(); Serial.println("# saved"); return;
    case 'x': P = DEFAULTS; saveParams(); Serial.println("# defaults restored"); printCalibration(); return;
    case 's': if (hasVal && val > 0) { P.S = val; save = true; } break;
    case 'n': if (hasVal && val > 0.1f && val < 2) { P.N = val; save = true; } break;
    case 'd': if (hasVal && val >= 0) { P.dead = val; save = true; } break;
    case 'l': if (hasVal && val >= 0 && val <= 1000) { P.delayMs = (uint16_t)val; save = true; } break;
    case 'q': if (hasVal && val > 0) { P.q = val; save = true; } break;
    case 'r': if (hasVal && val > 0) { P.r = val; save = true; } break;
    case 'f': if (hasVal && val >= 0 && val < 3) { P.floor = val; save = true; } break;
    case 'm': if (hasVal) { P.raw = val != 0; save = true; } break;
    default: Serial.println("# unknown command (c z s n d l q r f m w x D)"); return;
  }
  if (save) {
    saveParams();
    Serial.print("# set "); Serial.println(c);
  } else {
    Serial.print("# rejected "); Serial.println(c);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen) { cmd[cmdLen] = 0; handleCommand(cmd); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = ch;
    }
  }
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT);                   // B1, active low
  Wire.begin(I2C_SDA, I2C_SCL);
  enableInternalPullups();
  loadParams();
  scaleOK = initScale();
  delay(300);                                 // let the AFE settle after calibrateAFE
  Serial.println(scaleOK ? "# airspeed: NAU7802 OK - zeroing, keep the fan OFF"
                         : "# airspeed: NAU7802 NOT FOUND - check wiring");
  startZero();
}

void loop() {
  pollSerial();

  static uint32_t next = 0;
  if (next == 0) next = micros() + SAMPLE_US;
  if ((int32_t)(micros() - next) < 0) return;
  next += SAMPLE_US;
  if ((int32_t)(micros() - next) > (int32_t)SAMPLE_US) next = micros() + SAMPLE_US;
  uint32_t t = micros();

  // Button: re-zero on a press (debounced by the 5 ms tick).
  static bool btnPrev = true;
  bool btn = digitalRead(USER_BTN);
  if (!btn && btnPrev) startZero();
  btnPrev = btn;

  // Load cell: newest conversion (320 SPS, so nearly always fresh).
  static float load = NAN;
  static uint32_t lastFresh = millis();
  if (scaleOK && scale.available()) {
    load = (float)scale.getReading();
    lastFresh = millis();
  } else if (millis() - lastFresh > STALL_MS) {
    resetBus();
    lastFresh = millis();
  }

  static uint8_t recTick = 0;
  bool recNow = ++recTick >= REC_EVERY;
  if (recNow) recTick = 0;
  int ang = (P.raw || recNow) ? readAngle() : -1;
  if (recNow) {
    uint8_t mic = analogRead(MIC_PIN) >> 2;   // 10-bit -> 8-bit envelope
    recordSample(t, isnan(load) ? LOAD_NONE : (int32_t)load, (int16_t)ang, mic);
  }

  float f = 0, v = 0;
  if (zeroing) {
    v = P.floor;
    if (!isnan(load)) { zSum += load; zSq += (double)load * load; zN++; }
    digitalWrite(LED_BUILTIN, (millis() / 100) & 1);
    if (millis() - zeroStart >= ZERO_MS && zN > 50) {
      double mean = zSum / zN;                     // double: float L0 (~6.5e5) squared loses the variance
      float sd = sqrt(fmax(zSq / zN - mean * mean, 0.0));
      L0 = mean;
      zeroing = false;
      zeroed = true;
      kalmanReset(0);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("# zero L0 = "); Serial.print((long)L0);
      Serial.print(" sd = "); Serial.print((long)sd);
      Serial.println(sd > 1000 ? " (noisy: fan on or rig moving? press blue to redo)" : "");
    }
  } else if (zeroed && !isnan(load)) {
    f = kalmanStep(load - L0);
    v = speedFrom(f);
  } else {
    v = P.floor;
  }
  v = delayed(v);

  char vb[16];
  fmtFloat(vb, sizeof(vb), v, 3);
  char line[64];
  int len;
  if (P.raw) {
    len = snprintf(line, sizeof(line), "%lu,%ld,%d,%ld,%s\n", (unsigned long)t,
                   isnan(load) ? -99999L : (long)load, ang, (long)f, vb);
  } else {
    len = snprintf(line, sizeof(line), "%s\n", vb);
  }
  Serial.write((const uint8_t *)line, len);
}
