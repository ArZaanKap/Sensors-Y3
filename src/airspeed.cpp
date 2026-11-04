// airspeed — COMP0219 CW1 demo firmware: drag-plate airspeed at 200 Hz.
//
// Streams one airspeed in m/s per line at 200 Hz, 115200 baud: the format
// WindTunnelLogger expects. All processing runs on the Nucleo:
//
//   NAU7802 load (320 SPS) -> minus zero L0 -> 2-state Kalman (force, force rate)
//   -> v = S * max(f - DEAD, 0)^N -> optional output delay -> serial
//
// Drag on the plate goes as v^2, so N = 0.5 by physics. S is the one number the
// on-the-day calibration sets (tools/demo_cal.py).
//
// Zeroing (fan OFF, nothing touching the rig): automatically at power-up, and
// again on the blue USER button or the 'z' command. 2 s average; output is 0
// while zeroing. Green LED (LD2): blinking = zeroing, on = zeroed and running.
//
// Serial commands (one per line; replies start with '#'):
//   c          print the calibration ("calibration file on demand")
//   z          re-zero now (fan OFF)
//   s <x>      scale S (m/s per count^N)          n <x>  exponent N
//   d <x>      deadband DEAD (counts)              l <x>  output delay (ms, 0-1000)
//   q <x>      Kalman process noise (counts^2/s^3) r <x>  measurement noise (counts^2)
//   m 0|1      0 = speed only; 1 = raw "t_us,load,ang,f,v" (speed still last)
//   w          save parameters to flash (s/n/d/l/q/r also save themselves)
//   x          restore the built-in defaults (and save)
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
static const uint32_t SAMPLE_US = 5000;      // 200 Hz
static const float DT = SAMPLE_US * 1e-6f;
static const uint32_t STALL_MS = 100;        // no load conversion this long = bus stuck
static const uint32_t ZERO_MS = 2000;
static const int MAX_DELAY = 200;            // samples (1 s)

// ---------------------------------------------------------------- parameters
struct Params {
  uint32_t magic;
  float S, N, dead, q, r;
  uint16_t delayMs;
  uint8_t raw;
};
static const uint32_t MAGIC = 0xA1E50003;
// S from the FAN4 + anemometer fit (v = 0.0509 sqrt(dL)): a placeholder until
// the good tunnel's VDAS sets it.
static const Params DEFAULTS = {MAGIC, 0.0509f, 0.5f, 300.0f, 1e7f, 2.25e6f, 0, 0};
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
  return x > 0 ? P.S * powf(x, P.N) : 0.0f;
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
  Serial.println("# model: v = S * max(L - L0 - DEAD, 0)^N, L Kalman-filtered (2-state CV)");
  printParam("S", P.S, 6);
  printParam("N", P.N, 4);
  printParam("DEAD_counts", P.dead, 1);
  printParam("L0_counts", L0, 1);
  printParam("q_counts2_per_s3", P.q, 0);
  printParam("r_counts2", P.r, 0);
  printParam("delay_ms", P.delayMs, 0);
  printParam("raw_mode", P.raw, 0);
  printParam("zeroed", zeroed, 0);
  printParam("i2c_bus_resets", busResets, 0);
  Serial.println("# --- end ---");
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
    case 'z': startZero(); Serial.println("# zeroing (fan OFF)"); return;
    case 'w': saveParams(); Serial.println("# saved"); return;
    case 'x': P = DEFAULTS; saveParams(); Serial.println("# defaults restored"); printCalibration(); return;
    case 's': if (hasVal && val > 0) { P.S = val; save = true; } break;
    case 'n': if (hasVal && val > 0.1f && val < 2) { P.N = val; save = true; } break;
    case 'd': if (hasVal && val >= 0) { P.dead = val; save = true; } break;
    case 'l': if (hasVal && val >= 0 && val <= 1000) { P.delayMs = (uint16_t)val; save = true; } break;
    case 'q': if (hasVal && val > 0) { P.q = val; save = true; } break;
    case 'r': if (hasVal && val > 0) { P.r = val; save = true; } break;
    case 'm': if (hasVal) { P.raw = val != 0; save = true; } break;
    default: Serial.println("# unknown command (c z s n d l q r m w x)"); return;
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

  int ang = P.raw ? readAngle() : -1;

  float f = 0, v = 0;
  if (zeroing) {
    if (!isnan(load)) { zSum += load; zSq += (double)load * load; zN++; }
    digitalWrite(LED_BUILTIN, (millis() / 100) & 1);
    if (millis() - zeroStart >= ZERO_MS && zN > 50) {
      L0 = zSum / zN;
      float sd = sqrt(fmax(zSq / zN - (double)L0 * L0, 0.0));
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
