// Strain gauge test: Qwiic Scale (NAU7802) on Nucleo-F411RE
//
// Structure:
//   - a hardware timer ticks at 200 Hz and raises a flag (Step 1)
//   - loop() runs short non-blocking functions over and over (Step 2)
//   - on each tick, the latest load cell value is sent (Step 3)
//   - timing is measured so we can prove the 5 ms deadline is met (Step 4)
//
// Wiring (Qwiic cable -> Nucleo Arduino header):
//   Black -> GND, Red -> 3V3, Blue (SDA) -> D14/PB9, Yellow (SCL) -> D15/PB8
//
// Serial output @115200 (ST-Link virtual COM port):
//   "# ..." lines are status/comments
//   data lines (200 Hz) are CSV: t_ms,net,grams
//     net   = latest reading - tare offset, in ADC counts
//     grams = net / calibration factor (0 until calibrated)
//
// Serial commands (end with Enter):
//   t        = tare (zero with nothing on the cell)
//   c<mass>  = calibrate with a known mass in grams on the cell, e.g. "c200"
//   s        = rerun I2C scan
//   Note: t and c block for ~0.2 s while averaging, so a few output ticks are
//   missed when you use them. Fine for setup, don't use them mid-measurement.

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

const uint32_t OUTPUT_HZ = 200;

NAU7802 scale;

// Latest values: written by readLoadCell(), read by sendOutput()
int32_t latestNet = 0;
float calFactor = 0.0f; // counts per gram, 0 = not calibrated

char cmdBuf[16];
uint8_t cmdLen = 0;

// ---------- Step 1: 200 Hz hardware timer ----------
// The timer interrupt only sets a flag; the real work happens in loop().
// volatile = this variable changes inside an interrupt, so always re-read it.

HardwareTimer outputTimer(TIM2);
volatile bool outputDue = false;
volatile uint32_t tickMicros = 0;   // when the latest tick happened
volatile uint32_t missedTicks = 0;  // ticks that arrived before the last one was sent

void onOutputTimer()
{
  if (outputDue) missedTicks++;
  outputDue = true;
  tickMicros = micros();
}

// ---------- Step 4: timing statistics (reset every second) ----------

uint32_t samplesIn = 0;    // load cell samples read
uint32_t samplesOut = 0;   // lines sent
uint32_t worstLateUs = 0;  // longest delay from tick to send
uint32_t worstPassUs = 0;  // longest single pass through loop()
uint32_t lastReport = 0;

// ---------- Setup helpers ----------

void scanI2C()
{
  Serial.println("# I2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("#   found 0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x2A) Serial.print("  <- NAU7802 (Qwiic Scale)");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("#   nothing found - check wiring/power");
}

void tare()
{
  Serial.println("# Taring - keep the load cell unloaded...");
  scale.calculateZeroOffset(64);
  Serial.print("# Zero offset = ");
  Serial.println(scale.getZeroOffset());
}

void calibrate(float grams)
{
  if (grams <= 0)
  {
    Serial.println("# ERROR: give a positive mass, e.g. c200");
    return;
  }
  Serial.print("# Calibrating with ");
  Serial.print(grams, 1);
  Serial.println(" g - keep the mass still...");
  scale.calculateCalibrationFactor(grams, 64);
  calFactor = scale.getCalibrationFactor();
  Serial.print("# Calibration factor = ");
  Serial.print(calFactor, 3);
  Serial.println(" counts/g");
}

void handleCommand(const char *cmd)
{
  if (cmd[0] == 't') tare();
  else if (cmd[0] == 's') scanI2C();
  else if (cmd[0] == 'c') calibrate(atof(cmd + 1));
  else if (cmd[0] != '\0')
  {
    Serial.print("# unknown command: ");
    Serial.println(cmd);
  }
}

// ---------- Step 2: non-blocking tasks ----------
// Each function returns immediately if it has nothing to do. None of them wait.

void handleSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      cmdBuf[cmdLen] = '\0';
      handleCommand(cmdBuf);
      cmdLen = 0;
    }
    else if (cmdLen < sizeof(cmdBuf) - 1)
    {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void readLoadCell()
{
  if (!scale.available()) return; // no new sample yet (arrives at 320 Hz)
  latestNet = scale.getReading() - scale.getZeroOffset();
  samplesIn++;
}

// ---------- Step 3: send the latest value on each timer tick ----------

void sendOutput()
{
  if (!outputDue) return;
  outputDue = false;

  uint32_t late = micros() - tickMicros;
  if (late > worstLateUs) worstLateUs = late;

  Serial.print(millis());
  Serial.print(',');
  Serial.print(latestNet);
  Serial.print(',');
  Serial.println(calFactor != 0.0f ? latestNet / calFactor : 0.0f, 2);
  samplesOut++;
}

void reportStats()
{
  uint32_t now = millis();
  if (now - lastReport < 1000) return;

  Serial.print("# in=");
  Serial.print(samplesIn);
  Serial.print("Hz out=");
  Serial.print(samplesOut);
  Serial.print("Hz missed=");
  Serial.print(missedTicks);
  Serial.print(" late_max=");
  Serial.print(worstLateUs);
  Serial.print("us pass_max=");
  Serial.print(worstPassUs);
  Serial.println("us");

  samplesIn = samplesOut = 0;
  missedTicks = 0;
  worstLateUs = worstPassUs = 0;
  lastReport = now;
}

// ---------- Setup and main loop ----------

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("# NAU7802 strain gauge test (200 Hz timer output)");

  Wire.begin();          // SDA = PB9 (D14), SCL = PB8 (D15)
  Wire.setClock(400000);

  scanI2C();

  if (!scale.begin())
  {
    Serial.println("# ERROR: NAU7802 not detected at 0x2A. Halting.");
    while (1)
    {
      delay(1000);
    }
  }

  scale.setGain(NAU7802_GAIN_128);
  scale.setSampleRate(NAU7802_SPS_320);
  scale.calibrateAFE(); // must re-run after changing gain/rate

  // First few conversions after a config change are unsettled
  for (uint8_t i = 0; i < 10; i++)
  {
    while (!scale.available()) {}
    scale.getReading();
  }

  tare();
  Serial.println("# t_ms,net,grams");

  // Start the 200 Hz timer last, once everything else is ready
  outputTimer.setOverflow(OUTPUT_HZ, HERTZ_FORMAT);
  outputTimer.attachInterrupt(onOutputTimer);
  outputTimer.resume();
  lastReport = millis();
}

void loop()
{
  uint32_t passStart = micros();

  handleSerial();
  readLoadCell();
  sendOutput();
  reportStats();

  uint32_t pass = micros() - passStart;
  if (pass > worstPassUs) worstPassUs = pass;
}
