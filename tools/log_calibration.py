#!/usr/bin/env python3
"""
Calibration logger — prove you can record BOTH at once, then use it for real.
~/.platformio/penv/bin/python tools/log_calibration.py --sim-vdas --seconds 10  
 ~/.platformio/penv/bin/python tools/log_calibration.py --seconds 30 
  lsof /dev/cu.usbmodem3103      
Reads two devices simultaneously and writes everything to timestamped CSVs:

  1. Your board's raw sensor stream (USB serial) at 200 Hz.
  2. The tunnel's VDAS ground truth (USB HID), if the VDAS box is plugged in.

For every VDAS ground-truth reading it also writes the nearest-in-time raw
sample from your board, so the two streams are already aligned for fitting.

Files written to logs/:
  <stamp>_serial.csv    every raw sample from your board, one per line
  <stamp>_vdas.csv      every ground-truth pressure reading
  <stamp>_aligned.csv   each VDAS reading + your nearest raw sample (USE THIS)

It runs fine WITHOUT a VDAS box too (bench test at home): it just logs your
board and prints a warning.

Usage:
  ~/.platformio/penv/bin/python tools/log_calibration.py [--sim-vdas] [PORT] [BAUD]

If PORT is omitted, the first /dev/cu.usbmodem* device is used.
--sim-vdas generates a FAKE ground truth (bench test; not real wind).
Stop with Ctrl-C; all files are flushed and closed.
"""

import bisect
import csv
import datetime as dt
import glob
import os
import random
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Run with PlatformIO's python:\n"
             "  ~/.platformio/penv/bin/python tools/log_calibration.py")

# Ground truth reader comes from the wind-tunnel-logger package you installed.
# It opens the VDAS box over USB HID and keeps it streaming. If it isn't
# installed or the box isn't plugged in, we simply continue without it.
try:
    from wind_tunnel_logger.vdas_afa5 import GroundTruthReader
except ImportError:
    GroundTruthReader = None

ALIGN_TOL_S = 0.150          # match a VDAS sample to a serial sample this close
LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("No /dev/cu.usbmodem* port found - is the Nucleo plugged in?")
    return ports[0]


def is_data_line(fields):
    """True if every comma-separated field parses as a number (a sample)."""
    for f in fields:
        try:
            float(f)
        except ValueError:
            return False
    return True


class SimVdas:
    """Replicates wind-tunnel-logger's 'Simulated (bench test)' ground truth.

    Uses the SAME synthetic tunnel profile the official program uses: 8 s fan
    off, then speed steps (3, 6, 9, 10, 6, 2 m/s) with 3 s ramps, turned into
    whole-Pa pressure readings at ~5 Hz. Not real wind.
    """

    def __init__(self, listener):
        self._listener = listener
        self._stop = threading.Event()
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _demo_speed(self, t):
        t -= self._t0
        if t < 8:
            return 0.0
        levels = [3.0, 6.0, 9.0, 10.0, 6.0, 2.0]
        k, frac = divmod(t - 8, 12.0)
        k = int(k)
        cur = levels[k % len(levels)]
        prev = levels[(k - 1) % len(levels)] if k > 0 else 0.0
        return prev + (cur - prev) * frac / 3.0 if frac < 3.0 else cur

    def _run(self):
        while not self._stop.is_set():
            t = time.time()
            v = self._demo_speed(t)
            pa = -0.5 * 1.204 * v * v + 2.5 + random.gauss(0, 0.6)
            self._listener({"t": t, "pa": float(round(pa)), "valid": True, "cell": 1})
            self._stop.wait(0.2)   # ~5 readings/second, like the official simulator

    def close(self):
        self._stop.set()
        self._thread.join(1.0)


class CalibrationLogger:
    def __init__(self, port, baud, sim_vdas=False):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        # Serial samples: (epoch_s, [field strings]). Header is stored separately.
        self.ser_header = None
        self.ser_samples = []
        self.ser_lock = threading.Lock()

        # VDAS ground-truth samples: (epoch_s, pressure_pa)
        self.vdas_samples = []
        self.vdas_lock = threading.Lock()
        self.vdas = None

        if sim_vdas:
            self.vdas = SimVdas(self._on_vdas)
            print("SIMULATED VDAS (same fake profile as wind-tunnel-logger's bench test) - not real wind.")
        elif GroundTruthReader is not None:
            try:
                # cell=1 is the tunnel's airspeed pressure cell (per the manual).
                self.vdas = GroundTruthReader(cell=1)
                self.vdas.add_listener(self._on_vdas)
                print("VDAS box opened (cell 1) - ground truth streaming.")
            except Exception as e:
                print(f"VDAS not available ({e}) - logging your board only.")

    def _on_vdas(self, r):
        """Called on the VDAS reader thread; just record valid pressure readings."""
        if r.get("valid"):
            with self.vdas_lock:
                self.vdas_samples.append((r["t"], r["pa"]))

    def _handle_serial_line(self, raw, epoch):
        line = raw.decode("ascii", "replace").strip()
        if not line:
            return
        fields = line.split(",")
        if is_data_line(fields):
            with self.ser_lock:
                if self.ser_header is None:
                    self.ser_header = [f"ch{i}" for i in range(len(fields))]
                if len(fields) == len(self.ser_header):
                    self.ser_samples.append((epoch, fields))
        else:
            # A header line from the firmware (e.g. t_us,mic,gate,ang_raw).
            with self.ser_lock:
                self.ser_header = fields

    def run(self, seconds=None):
        print(f"Listening on {self.ser.port} @ {self.ser.baudrate}. Ctrl-C to stop.")
        print("Recording your board and VDAS ground truth together.\n")
        n = 0
        t0 = time.time()
        while seconds is None or time.time() - t0 < seconds:
            try:
                raw = self.ser.readline()
            except Exception as e:
                # macOS USB-CDC can throw this spuriously ("device reports
                # readiness to read but returned no data"): keep trying.
                print(f"\n[serial read error, continuing: {e}]", flush=True)
                time.sleep(0.05)
                continue
            if raw:
                self._handle_serial_line(raw, time.time())
                n += 1
                if n % 200 == 0:
                    with self.ser_lock:
                        s = len(self.ser_samples)
                    with self.vdas_lock:
                        v = len(self.vdas_samples)
                    print(f"\rboard samples: {s}   ground-truth readings: {v}", end="", flush=True)

    def stop(self):
        try:
            self.ser.close()
        except Exception:
            pass
        if self.vdas is not None:
            try:
                self.vdas.close()
            except Exception:
                pass

    # ---- writing ----------------------------------------------------------
    def write(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base = os.path.join(LOG_DIR, stamp)

        with self.ser_lock:
            header = self.ser_header or ["raw"]
            samples = list(self.ser_samples)
        with self.vdas_lock:
            vdas = list(self.vdas_samples)

        # 1. Raw serial stream.
        serial_cols = ["timestamp_epoch"] + (header if header else ["raw"])
        with open(base + "_serial.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(serial_cols)
            for epoch, fields in samples:
                w.writerow([f"{epoch:.6f}"] + fields)

        # 2. Ground truth stream.
        with open(base + "_vdas.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_epoch", "pressure_pa"])
            for epoch, pa in vdas:
                w.writerow([f"{epoch:.6f}", pa])

        # 3. Aligned: each VDAS reading + nearest serial sample within tolerance.
        ser_epochs = [s[0] for s in samples]
        aligned_cols = ["vdas_timestamp_epoch", "pressure_pa", "serial_timestamp_epoch"] + header
        with open(base + "_aligned.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(aligned_cols)
            paired = 0
            for v_epoch, pa in vdas:
                i = bisect.bisect_left(ser_epochs, v_epoch)
                best = None
                for j in (i - 1, i):
                    if 0 <= j < len(ser_epochs) and abs(ser_epochs[j] - v_epoch) <= ALIGN_TOL_S:
                        if best is None or abs(ser_epochs[j] - v_epoch) < abs(ser_epochs[best] - v_epoch):
                            best = j
                if best is None:
                    continue
                s_epoch, fields = samples[best]
                w.writerow([f"{v_epoch:.6f}", pa, f"{s_epoch:.6f}"] + fields)
                paired += 1

        print(f"\nWrote {len(samples)} board samples and {len(vdas)} ground-truth readings "
              f"({paired} aligned pairs).")
        print(f"  {base}_serial.csv")
        print(f"  {base}_vdas.csv")
        print(f"  {base}_aligned.csv")


def main():
    sim_vdas = False
    seconds = None
    args = []
    raw_args = sys.argv[1:]
    i = 0
    while i < len(raw_args):
        a = raw_args[i]
        if a == "--sim-vdas":
            sim_vdas = True
        elif a == "--seconds":
            seconds = float(raw_args[i + 1])
            i += 1
        elif not a.startswith("--"):
            args.append(a)
        i += 1

    port = args[0] if len(args) > 0 else find_port()
    baud = int(args[1]) if len(args) > 1 else 115200

    logger = CalibrationLogger(port, baud, sim_vdas=sim_vdas)
    try:
        logger.run(seconds)
    except KeyboardInterrupt:
        pass
    finally:
        logger.stop()
        logger.write()


if __name__ == "__main__":
    main()
