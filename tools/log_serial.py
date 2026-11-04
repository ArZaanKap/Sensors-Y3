#!/usr/bin/env python3
"""
Capture the Nucleo logger CSV stream to a timestamped file.

The firmware (src/logger.cpp) streams one line per sample at 200 Hz:

    t_us,mic,gate,ang_raw

This script reads that stream and writes logs/<date>_<time>.csv, adding a
host wall-clock timestamp and converting the AS5048B raw angle to degrees:

    host_time,t_us,mic,gate,ang_raw,ang_deg

Ground-truth data can be merged into the same rows later.

Usage:
    ~/.platformio/penv/bin/python tools/log_serial.py [PORT] [BAUD]

If PORT is omitted, the first /dev/cu.usbmodem* device is used.
Stop with Ctrl-C; the file is flushed and closed cleanly.
"""

import csv
import datetime as dt
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Run with PlatformIO's python:\n"
             "  ~/.platformio/penv/bin/python tools/log_serial.py")

DEG_PER_LSB = 360.0 / 16384.0
LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("No /dev/cu.usbmodem* port found - is the Nucleo plugged in?")
    return ports[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = os.path.join(LOG_DIR, f"{stamp}.csv")

    ser = serial.Serial(port, baud, timeout=1)
    # Opening the port resets the board (USB CDC); wait for the header to arrive.
    print(f"Listening on {port} @ {baud}. Writing to {path}  (Ctrl-C to stop)")

    rows = 0
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["host_time", "t_us", "mic", "gate", "ang_raw", "ang_deg"])
        f.flush()
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", "replace").strip()
                if not line or line.startswith("t_us"):   # skip blank / header
                    continue
                parts = line.split(",")
                if len(parts) != 4:
                    continue
                t_us, mic, gate, ang_raw = parts
                ang = int(ang_raw)
                deg = "" if ang < 0 else f"{ang * DEG_PER_LSB:.2f}"
                host = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                writer.writerow([host, t_us, mic, gate, ang_raw, deg])
                rows += 1
                if rows % 200 == 0:      # flush about once a second
                    f.flush()
                    print(f"\r{rows} samples recorded", end="", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            f.flush()

    ser.close()
    print(f"\nDone. {rows} samples written to {path}")


if __name__ == "__main__":
    main()
