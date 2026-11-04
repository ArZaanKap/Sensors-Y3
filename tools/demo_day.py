#!/usr/bin/env python3
"""
demo_day.py - zero and calibrate the `airspeed` firmware against the tunnel.

    python tools/demo_day.py zero         fan OFF: re-measure the load cell's zero
    python tools/demo_day.py fit [run]    fit the newest saved run (or [run]) and send to the board
    python tools/demo_day.py cal          print the board's calibration and save it to a file

Use PlatformIO's Python (it has pyserial):  ~/.platformio/penv/Scripts/python
The board's COM port is found automatically (--port COMx to override). Only one
program can use the port: press Disconnect in WindTunnelLogger first.
--no-board: fit shows the numbers without touching a board (practice).

Calibration run (record it in WindTunnelLogger, ~2 min; timings don't matter):
    fan off 10 s | max 25 s | 3/4 15 s | 1/2 15 s | 1/4 15 s | max 15 s | fan off 25 s | Stop & Save

fit works out, from the run's _vdas.csv (tunnel) and _student.csv (your board):
    scale  S_new = k * S_old (least squares with the air moving)
    lag    VDAS reacts later than the load cell -> delay our output to line up
    floor  the app's speed at rest is sqrt(2|dp - offset|/rho), often ~1 m/s, not 0
"""
import argparse
import bisect
import csv
import glob
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing - run with  ~/.platformio/penv/Scripts/python tools/demo_day.py ...")

LOGS = os.path.join(os.path.expanduser("~"), "Documents", "Wind Tunnel Logger", "logs")


# ---------------------------------------------------------------- board
def open_board(port):
    if not port:
        ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x0483]
        if not ports:
            sys.exit("No Nucleo found - plug in its USB cable.")
        port = ports[0]
    try:
        ser = serial.Serial(port, 115200, timeout=0.3)
    except serial.SerialException:
        sys.exit(f"Can't open {port} - press Disconnect in WindTunnelLogger, then run this again.")
    time.sleep(0.2)
    return ser


def command(ser, line, wait=0.6):
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode("ascii", "replace")
    return [l.strip() for l in out.splitlines() if l.startswith("#")]


def params(ser):
    vals = {}
    for l in command(ser, "c"):
        if "=" in l:
            k, v = l[1:].split("=", 1)
            try:
                vals[k.strip()] = float(v)
            except ValueError:
                pass
    if "S" not in vals:
        sys.exit("The board didn't answer - is the airspeed firmware flashed?")
    return vals


def do_zero(a):
    ser = open_board(a.port)
    print("Zeroing (2 s) - fan OFF, nobody touching the rig...")
    ser.reset_input_buffer()
    ser.write(b"z\n")
    t0, buf = time.time(), ""
    while time.time() - t0 < 6:
        buf += ser.read(ser.in_waiting or 1).decode("ascii", "replace")
        line = next((l for l in buf.splitlines() if l.startswith("# zero L0")), None)
        if line:
            sd = float(line.split("sd =")[1].split()[0])
            print(f"Done: {line[2:]}  ->  {'OK' if sd < 600 else 'NOISY: fan on or rig moving? run zero again'}")
            print("Now Connect the board in WindTunnelLogger.")
            return
    print("No reply from the board.")


def do_cal(a):
    ser = open_board(a.port)
    txt = "\n".join(command(ser, "c"))
    path = os.path.join(os.path.dirname(LOGS), time.strftime("calibration_%Y%m%d_%H%M%S.txt"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(txt + "\n")
    print(txt)
    print(f"saved to {path}")


# ---------------------------------------------------------------- fit
def series(path):
    t, v = [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                t.append(float(r["timestamp_epoch"]))
                v.append(float(r["value_raw_mps"]))
            except (KeyError, ValueError):
                pass
    return t, v


def interp(x, xs, ys):
    if x < xs[0] or x > xs[-1]:
        return None
    i = bisect.bisect_left(xs, x)
    if xs[i] == x:
        return ys[i]
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])


def fit(run, min_lag):
    base = run[: -len("_aligned.csv")]
    tg, g = series(base + "_vdas.csv")
    tu, u = series(base + "_student.csv")
    rate = len(tu) / (tu[-1] - tu[0])
    print(f"Run {os.path.basename(base)}: {len(tg)} tunnel readings, your board {rate:.0f} Hz"
          + ("  (!! below 200 Hz)" if rate < 196 else ""))

    def ours(lag):
        return [interp(t - lag, tu, u) for t in tg]

    def mse(pred):
        e = [(a - b) ** 2 for a, b in zip(g, pred) if b is not None]
        return sum(e) / len(e)

    def scale(x):
        pairs = [(a, b) for a, b in zip(g, x) if b is not None and a > 2 and b > 2]
        den = sum(b * b for _, b in pairs)
        return sum(a * b for a, b in pairs) / den if den else 1.0

    best = None
    lag = min_lag
    while lag <= 1.5:
        x = ours(lag)
        k = scale(x)
        m = mse([k * b if b is not None else None for b in x])
        if best is None or m < best[0]:
            best = (m, lag, k)
        lag += 0.025
    _, lag, k = best
    x = ours(lag)
    rest = [a for a, b in zip(g, x) if b is not None and k * b < 1.2 and a < 2.0]
    floor = min(sum(rest) / len(rest), 2.5) if len(rest) >= 5 else None
    after = mse([max(k * b, floor or 0) if b is not None else None for b in x])
    print(f"MSE vs tunnel: {mse(ours(0)):.3f} as recorded  ->  {after:.3f} with the new parameters")
    return k, lag, floor


def do_fit(a):
    run = a.run
    if not run:
        runs = sorted(glob.glob(os.path.join(LOGS, "*_aligned.csv")), key=os.path.getmtime)
        if not runs:
            sys.exit(f"No saved runs in {LOGS} - did you press Stop & Save in the app?")
        run = runs[-1]
    ser = None if a.no_board else open_board(a.port)
    cur = params(ser) if ser else {"S": 0.0509, "delay_ms": 0, "floor_mps": 0}
    k, lag, floor = fit(run, -cur.get("delay_ms", 0) / 1000)
    S = k * cur["S"]
    D = cur.get("delay_ms", 0) + lag * 1000
    F = cur.get("floor_mps", 0) if floor is None else floor
    print(f"  S     {cur['S']:.6f} -> {S:.6f}  (x{k:.3f})")
    print(f"  delay {cur.get('delay_ms', 0):6.0f} -> {D:6.0f} ms")
    print(f"  floor {cur.get('floor_mps', 0):6.2f} -> {F:6.2f} m/s")
    if abs(k - 1) > 0.5:
        print("  !! scale changes by over 50% - right run? right tunnel?")
    if not ser:
        return
    if input("Send to the board? [Y/n] ").strip().lower() in ("", "y", "yes"):
        for cmd in (f"s {S:.6f}", f"l {D:.0f}", f"f {F:.3f}"):
            command(ser, cmd, 2.5)                  # each saves to flash (~2 s)
        p = params(ser)
        print(f"Board now: S={p['S']:.6f} delay={p['delay_ms']:.0f} ms floor={p['floor_mps']:.2f} m/s")
        print("Next: when the air is still, run  zero  again, then Connect in WindTunnelLogger.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["zero", "fit", "cal"])
    ap.add_argument("run", nargs="?", help="fit: a run's _aligned.csv (default: newest in the app's logs)")
    ap.add_argument("--port")
    ap.add_argument("--no-board", action="store_true", help="fit: just show the numbers")
    a = ap.parse_args()
    {"zero": do_zero, "fit": do_fit, "cal": do_cal}[a.action](a)


if __name__ == "__main__":
    main()
