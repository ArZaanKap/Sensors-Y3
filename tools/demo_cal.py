#!/usr/bin/env python3
"""
On-the-day calibration of the `airspeed` firmware against the tunnel's VDAS.

1. Flash `airspeed`, mount the rig, fan OFF, press the blue button (LED steady).
2. In WindTunnelLogger: ground truth = Wind tunnel, wait for the tunnel offset,
   connect the board, Start Logging, then run a mini protocol:
     fan off 10 s | full 30 s | 1 panel off 20 s | 2 panels off 20 s | both back 10 s | fan off 25 s
   Stop & Save.
3. In WindTunnelLogger, Disconnect the board (frees the COM port), then:
     python tools/demo_cal.py <logs>/<run>_aligned.csv              # report only
     python tools/demo_cal.py <logs>/<run>_aligned.csv --port COM13  # report + send to board
   Add --exponent to also fit N (only if the report says it helps).
4. Reconnect the board in WindTunnelLogger.

What it fits, using the run's _student.csv (200 Hz) and _vdas.csv (~4 Hz):
  lag    shift of our stream that best matches VDAS (VDAS reacts later -> delay our output)
  scale  k with VDAS ~ k * ours  ->  S_new = k * S_old             (always)
  N      VDAS ~ c * ours^p       ->  N_new = p*N_old, S_new = c*S_old^p   (--exponent)
It reads the board's current S/N/delay first (or takes --S/--N/--delay-ms if
no port is given) so the new values are absolute.
Needs numpy and pyserial.
"""
import argparse
import csv
import os
import sys
import time

import numpy as np


# ---------------------------------------------------------------- files
def read_cols(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    head = [h.strip() for h in rows[0]]
    cols = {h: [] for h in head}
    for r in rows[1:]:
        for h, x in zip(head, r):
            cols[h].append(x)
    return cols


def num(col):
    out = []
    for x in col:
        try:
            out.append(float(x))
        except ValueError:
            out.append(np.nan)
    return np.array(out)


def pick(cols, *names):
    for n in names:
        if n in cols:
            return num(cols[n])
    for n in names:                       # loose match
        for h in cols:
            if n in h:
                return num(cols[h])
    raise KeyError(f"none of {names} in {list(cols)}")


def load_run(path):
    base = path
    for suf in ("_aligned.csv", "_student.csv", "_vdas.csv"):
        if base.endswith(suf):
            base = base[: -len(suf)]
    vd = read_cols(base + "_vdas.csv")
    st = read_cols(base + "_student.csv")
    tg, g = pick(vd, "timestamp_epoch"), pick(vd, "value_raw_mps", "mps")
    tu, u = pick(st, "timestamp_epoch"), pick(st, "value_raw_mps", "mps")
    mg, mu = np.isfinite(tg) & np.isfinite(g), np.isfinite(tu) & np.isfinite(u)
    print(f"run {os.path.basename(base)}: VDAS {mg.sum()} samples "
          f"({mg.sum() / (tg[mg][-1] - tg[mg][0]):.1f} Hz), board {mu.sum()} samples "
          f"({mu.sum() / (tu[mu][-1] - tu[mu][0]):.1f} Hz)")
    return tg[mg], g[mg], tu[mu], u[mu]


# ---------------------------------------------------------------- fitting
def ours_at(tg, tu, u, lag):
    """Our stream delayed by `lag` s, sampled at the VDAS instants."""
    return np.interp(tg - lag, tu, u, left=np.nan, right=np.nan)


def fit_scale(g, x):
    m = np.isfinite(x) & ((g > 1.0) | (x > 1.0))
    return (g[m] @ x[m]) / (x[m] @ x[m])


def mse(g, x):
    m = np.isfinite(x)
    return np.mean((g[m] - x[m]) ** 2)


def report(tg, g, tu, u):
    lags = np.arange(-0.5, 1.51, 0.025)
    res = []
    for L in lags:
        x = ours_at(tg, tu, u, L)
        k = fit_scale(g, x)
        res.append(mse(g, k * x))
    res = np.array(res)
    lag = lags[np.argmin(res)]
    x0 = ours_at(tg, tu, u, 0.0)
    x = ours_at(tg, tu, u, lag)
    k = fit_scale(g, x)
    print(f"\nas recorded           MSE {mse(g, x0):.4f}  (bias {np.nanmean(x0 - g):+.3f} m/s)")
    print(f"scale only (lag 0)    MSE {mse(g, fit_scale(g, x0) * x0):.4f}  k={fit_scale(g, x0):.4f}")
    print(f"scale + lag {lag*1000:4.0f} ms   MSE {res.min():.4f}  k={k:.4f}")

    m = np.isfinite(x) & (g > 1.5) & (x > 1.5)
    p, lc = np.polyfit(np.log(x[m]), np.log(g[m]), 1)
    c = np.exp(lc)
    xe = np.where(x > 0, c * np.abs(x) ** p, 0)
    print(f"scale + exponent      MSE {mse(g, xe):.4f}  (VDAS ~ {c:.4f} * ours^{p:.3f})")

    # per speed band, to see where the error is
    print("\nGT band     n   mean GT   mean ours*k   err")
    for lo, hi in ((-1, 0.5), (0.5, 3), (3, 6), (6, 9), (9, 15)):
        mm = (g >= lo) & (g < hi) & np.isfinite(x)
        if mm.sum():
            print(f"{lo:4.1f}-{hi:<4.1f} {mm.sum():5d} {g[mm].mean():8.2f} {(k * x[mm]).mean():10.2f} "
                  f"{(k * x[mm] - g[mm]).mean():+7.2f}")
    return lag, k, p, c


# ---------------------------------------------------------------- board
def board_cmd(ser, line, wait=1.5):
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    time.sleep(wait)                      # parameter saves erase a flash sector (~1 s)
    out = ser.read(ser.in_waiting or 1).decode("ascii", "replace")
    return [l for l in out.splitlines() if l.startswith("#")]


def board_params(ser):
    lines = board_cmd(ser, "c", 0.5)
    vals = {}
    for l in lines:
        if "=" in l:
            k, v = l[1:].split("=", 1)
            try:
                vals[k.strip()] = float(v)
            except ValueError:
                pass
    return vals


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="the run's _aligned.csv (or _student/_vdas.csv)")
    ap.add_argument("--port", help="board COM port: read its params and send the new ones")
    ap.add_argument("--exponent", action="store_true", help="also fit N")
    ap.add_argument("--no-lag", action="store_true", help="don't change the output delay")
    ap.add_argument("--S", type=float, help="S used during the run (if no --port)")
    ap.add_argument("--N", type=float, default=0.5)
    ap.add_argument("--delay-ms", type=float, default=0)
    a = ap.parse_args()

    tg, g, tu, u = load_run(a.run)
    lag, k, p, c = report(tg, g, tu, u)

    ser = None
    if a.port:
        import serial
        ser = serial.Serial(a.port, 115200, timeout=0.5)
        time.sleep(0.3)
        cur = board_params(ser)
        if "S" not in cur:
            sys.exit("board didn't answer 'c' - is the airspeed firmware running, and the logger disconnected?")
        S0, N0, D0 = cur["S"], cur["N"], cur.get("delay_ms", 0)
    else:
        if a.S is None:
            print("\n(no --port and no --S: showing relative changes only)")
        S0, N0, D0 = a.S or 1.0, a.N, a.delay_ms

    if a.exponent:
        S1, N1 = c * S0 ** p, p * N0
    else:
        S1, N1 = k * S0, N0
    D1 = D0 if a.no_lag else max(0.0, D0 + lag * 1000)
    print(f"\nnew parameters: S = {S1:.6f} (was {S0:.6f})  N = {N1:.4f} (was {N0:.4f})  "
          f"delay = {D1:.0f} ms (was {D0:.0f})")

    if ser:
        for cmd in (f"s {S1:.6f}", f"n {N1:.4f}", f"l {D1:.0f}"):
            print(" ", cmd, "->", " | ".join(board_cmd(ser, cmd)))
        print("\n".join(board_cmd(ser, "c", 0.5)))
        ser.close()
        print("done - reconnect the board in WindTunnelLogger")


if __name__ == "__main__":
    main()
