#!/usr/bin/env python3
"""
Cross-run check of the two main sensors (NAU7802 load cell, AS5048B rotor) on
every steady fan plateau in the 2026-11-04 logs, using the FAN4 calibration
v = A_PHYS * sqrt(dL) (tools/fit_fan4.py).

Plateau and baseline windows were picked by hand from 2 s summaries (seconds
from the start of each file). dL = plateau load - fan-off baseline just before
fan-on; 'post' = fan-off load after the run, if any (shows zero drift).

Run: python tools/analyse_plateaus.py  -> prints table, writes logs/plateaus.png
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "..", "logs")
A_PHYS = 0.05094          # m/s per sqrt(count), from FAN4 + anemometer
RHO = 1.20                # kg/m^3 at ~21 C

# file, baseline window, post-run fan-off window (or None), plateaus, VDAS file (or None)
RUNS = [
    ("FAN1.1.csv", (20, 25), (126, 136), [(30, 56), (60, 74), (86, 100), (106, 121)], None),
    ("FAN2.csv", (13, 17), None, [(24, 44), (48, 55), (60, 90)], None),
    ("FAN3.csv", (8, 22), None, [(30, 78)], None),
    ("2026-11-04_17-00-12_serial.csv", (10, 19), None, [(24, 35)], None),
    ("FAN4.csv", (29, 33), None, [(37, 63), (66, 86)], "anemometer"),
    ("2026-11-04_15-40-16_serial.csv", (0, 6.5), None, [(10.5, 16.5)], "2026-11-04_15-40-16_vdas.csv"),
]
FAN4_GT = [6.08, 7.22]    # anemometer plateau means (data1_anemometer_gt.csv)


def load_board(name):
    d = np.genfromtxt(os.path.join(LOGS, name), delimiter=",", skip_header=1, invalid_raise=False)
    d = d[~np.isnan(d).any(axis=1)]
    d = d[(d[:, 4] != -99999) & (d[:, 4] != 0) & (d[:, 5] >= 0)]
    return d


def window(d, t0, a, b):
    rel = d[:, 0] - t0
    return d[(rel >= a) & (rel < b)]


def rps(w):
    turns = np.unwrap(w[:, 5] * 2 * np.pi / 16384) / (2 * np.pi)
    return (turns[-1] - turns[0]) / (w[-1, 0] - w[0, 0])


def main():
    rows = []
    print(f"{'run':16s} {'plateau':>10s} {'base':>8s} {'post':>8s} {'dL':>7s} {'dL sd':>6s} "
          f"{'v_model':>7s} {'rps':>6s} {'GT v':>6s}")
    for name, bw, pw, plats, gt in RUNS:
        d = load_board(name)
        t0 = d[0, 0]
        base = window(d, t0, *bw)[:, 4].mean()
        post = window(d, t0, *pw)[:, 4].mean() if pw else np.nan
        vd = None
        if gt and gt != "anemometer":
            vd = np.genfromtxt(os.path.join(LOGS, gt), delimiter=",", skip_header=1)
        for i, (a, b) in enumerate(plats):
            w = window(d, t0, a, b)
            dL = w[:, 4].mean() - base
            # load SD of 1 s means (flow unsteadiness + noise), not of raw samples
            sec = [w[(w[:, 0] - t0 >= s) & (w[:, 0] - t0 < s + 1), 4].mean() for s in range(int(a), int(b))]
            v_model = A_PHYS * np.sqrt(max(dL, 0))
            r = abs(rps(w))
            g = np.nan
            if gt == "anemometer":
                g = FAN4_GT[i]
            elif vd is not None:
                P = vd[(vd[:, 0] - t0 >= a) & (vd[:, 0] - t0 < b), 1].mean()
                g = np.sqrt(2 * P / RHO)
            label = name.replace("2026-11-04_", "").replace("_serial.csv", "").replace(".csv", "")
            rows.append((label, dL, v_model, r, g))
            print(f"{label:16s} {a:4.0f}-{b:<5.0f} {base/1e3:7.1f}k {post/1e3:7.1f}k {dL:7.0f} "
                  f"{np.std(sec):6.0f} {v_model:7.2f} {r:6.2f} {g:6.2f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
    labels = sorted(set(r[0] for r in rows), key=[r[0] for r in rows].index)
    for lab in labels:
        rr = [r for r in rows if r[0] == lab]
        ax[0].plot([r[2] for r in rr], [r[3] for r in rr], "o", ms=8, label=lab)
    ax[0].set(xlabel="v from load cell model (m/s)", ylabel="rotor |rev/s|",
              title="Rotor vs load cell, every steady plateau")
    ax[0].legend(fontsize=8)
    gtr = [r for r in rows if not np.isnan(r[4])]
    lim = 9
    ax[1].plot([0, lim], [0, lim], "k--", lw=1, label="perfect")
    for r in gtr:
        mk = "s" if r[0] == "FAN4" else "^"
        ax[1].plot(r[4], r[2], mk, ms=9, label=f"{r[0]} ({'anemometer' if r[0]=='FAN4' else 'VDAS pitot'})")
    ax[1].set(xlim=(0, lim), ylim=(0, lim), xlabel="ground truth v (m/s)", ylabel="model v (m/s)",
              title="Load-cell model vs each ground truth")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(LOGS, "plateaus.png")
    fig.savefig(out, dpi=110)
    print("plot:", out)


if __name__ == "__main__":
    main()
