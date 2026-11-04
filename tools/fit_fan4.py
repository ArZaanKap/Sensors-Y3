#!/usr/bin/env python3
"""
Calibrate logger3 sensors (FAN4.csv) against the anemometer ground truth read
from data1.mp4 (logs/data1_anemometer_gt.csv).

Pipeline
  1. Sync: CSV_rel_time = video_time + OFFSET. The anemometer display shows the
     average over the AVG_WIN seconds before each ~1 Hz update.
  2. Features per ground-truth reading (same window):
       dL   = mean(load_raw) - fan-off baseline   (drag force, ~ v^2)
       rps  = rotor revolutions/s from the unwrapped AS5048B angle
  3. Models, scored with leave-one-segment-out cross-validation:
       physics   v = a*sqrt(dL)                  (1 parameter)
       poly1/2   v = polynomial in dL
       sqrt+b    v = a*sqrt(dL) + b
       rotor     v = v0 + c*rps  (rotor turning only)
       MLP       small neural net on [dL, rps]
  4. Kalman filter on the 200 Hz load signal, giving a live velocity estimate.

Run: python tools/fit_fan4.py   (numpy, scikit-learn, matplotlib)
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "..", "logs")
OFFSET = 26.3     # s, CSV_rel = video_t + OFFSET (fitted, see sync())
AVG_WIN = 2.0     # s, anemometer display averaging window
BASE_WIN = (29.0, 33.0)   # CSV_rel s, fan off, right before fan-on

SEGMENTS = {      # ground-truth index ranges (video seconds)
    "off": range(0, 8), "spinup": range(8, 12), "p1_6.1": range(12, 38),
    "step": range(38, 41), "p2_7.2": range(41, 60),
}


def load():
    d = np.genfromtxt(os.path.join(LOGS, "FAN4.csv"), delimiter=",", skip_header=1)
    g = np.genfromtxt(os.path.join(LOGS, "data1_anemometer_gt.csv"), delimiter=",", skip_header=4)
    rel = d[:, 0] - d[0, 0]
    load_raw = d[:, 4]
    ang = d[:, 5]
    turns = np.unwrap(ang * 2 * np.pi / 16384) / (2 * np.pi)   # cumulative revolutions
    base = load_raw[(rel >= BASE_WIN[0]) & (rel < BASE_WIN[1])].mean()
    return rel, load_raw - base, turns, g[:, 0].astype(int), g[:, 3]


def features(rel, dL, turns, k, offset=OFFSET, win=AVG_WIN):
    X = []
    for kk in k:
        t1 = kk - 0.38 + offset           # display update instant in CSV time
        m = (rel >= t1 - win) & (rel < t1)
        tt = rel[m]
        rps = (turns[m][-1] - turns[m][0]) / (tt[-1] - tt[0])
        X.append([dL[m].mean(), abs(rps)])
    return np.array(X)


def sync(rel, dL, turns, k, v):
    best = (-1, None, None)
    for off in np.arange(23.5, 27.0, 0.05):   # CSV ends ~86 s
        for win in (0.5, 1.0, 1.5, 2.0, 3.0):
            x = np.sqrt(np.clip(features(rel, dL, turns, k, off, win)[:, 0], 0, None))
            r = np.corrcoef(x, v)[0, 1]
            if r > best[0]:
                best = (r, off, win)
    return best


# ---------------------------------------------------------------- models
def fit_physics(X, y):
    s = np.sqrt(np.clip(X[:, 0], 0, None))
    a = (s @ y) / (s @ s)
    return lambda Xn: a * np.sqrt(np.clip(Xn[:, 0], 0, None))


def fit_sqrt_b(X, y):
    s = np.sqrt(np.clip(X[:, 0], 0, None))
    a, b = np.polyfit(s, y, 1)
    return lambda Xn: a * np.sqrt(np.clip(Xn[:, 0], 0, None)) + b


def fit_poly(deg):
    def f(X, y):
        c = np.polyfit(X[:, 0] / 1e4, y, deg)
        return lambda Xn: np.polyval(c, Xn[:, 0] / 1e4)
    return f


def fit_rotor(X, y):
    m = X[:, 1] > 0.3
    if m.sum() < 2:
        return None
    c, v0 = np.polyfit(X[m, 1], y[m], 1)
    return lambda Xn: np.where(Xn[:, 1] > 0.3, v0 + c * Xn[:, 1], np.nan)


def fit_mlp(X, y):
    from sklearn.neural_network import MLPRegressor
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    preds = []
    for seed in range(5):        # small ensemble: single nets on 60 points are erratic
        m = make_pipeline(StandardScaler(),
                          MLPRegressor(hidden_layer_sizes=(8,), activation="tanh", alpha=1e-3,
                                       max_iter=20000, random_state=seed))
        m.fit(X, y)
        preds.append(m)
    return lambda Xn: np.mean([p.predict(Xn) for p in preds], axis=0)


MODELS = {"physics a*sqrt(dL)": fit_physics, "sqrt+b": fit_sqrt_b,
          "poly1 (linear dL)": fit_poly(1), "poly2": fit_poly(2),
          "rotor v0+c*rps": fit_rotor, "MLP [dL,rps]": fit_mlp}


def rmse(p, y):
    m = ~np.isnan(p)
    return np.sqrt(np.mean((p[m] - y[m]) ** 2)) if m.any() else np.nan


def cross_validate(X, y):
    print("\nLeave-one-segment-out RMSE (m/s) - trained without that segment:")
    print(f"{'model':22s}" + "".join(f"{s:>9s}" for s in SEGMENTS) + f"{'train-all':>11s}")
    for name, fit in MODELS.items():
        row = []
        for seg, idx in SEGMENTS.items():
            te = np.zeros(len(y), bool); te[list(idx)] = True
            f = fit(X[~te], y[~te])
            row.append(rmse(f(X[te]), y[te]) if f else np.nan)
        f = fit(X, y)
        print(f"{name:22s}" + "".join(f"{r:9.3f}" for r in row) + f"{rmse(f(X), y):11.3f}")


# ---------------------------------------------------------------- Kalman
def kalman_velocity(rel, dL, a, q=0.05, r_counts=1500.0):
    """1-state Kalman filter on s = sqrt(dL) (~ v/a), random-walk process.
    q: process noise of v in (m/s)^2 per second; r_counts: load noise (counts).
    q sweep on FAN4: q=4 -> plateau SD 0.19 m/s; q=0.05 -> 0.13 (~= 0.5 s average)."""
    dt = 1 / 200
    s_meas = np.sqrt(np.clip(dL, 0, None))
    x, P = s_meas[0], 1.0
    Q = q * dt / a**2
    out = np.empty_like(s_meas)
    for i, z in enumerate(s_meas):
        P += Q
        # measurement noise in sqrt-space: d(sqrt L) = dL / (2 sqrt L)
        R = (r_counts / (2 * max(z, 30.0))) ** 2
        K = P / (P + R)
        x += K * (z - x)
        P *= 1 - K
        out[i] = x
    return a * out


def main():
    rel, dL, turns, k, v = load()
    r, off, win = sync(rel, dL, turns, k, v)
    print(f"sync: best r={r:.4f} at offset={off:.2f}s, display window={win}s "
          f"(using OFFSET={OFFSET}, AVG_WIN={AVG_WIN})")
    X = features(rel, dL, turns, k)
    print("\nper-segment means:   GT v    dL(counts)   rotor rps")
    for seg, idx in SEGMENTS.items():
        i = list(idx)
        print(f"  {seg:8s}       {v[i].mean():6.2f}  {X[i,0].mean():10.0f}  {X[i,1].mean():8.2f}")
    cross_validate(X, v)

    a_phys = None
    f = fit_physics(X, v)
    a_phys = f(np.array([[1e4, 0]]))[0] / 100.0
    print(f"\nphysics fit (all data): v = {a_phys:.5f} * sqrt(dL)   "
          f"=> dL = {1/a_phys**2:.1f} * v^2 counts")

    # Kalman vs moving average, scored on the anemometer windows
    vk = kalman_velocity(rel, dL, a_phys)
    s = a_phys * np.sqrt(np.clip(dL, 0, None))
    ma = np.convolve(s, np.ones(200) / 200, "same")
    for name, sig in (("raw 200 Hz", s), ("1 s moving avg", ma), ("Kalman", vk)):
        p = np.array([sig[(rel >= kk - 0.38 + OFFSET - AVG_WIN) & (rel < kk - 0.38 + OFFSET)].mean() for kk in k])
        steady = (rel > 40) & (rel < 62)
        print(f"  {name:15s} window-mean RMSE {rmse(p, v):.3f} m/s, "
              f"sample SD on p1 plateau {sig[steady].std():.3f} m/s")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 7))
        ax[0].plot(rel - OFFSET, s, lw=0.3, color="0.75", label="a*sqrt(dL) 200 Hz")
        ax[0].plot(rel - OFFSET, vk, lw=1.2, label="Kalman")
        ax[0].step(k - 0.38, v, where="post", color="k", lw=1.5, label="anemometer")
        ax[0].set(xlim=(-2, 61), ylim=(-0.5, 10), xlabel="video time (s)", ylabel="v (m/s)")
        ax[0].legend(loc="upper left")
        ax[1].scatter(X[:, 0], v, c=k, cmap="viridis", s=18)
        xx = np.linspace(0, X[:, 0].max() * 1.15, 200)
        ax[1].plot(xx, a_phys * np.sqrt(xx), "r", label=f"v = {a_phys:.4f} sqrt(dL)")
        ax[1].set(xlabel="dL (load counts above fan-off)", ylabel="anemometer v (m/s)")
        ax[1].legend()
        fig.tight_layout()
        out = os.path.join(LOGS, "fan4_fit.png")
        fig.savefig(out, dpi=120)
        print("plot:", out)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
