"""Is the fitted exponent N (v = k * (F - DEAD)^N) real, or overfitting?

Fits N on many different parts of the 2026-11-05 good-tunnel run and scores it on
parts it never saw. Offline only; does not touch the board, firmware or demo_day.py.

  & $py tools/exponent_check.py      -> plots/2026-11-05_good-tunnel_12_exponent-robustness.{html,png}

Leakage rules:
  - fitted on the training part only, every time: N, scale k, output delay D
  - fixed beforehand, never fitted on scored data: zero L0 (taken before the run),
    Kalman q and r (lab values), DEAD = 300 (lab value), floor (mean tunnel reading
    in the fan-off BEFORE the scored span)
  - test parts are contiguous blocks, never random points (neighbouring tunnel
    readings 250 ms apart are almost identical, so random splits leak)
  - a 2 s buffer around each test block is removed from training (covers the output
    delay and the Kalman filter's memory)
"""
import bisect, csv, json, math, os, random, re, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
LOGS, PLOTS = os.path.join(ROOT, "demoday_logs"), os.path.join(ROOT, "plots")
RUN, RAW = os.path.join(LOGS, "run_20261105_092221"), os.path.join(LOGS, "raw_20261105_092538.csv")
OUT = os.path.join(PLOTS, "2026-11-05_good-tunnel_12_exponent-robustness")

Q, R, DEAD = 1e7, 2.25e6 / 2, 300.0            # firmware q; r halved for the 100 Hz log
N_GRID = [round(0.40 + 0.005 * i, 3) for i in range(61)]
D_GRID = [round(0.15 + 0.025 * i, 3) for i in range(23)]
BUFFER, N_BLOCKS, BOOT_BLOCK, BOOT_REPS = 2.0, 5, 5.0, 300

def series(path, col="value_raw_mps"):
    t, v = [], []
    for r in csv.DictReader(open(path, newline="")):
        try:
            t.append(float(r["timestamp_epoch"])); v.append(float(r[col]))
        except (KeyError, ValueError):
            pass
    return t, v

def interp(ts, vs, x):
    if x < ts[0] or x > ts[-1]:
        return None
    i = bisect.bisect_left(ts, x)
    if ts[i] == x:
        return vs[i]
    return vs[i - 1] + (vs[i] - vs[i - 1]) * (x - ts[i - 1]) / (ts[i] - ts[i - 1])

tg, g = series(RUN + "_vdas.csv")
tu, u = series(RUN + "_student.csv")            # only used to find fan-off moments
lag_board = float(re.search(r"lag (\d+) ms", open(RUN + "_fit.txt").read()).group(1)) / 1000
T0 = tg[0]
head = open(RAW).readline()
L0 = float(re.search(r"L0=(-?[\d.]+)", head).group(1))
DT = int(re.search(r"dt_us=(\d+)", head).group(1)) / 1e6
raw = [r for r in csv.reader(open(RAW)) if r and not r[0].startswith("#")][1:]
tr = [float(r[0]) for r in raw]
z = [None if int(r[2]) == -99999 else int(r[2]) - L0 for r in raw]
T_FROM = tr[0] + 2.0

# fixed-in-advance floor: mean tunnel reading while the fan was off, before the scored span
pre = [gv for t, gv in zip(tg, g) if t < T_FROM and (lambda x: x is not None and x < 0.3)(interp(tu, u, t - lag_board))]
FLOOR = sum(pre) / len(pre)

# live Kalman on force, exactly as the firmware (2-state constant velocity)
q00, q01, q11 = Q * DT ** 3 / 3, Q * DT ** 2 / 2, Q * DT
x0 = x1 = 0.0; p00, p01, p11 = R, 0.0, 1e10
F = []
for zk in z:
    x0 += DT * x1
    n00 = p00 + DT * (2 * p01 + DT * p11) + q00; n01 = p01 + DT * p11 + q01; n11 = p11 + q11
    if zk is not None:
        s = n00 + R; k0, k1 = n00 / s, n01 / s; e = zk - x0
        x0 += k0 * e; x1 += k1 * e
        p00, p01, p11 = (1 - k0) * n00, (1 - k0) * n01, n11 - k1 * n01
    else:
        p00, p01, p11 = n00, n01, n11
    F.append(x0 - DEAD)                         # clipped at 0 later (after any drift correction)

# scored tunnel readings
idx = [i for i, t in enumerate(tg) if t >= T_FROM]
TT = [tg[i] for i in idx]
GT = [g[i] for i in idx]
n = len(TT)

# Competing explanation: the load-cell zero drifts during the run. Model it as a linear drift
# c counts/s since the run started (the zero was taken just before): F_true = F - c*(t - T0).
C_GRID = [float(c) for c in range(0, 25, 2)]                    # 0..24 counts/s (~0..2300 by the end)
N_SMALL = [round(0.48 + 0.01 * i, 2) for i in range(13)]        # 0.48..0.60 for the joint fit
KS_N = [(N, 0.0) for N in N_GRID]
KS_HALF = [(0.5, 0.0)]
KS_DRIFT = [(0.5, c) for c in C_GRID]
KS_BOTH = [(N, c) for N in N_SMALL for c in C_GRID]

# precompute the unscaled model output max(F - drift, 0)^N at every scored reading, per (N, c, delay)
print("precomputing ...")
B = {}
for N, c in sorted(set(KS_N + KS_HALF + KS_DRIFT + KS_BOTH)):
    sp = [max(f - c * (t - T0), 0.0) ** N for f, t in zip(F, tr)]
    for d in D_GRID:
        B[N, c, d] = [interp(tr, sp, t - d) for t in TT]

def fit(train, keys=KS_N):
    """Fit k (least squares, fan clearly on), D, and whichever of N / drift c the keys allow, on train only."""
    best = None
    for N, c in keys:
        for d in D_GRID:
            b = B[N, c, d]
            pairs = [(GT[i], b[i]) for i in train if b[i] is not None and GT[i] > 2]
            sbb = sum(y * y for _, y in pairs)
            if not sbb:
                continue
            k = sum(a * y for a, y in pairs) / sbb
            e = [(max(k * b[i], FLOOR) - GT[i]) ** 2 for i in train if b[i] is not None]
            m = sum(e) / len(e)
            if best is None or m < best[0]:
                best = (m, N, c, d, k)
    return {"mse": best[0], "N": best[1], "c": best[2], "d": best[3], "k": best[4]}

def test_mse(p, test):
    b = B[p["N"], p["c"], p["d"]]
    e = [(max(p["k"] * b[i], FLOOR) - GT[i]) ** 2 for i in test if b[i] is not None]
    return sum(e) / len(e), len(e)

def profile(train):
    """Best MSE on train for each N (k, D refitted), relative to the minimum."""
    prof = [(N, fit(train, [(N, 0.0)])["mse"]) for N in N_GRID]
    m = min(v for _, v in prof)
    return [[N, round(v / m, 4)] for N, v in prof]

# ---- contiguous blocks
t_lo, t_hi = TT[0], TT[-1] + 1e-6
edges = [t_lo + (t_hi - t_lo) * j / N_BLOCKS for j in range(N_BLOCKS + 1)]
blocks = [[i for i in range(n) if edges[j] <= TT[i] < edges[j + 1]] for j in range(N_BLOCKS)]
def speed_range(ix):
    v = [GT[i] for i in ix]
    return f"{min(v):.1f}–{max(v):.1f} m/s"

rows, folds = [], []
ALL = list(range(n))
full = fit(ALL)
rows.append({"label": "All of the scored run", "N": full["N"], "group": "all", "range": speed_range(ALL)})
print(f"all data: N = {full['N']:.3f}  D = {full['d'] * 1000:.0f} ms  floor {FLOOR:.3f}")

# leave-one-block-out: fit on the other blocks (minus a buffer), score on the left-out block
for j, blk in enumerate(blocks):
    a, b_ = edges[j] - BUFFER, edges[j + 1] + BUFFER
    train = [i for i in ALL if not (a <= TT[i] < b_)]
    p_fit, p_half, p_drift, p_both = fit(train), fit(train, KS_HALF), fit(train, KS_DRIFT), fit(train, KS_BOTH)
    m_fit, cnt = test_mse(p_fit, blk)
    m_half, _ = test_mse(p_half, blk)
    m_drift, _ = test_mse(p_drift, blk)
    m_both, _ = test_mse(p_both, blk)
    folds.append({"block": j + 1, "from": round(edges[j] - T0, 1), "to": round(edges[j + 1] - T0, 1),
                  "range": speed_range(blk), "N": p_fit["N"], "c": p_drift["c"], "both_N": p_both["N"], "both_c": p_both["c"],
                  "mse_fit": m_fit, "mse_half": m_half, "mse_drift": m_drift, "mse_both": m_both, "n": cnt})
    rows.append({"label": f"All except block {j + 1}", "N": p_fit["N"], "group": "lobo", "range": ""})
    print(f"block {j + 1} ({edges[j] - T0:5.1f}-{edges[j + 1] - T0:5.1f} s, {speed_range(blk)}): trained N = {p_fit['N']:.3f}, "
          f"drift c = {p_drift['c']:.0f}/s, both N={p_both['N']:.2f} c={p_both['c']:.0f}/s"
          f"   test MSE  N=0.5: {m_half:.4f}  N fit: {m_fit:.4f}  drift: {m_drift:.4f}  both: {m_both:.4f}")

# halves, each fitted alone
half = [i for i in ALL if TT[i] < (t_lo + t_hi) / 2 - BUFFER / 2], [i for i in ALL if TT[i] >= (t_lo + t_hi) / 2 + BUFFER / 2]
for name, h in zip(("First half only", "Second half only"), half):
    rows.append({"label": name, "N": fit(h)["N"], "group": "half", "range": speed_range(h)})

# each block alone (shows how well a short stretch can pin N down)
for j, blk in enumerate(blocks):
    rows.append({"label": f"Block {j + 1} only", "N": fit(blk)["N"], "group": "single", "range": speed_range(blk)})
for r in rows:
    if r["group"] in ("half", "single"):
        print(f"  {r['label']:18s} ({r['range']}): N = {r['N']:.3f}")

# moving-block bootstrap: resample 5 s blocks with replacement, refit N
print(f"bootstrap ({BOOT_REPS} resamples of {BOOT_BLOCK:.0f} s blocks) ...")
bb = []
t = t_lo
while t < t_hi:
    bb.append([i for i in ALL if t <= TT[i] < t + BOOT_BLOCK]); t += BOOT_BLOCK
bb = [x for x in bb if x]
rng = random.Random(20261105)
boot = []
for _ in range(BOOT_REPS):
    sample = [i for blk in (rng.choice(bb) for _ in bb) for i in blk]
    boot.append(fit(sample)["N"])
boot.sort()
ci = (boot[int(0.025 * len(boot))], boot[int(0.975 * len(boot)) - 1])
print(f"bootstrap N: median {boot[len(boot) // 2]:.3f}, 95% interval {ci[0]:.3f}-{ci[1]:.3f}, share > 0.5: {sum(x > 0.5 for x in boot) / len(boot):.0%}")

pool = lambda key: sum(f[key] * f["n"] for f in folds) / sum(f["n"] for f in folds)
pooled_fit, pooled_half, pooled_drift, pooled_both = pool("mse_fit"), pool("mse_half"), pool("mse_drift"), pool("mse_both")
fd, fb = fit(ALL, KS_DRIFT), fit(ALL, KS_BOTH)
print(f"leave-one-block-out pooled test MSE: N=0.5 {pooled_half:.4f} | fitted N {pooled_fit:.4f} | "
      f"N=0.5 + drift {pooled_drift:.4f} | both {pooled_both:.4f}")
print(f"all data: drift-only c = {fd['c']:.0f} counts/s; joint fit N = {fb['N']:.2f}, c = {fb['c']:.0f} counts/s")

# measured zero after the run: force above L0 once the fan is off, before the rig was unmounted
# (an UPPER bound on drift: the air may still be coasting, and the tunnel can't show below 0.8 m/s)
raw_l = [int(r[2]) for r in raw]
t_end = TT[-1]
rest_after = []
for s in range(0, 9):
    seg = [l - L0 for t, l in zip(tr, raw_l) if t_end + s <= t < t_end + s + 1 and l not in (-99999, 0)]
    if len(seg) > 50:
        rest_after.append([round(t_end + s + 0.5 - T0, 1), round(sum(seg) / len(seg))])
needed = {"drift": fd["c"] * (t_end - T0), "both": fb["c"] * (t_end - T0)}
print("measured offset after the run (counts): " + ", ".join(f"{t:.0f} s: {v:+d}" for t, v in rest_after))
print(f"drift the models need by the end of the run: drift-only {needed['drift']:.0f}, joint {needed['both']:.0f} counts")

# same speed, early vs late: with k and D fitted on the whole run for a fixed N, compare the mean
# error in each 1 m/s tunnel-speed band between the first and second half of the run.
# Drift predicts late > early in every band (most at low speed); a wrong exponent predicts no
# early/late difference within a band, only a trend across bands.
t_split = (t_lo + t_hi) / 2
def early_late(N):
    p = fit(ALL, [(N, 0.0)])
    b = B[N, 0.0, p["d"]]
    out = []
    for lo in range(3, 10):
        e = [max(p["k"] * b[i], FLOOR) - GT[i] for i in ALL if b[i] is not None and lo <= GT[i] < lo + 1 and TT[i] < t_split]
        l = [max(p["k"] * b[i], FLOOR) - GT[i] for i in ALL if b[i] is not None and lo <= GT[i] < lo + 1 and TT[i] >= t_split]
        if len(e) >= 4 and len(l) >= 4:
            out.append({"band": f"{lo}–{lo + 1}", "early": sum(e) / len(e), "late": sum(l) / len(l), "ne": len(e), "nl": len(l)})
    return out
el_half, el_fit = early_late(0.5), early_late(0.53)
for name, rows_ in (("N = 0.50", el_half), ("N = 0.53", el_fit)):
    print(f"{name}: " + "  ".join(f"{r['band']}: early {r['early']:+.2f} late {r['late']:+.2f} (n {r['ne']}/{r['nl']})" for r in rows_))

data = {"rows": rows, "rest_after": rest_after, "needed": needed, "early_late": {"half": el_half, "fit": el_fit, "split": round(t_split - T0, 1)}, "folds": folds, "boot": {"median": boot[len(boot) // 2], "lo": ci[0], "hi": ci[1],
        "share_above": sum(x > 0.5 for x in boot) / len(boot), "reps": BOOT_REPS, "hist": boot},
        "profiles": {"all": profile(ALL), "half1": profile(half[0]), "half2": profile(half[1])},
        "pooled": {"fit": pooled_fit, "half": pooled_half, "drift": pooled_drift, "both": pooled_both},
        "all_fit": {"drift_c": fd["c"], "both_N": fb["N"], "both_c": fb["c"]}, "floor": FLOOR, "buffer": BUFFER,
        "span": [round(TT[0] - T0, 1), round(TT[-1] - T0, 1)]}

os.makedirs(PLOTS, exist_ok=True)
tpl = open(os.path.join(HERE, "exponent_check_template.html"), encoding="utf-8").read()
open(OUT + ".html", "w", encoding="utf-8").write(tpl.replace("/*DATA*/null", json.dumps(data)))
edge = next((p for p in (os.path.expandvars(r"%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"),
                         os.path.expandvars(r"%ProgramFiles%\Microsoft\Edge\Application\msedge.exe")) if os.path.exists(p)), None)
if edge:
    url = "file:///" + (OUT + ".html").replace("\\", "/")
    dom = subprocess.run([edge, "--headless=new", "--disable-gpu", "--window-size=1200,1000", "--dump-dom", url],
                         capture_output=True, text=True, encoding="utf-8").stdout
    m = re.search(r'data-h="(\d+)"', dom)
    subprocess.run([edge, "--headless=new", "--disable-gpu", "--hide-scrollbars", f"--window-size=1200,{m.group(1) if m else 2000}",
                    f"--screenshot={OUT}.png", url + "#theme=light"], capture_output=True)
    print("wrote", OUT + ".png")
