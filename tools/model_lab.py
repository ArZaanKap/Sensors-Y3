"""Model lab: replay airspeed models OFFLINE on a recorded good-tunnel run and compare
them with the tunnel ground truth (VDAS). Does not touch the board, the firmware or
demo_day.py. Stdlib only.

  & $py tools/model_lab.py            # 2026-11-05 calibration run -> plots/

Writes plots/<date>_good-tunnel_model-comparison.html (interactive) and, with
--png, one PNG per section via headless Edge.

Models, all replayed on the board's raw 100 Hz load log. Every model fits scale k and
output delay D (lowest MSE); floor fixed at the demo value unless stated:
  1 current     force Kalman (firmware's q, r), then v = S*sqrt(F - DEAD)
  2 kf_rts      same Kalman + fixed-lag RTS smoother: the output for time t is
                recomputed using the readings up to t + D (the delay we pay anyway)
  3 ekf         physics EKF: state = speed; measurement F = DEAD + v^2/S^2
  4 ekf_rts     the EKF + the same fixed-lag smoother
  5 b_exp       idea B1: exponent N fitted (board: n command)
  6 b_all       idea B2: N, dead-band and Kalman q tuned together (n, d, q)
  7 c_vdas      idea C (rejected): output the tunnel's expected reading, treating its
                fan-off spikes as pressure noise present at every speed
  8 b_floor     B1 + floor = mean fan-off tunnel reading, from data before the span
Each model also gets a held-out MSE: fitted on one half of the span, scored on the
other half (and vice versa), so extra parameters can't win just by fitting noise.
"""
import bisect, csv, json, math, os, re, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
LOGS = os.path.join(ROOT, "demoday_logs")
PLOTS = os.path.join(ROOT, "plots")
RUN = os.path.join(LOGS, "run_20261105_092221")
RAW = os.path.join(LOGS, "raw_20261105_092538.csv")
PREFIX = "2026-11-05_good-tunnel"

# ------------------------------------------------------------------ data
def series(path, col="value_raw_mps"):
    t, v = [], []
    for r in csv.DictReader(open(path, newline="")):
        try:
            t.append(float(r["timestamp_epoch"])); v.append(float(r[col]))
        except (KeyError, ValueError):
            pass
    return t, v

tg, g = series(RUN + "_vdas.csv")
tu, u = series(RUN + "_student.csv")
fit_txt = open(RUN + "_fit.txt").read()
K_BOARD, LAG_BOARD, FLOOR = (float(x) for x in re.search(r"k ([\d.]+)\s+lag (\d+) ms\s+floor ([\d.]+)", fit_txt).groups())
LAG_BOARD /= 1000
T0 = tg[0]
S_BOARD = 0.0509 * K_BOARD                     # S on the board now (0.036196)
DEAD = 300.0

head = open(RAW).readline()
L0 = float(re.search(r"L0=(-?[\d.]+)", head).group(1))
DT = int(re.search(r"dt_us=(\d+)", head).group(1)) / 1e6
raw = [r for r in csv.reader(open(RAW)) if r and not r[0].startswith("#")][1:]
tr = [float(r[0]) for r in raw]
z = [None if int(r[2]) == -99999 else int(r[2]) - L0 for r in raw]

T_FROM = tr[0] + 2.0                           # comparison span: raw log starts 8.4 s in; 2 s to settle
gt_span = [(t, v) for t, v in zip(tg, g) if t >= T_FROM]

def interp(ts, vs, x):
    if x < ts[0] or x > ts[-1]:
        return None
    i = bisect.bisect_left(ts, x)
    if ts[i] == x:
        return vs[i]
    return vs[i - 1] + (vs[i] - vs[i - 1]) * (x - ts[i - 1]) / (ts[i] - ts[i - 1])

# ------------------------------------------------------------------ filters
Q_F, R_200 = 1e7, 2.25e6                      # firmware values (200 Hz)
R = R_200 / 2                                  # 100 Hz log: each sample stands for two 200 Hz readings

def qmat(q):
    return (q * DT ** 3 / 3, q * DT ** 2 / 2, q * DT)

def run_filter(h, H, q, x_init, P_init):
    """Generic (E)KF on state [x, xdot] with F = [[1, DT], [0, 1]].
    h(x0) = predicted measurement, H(x0) = dh/dx0. Stores what RTS needs."""
    q00, q01, q11 = qmat(q)
    x0, x1 = x_init
    p00, p01, p11 = P_init
    xf, xp, C = [], [], []
    Pf = []
    for zk in z:
        # predict
        a0, a1 = x0 + DT * x1, x1
        n00 = p00 + DT * (2 * p01 + DT * p11) + q00
        n01 = p01 + DT * p11 + q01
        n11 = p11 + q11
        xp.append((a0, a1))
        if zk is not None:
            Hk = H(a0)
            s = Hk * Hk * n00 + R
            k0, k1 = n00 * Hk / s, n01 * Hk / s
            e = zk - h(a0)
            x0, x1 = a0 + k0 * e, a1 + k1 * e
            p00, p01, p11 = (1 - k0 * Hk) * n00, (1 - k0 * Hk) * n01, n11 - k1 * Hk * n01
        else:
            x0, x1, p00, p01, p11 = a0, a1, n00, n01, n11
        xf.append((x0, x1))
        Pf.append((p00, p01, p11))
    # RTS gains C_j = Pf_j F' inv(Pp_{j+1}); Pp_{j+1} = F Pf_j F' + Q
    for j in range(len(z) - 1):
        p00, p01, p11 = Pf[j]
        m00 = p00 + DT * (2 * p01 + DT * p11) + q00          # Pp_{j+1}
        m01 = p01 + DT * p11 + q01
        m11 = p11 + q11
        b00, b01, b10, b11 = p00 + DT * p01, p01, p01 + DT * p11, p11   # Pf F'
        det = m00 * m11 - m01 * m01
        i00, i01, i11 = m11 / det, -m01 / det, m00 / det
        C.append((b00 * i00 + b01 * i01, b00 * i01 + b01 * i11,
                  b10 * i00 + b11 * i01, b10 * i01 + b11 * i11))
    return xf, xp, C

def fixed_lag(xf, xp, C, N):
    """est[i] = smoothed state of sample i using samples up to i + N (available at time i + N)."""
    n = len(xf)
    est = [None] * n
    for k in range(N, n):
        s0, s1 = xf[k]
        for j in range(k - 1, k - N - 1, -1):
            d0, d1 = s0 - xp[j + 1][0], s1 - xp[j + 1][1]
            c = C[j]
            s0 = xf[j][0] + c[0] * d0 + c[1] * d1
            s1 = xf[j][1] + c[2] * d0 + c[3] * d1
        est[k - N] = (s0, s1)
    return est

# ------------------------------------------------------------------ fit + score
# SPAN = the scored tunnel readings. HALF_A / HALF_B split it in time for held-out scores:
# fit on one half, score on the other, so extra parameters can't win by overfitting.
SPAN = gt_span
_mid = SPAN[len(SPAN) // 2][0]
HALF_A = [p for p in SPAN if p[0] < _mid]
HALF_B = [p for p in SPAN if p[0] >= _mid]

def score_mse(vals_at, span):
    e = [(vals_at(t), gv) for t, gv in span]
    e = [(m - gv) ** 2 for m, gv in e if m is not None]
    return sum(e) / len(e)

def score(vals_at, span=SPAN):
    """vals_at(t) -> model output at time t (None if unavailable)."""
    pts = [(t - T0, gv, vals_at(t)) for t, gv in span]
    pts = [p for p in pts if p[2] is not None]
    err = [m - gv for _, gv, m in pts]
    mse = sum(e * e for e in err) / len(err)
    bands = []
    for lo in (0, 2, 4, 6, 8, 10):
        e = [m - gv for _, gv, m in pts if (lo <= gv < lo + 2) or (lo == 10 and gv >= 10)]
        if e:
            bands.append({"band": f"{lo}–{lo + 2}" if lo < 10 else "10+", "n": len(e),
                          "bias": sum(e) / len(e), "rms": math.sqrt(sum(x * x for x in e) / len(e))})
    return {"pts": pts, "mse": mse, "rms": math.sqrt(mse), "bias": sum(err) / len(err), "n": len(pts), "bands": bands}

def fit_scale(base, span):
    """Least-squares scale k on readings with the fan clearly on (tunnel > 2 m/s)."""
    pairs = [(gv, base(t)) for t, gv in span]
    pairs = [(a, b) for a, b in pairs if b is not None and a > 2 and b > 0]
    return sum(a * b for a, b in pairs) / sum(b * b for _, b in pairs)

def make_model(speed_series, delay):
    """speed_series[i] = estimate of the wind at tr[i]; output at time t = that estimate for t - delay."""
    ok = [(t, v) for t, v in zip(tr, speed_series) if v is not None]
    ts, vs = [p[0] for p in ok], [p[1] for p in ok]
    return lambda t: interp(ts, vs, t - delay)

def floor_map(v):
    return max(v, FLOOR)

def build(series, delay, k, out_map):
    base = make_model(series, delay)
    return lambda t: (lambda v: None if v is None else out_map(k * v))(base(t))

def fit_kd(cands, span, out_map=floor_map):
    """cands: list of (delay, series). Scale k by least squares, delay by lowest MSE."""
    best = None
    for delay, sp in cands:
        base = make_model(sp, delay)
        k = fit_scale(base, span)
        m = score_mse(lambda t: (lambda v: None if v is None else out_map(k * v))(base(t)), span)
        if best is None or m < best[0]:
            best = (m, delay, k, sp)
    _, delay, k, sp = best
    if out_map is not floor_map:
        # a nonlinear output map shifts the output, so least-squares k on the raw speed is
        # no longer optimal: refine k for the mapped output (golden section, +-15%)
        base = make_model(sp, delay)
        f = lambda kk: score_mse(lambda t: (lambda v: None if v is None else out_map(kk * v))(base(t)), span)
        lo, hi, gr = 0.85 * k, 1.15 * k, (math.sqrt(5) - 1) / 2
        for _ in range(14):
            a, b = hi - gr * (hi - lo), lo + gr * (hi - lo)
            if f(a) < f(b):
                hi = b
            else:
                lo = a
        k = (lo + hi) / 2
    return {"delay": delay, "k": k, "series": sp, "model": build(sp, delay, k, out_map)}

def holdout(fitter):
    """Fit on each half, score on the other half; pooled MSE over both held-out halves."""
    a, b = fitter(HALF_A), fitter(HALF_B)
    e = [(a["model"](t) - gv) ** 2 for t, gv in HALF_B if a["model"](t) is not None] + \
        [(b["model"](t) - gv) ** 2 for t, gv in HALF_A if b["model"](t) is not None]
    return sum(e) / len(e)

def force_to_speed(f, S=S_BOARD):
    return S * math.sqrt(max(f - DEAD, 0.0))

# ------------------------------------------------------------------ the models
print("forward Kalman (force) ...")
kf = run_filter(lambda x: x, lambda x: 1.0, Q_F, (0.0, 0.0), (R, 0.0, 1e10))
v_kf = [force_to_speed(x[0]) for x in kf[0]]

C_PHYS = 1 / S_BOARD ** 2
V_REF = 6.0                                     # q matched to the force filter at 6 m/s
Q_V = Q_F / (2 * C_PHYS * V_REF) ** 2
print(f"EKF: c = 1/S^2 = {C_PHYS:.0f} counts/(m/s)^2, q_v = {Q_V:.3f} (m/s)^2/s^3")
ekf = run_filter(lambda v: DEAD + C_PHYS * v * abs(v), lambda v: 2 * C_PHYS * abs(v) + 1e-6,
                 Q_V, (1.0, 0.0), (1.0, 0.0, 10.0))
v_ekf = [x[0] for x in ekf[0]]

forward_delays = [i * 0.025 for i in range(0, 41)]
lag_ns = list(range(5, 81, 5))                  # smoother lags 50..800 ms

cands1 = [(d, v_kf) for d in forward_delays]
print("smoothing (models 2, 4) ...")
cands2 = [(N * DT, [None if s is None else force_to_speed(s[0]) for s in fixed_lag(*kf, N)]) for N in lag_ns]
cands3 = [(d, v_ekf) for d in forward_delays]
cands4 = [(N * DT, [None if s is None else s[0] for s in fixed_lag(*ekf, N)]) for N in lag_ns]

# ---- idea C: imitate the tunnel's reading. VDAS shows sqrt(2|p + e| / rho), where p is the
# true (negative) pitot pressure and e its error: the leftover zero offset plus whole-Pa steps.
# The fan-off readings ARE samples of e, so the expected tunnel reading at true speed v is
#   E[ sqrt(|v^2 - 2e/rho|) ]  over those samples (+ optional extra Gaussian noise sigma, in Pa).
vrows = list(csv.DictReader(open(RUN + "_vdas.csv", newline="")))
rho_s = sorted(2 * abs(float(r["pa_corrected"])) / float(r["value_raw_mps"]) ** 2
               for r in vrows if float(r["value_raw_mps"]) > 3)
RHO = rho_s[len(rho_s) // 2]
rest_e = [float(r["pa_corrected"]) for r in vrows
          if (lambda x: x is not None and x < 0.3)(interp(tu, u, float(r["timestamp_epoch"]) - LAG_BOARD))]
REST_E = [(e, rest_e.count(e) / len(rest_e)) for e in sorted(set(rest_e))]
REST_MEAN = sum(math.sqrt(2 * abs(e) / RHO) * w for e, w in REST_E)
# MSE-best floor = mean fan-off reading (spikes included). Taken only from the fan-off readings
# BEFORE the scored span, so the floor isn't fitted on the data it is scored on.
_pre = [gv for t, gv in zip(tg, g) if t < T_FROM and
        (lambda x: x is not None and x < 0.3)(interp(tu, u, t - LAG_BOARD))]
FLOOR_REST = sum(_pre) / len(_pre)
print(f"floor from {len(_pre)} fan-off readings before the span: {FLOOR_REST:.3f} m/s (board uses {FLOOR:.3f})")
print(f"tunnel: rho = {RHO:.3f} kg/m3, fan-off error e (Pa: share) = "
      + ", ".join(f"{e:+.2f}: {w:.0%}" for e, w in REST_E) + f"  -> mean fan-off reading {REST_MEAN:.3f} m/s")
GH = [(-2, 0.0545), (-1, 0.2442), (0, 0.4026), (1, 0.2442), (2, 0.0545)]   # 5-point normal

def vdas_map_factory(sigma):
    nodes = [(e + sigma * zz, w * wz) for e, w in REST_E for zz, wz in (GH if sigma > 0 else [(0, 1.0)])]
    return lambda v: sum(w * math.sqrt(abs(v * v - 2 * e / RHO)) for e, w in nodes)

# ---- idea B: tune the pipeline's own parameters (all already settable on the board: n, d, q)
_kf_cache = {}
def kf_force(q):
    if q not in _kf_cache:
        _kf_cache[q] = [x[0] for x in run_filter(lambda x: x, lambda x: 1.0, q, (0.0, 0.0), (R, 0.0, 1e10))[0]]
    return _kf_cache[q]

TUNE_DELAYS = [0.15 + i * 0.025 for i in range(23)]          # 150..700 ms
START = {"q": Q_F, "N": 0.5, "dead": DEAD}

def tune(span, grids, extra=None):
    """Coordinate descent over `grids`; k and delay are refitted inside every trial."""
    p = dict(START, **(extra or {}))
    def evaluate(p):
        sp = [max(x - p["dead"], 0.0) ** p["N"] for x in kf_force(p["q"])]   # unscaled: k absorbs S
        om = vdas_map_factory(p["sigma"]) if "sigma" in p else \
            (floor_map if "floor" not in p else (lambda v, fl=p["floor"]: max(v, fl)))
        r = fit_kd([(d, sp) for d in TUNE_DELAYS], span, om)
        return score_mse(r["model"], span), r
    best_m, best_r = evaluate(p)
    for _ in range(3):
        changed = False
        for name, grid in grids.items():
            for val in grid:
                if val == p[name]:
                    continue
                trial = dict(p, **{name: val})
                m, r = evaluate(trial)
                if m < best_m - 1e-9:
                    best_m, best_r, p, changed = m, r, trial, True
        if not changed:
            break
    best_r["params"] = p
    return best_r

G_N = [round(0.40 + 0.01 * i, 2) for i in range(31)]
G_DEAD = [0, 100, 200, 300, 500, 800, 1200]
G_Q = [1e5, 1e6, 3e6, 1e7, 3e7, 1e8, 1e9]
G_SIG = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]

FITTERS = [
    ("current", lambda s: fit_kd(cands1, s)),
    ("kf_rts", lambda s: fit_kd(cands2, s)),
    ("ekf", lambda s: fit_kd(cands3, s)),
    ("ekf_rts", lambda s: fit_kd(cands4, s)),
    ("b_exp", lambda s: tune(s, {"N": G_N})),
    ("b_all", lambda s: tune(s, {"N": G_N, "dead": G_DEAD, "q": G_Q})),
    ("c_vdas", lambda s: tune(s, {"sigma": G_SIG}, {"sigma": 0.0})),
    ("b_floor", lambda s: tune(s, {"N": G_N}, {"floor": FLOOR_REST})),
]
FIT = {}
for key, fitter in FITTERS:
    print(f"fitting {key} (whole span + both halves) ...")
    r = fitter(SPAN)
    FIT[key] = dict(score(r["model"]), **r, holdout=holdout(fitter))
m1, m2, m3, m4 = FIT["current"], FIT["kf_rts"], FIT["ekf"], FIT["ekf_rts"]

# the board's real stream on the same span (no refit), for reference
board_model = lambda t: (lambda x: None if x is None else max(K_BOARD * x, FLOOR))(interp(tu, u, t - LAG_BOARD))
board_span = score(board_model)
board_all_pts = [(gv - 0, board_model(t)) for t, gv in zip(tg, g)]
board_all = sum((m - gv) ** 2 for gv, m in board_all_pts if m is not None) / sum(1 for _, m in board_all_pts if m is not None)

def ptxt(p):
    s = f"N = {p['N']:.2f}, dead-band {p['dead']:.0f} counts, q = {p['q']:.0e}"
    return s + (f", σ = {p['sigma']:.1f} Pa" if "sigma" in p else "")

pb1, pb2, pc, pbf = (FIT[k]["params"] for k in ("b_exp", "b_all", "c_vdas", "b_floor"))
MODELS = [
    ("current", "Current method (Kalman on force)",
     "v = k·S·√(F̂ − 300), F̂ = live Kalman estimate of the force",
     "What runs on the board now, replayed from the raw log and refitted on this span like the others."),
    ("kf_rts", "Kalman + fixed-lag smoother (idea A)",
     "same as 1, but F̂(t) is re-estimated once the readings up to t + D have arrived",
     "We delay the output by D anyway. The smoother uses the readings that arrive during that wait to correct its estimate."),
    ("ekf", "Physics EKF (speed as the state)",
     "state v, v̇;  measurement  F = 300 + v·|v| / S²  (drag law inside the filter)",
     "The filter tracks speed directly and knows force grows with v², so it smooths more at low speed (where a force step means little) and less at high speed."),
    ("ekf_rts", "Physics EKF + fixed-lag smoother",
     "model 3 plus the same backward pass as model 2",
     "Both ideas together."),
    ("b_exp", "Fitted exponent (idea B1)",
     f"v = k·(F̂ − 300)^N with N = {pb1['N']:.2f} instead of ½",
     "Same filter; only the power law is fitted. Board-ready: the firmware's n command sets N, no code change."),
    ("b_all", "Exponent + dead-band + filter tuned (idea B2)",
     f"v = k·(F̂ − dead)^N, {ptxt(pb2)}",
     "N, the dead-band and the Kalman q (how fast the force may change) tuned together. Board-ready: n, d and q commands."),
    ("c_vdas", "Imitate the tunnel's noise at every speed (idea C, rejected)",
     f"output = E[ √|v² − 2e/ρ| ] over the tunnel's fan-off errors e, σ = {pc['sigma']:.1f} Pa (replaces the floor)",
     "Assumes the fan-off spikes are pressure noise that is also added when the fan runs, which would push the tunnel's reading up at low speed. "
     "The data says no: most spikes happen while the fan spins up or coasts (real air moving), and the tunnel does not read high at 2–4 m/s. Kept as a negative result."),
    ("b_floor", "Fitted exponent + floor = mean fan-off reading (B1 + C-lite)",
     f"v = max(k·(F̂ − 300)^{pbf['N']:.2f}, {FLOOR_REST:.2f})",
     f"Model 5 with the floor set to the mean of the tunnel's fan-off readings, spikes included ({FLOOR_REST:.2f} m/s, from the fan-off before the scored span). "
     "Board-ready: n and f commands."),
]
for key, name, *_ in MODELS:
    m = FIT[key]
    print(f"  {name:46s} MSE {m['mse']:.4f}  held-out {m['holdout']:.4f}  k {m['k']:.3f}  delay {m['delay'] * 1000:4.0f} ms"
          + (f"  {ptxt(m['params'])}" if "params" in m else ""))
print(f"  board stream as sent (no refit)          MSE {board_span['mse']:.4f} on the span, {board_all:.4f} whole run")

# ------------------------------------------------------------------ explainer for idea A
# 6 s window around the fastest change in the tunnel reading inside the span
slopes = [(abs(g[i + 4] - g[i - 4]), tg[i]) for i in range(4, len(g) - 4) if tg[i] > T_FROM + 3 and tg[i] < tr[-1] - 5]
t_mid = max(slopes)[1]
wa, wb = t_mid - 3, t_mid + 3
def jitter(s):
    """RMS change per sample while the fan was on (high-frequency noise)."""
    idx = [i for i, t in enumerate(tr[:-1]) if T_FROM <= t <= tg[-1] - 4]
    d = [s[i + 1] - s[i] for i in idx if s[i] is not None and s[i + 1] is not None]
    return math.sqrt(sum(x * x for x in d) / len(d))

live = v_kf
smooth = m2["series"]
expl = {
    "raw": [[round(t - T0, 3), round(force_to_speed(zz), 3)] for t, zz in zip(tr, z) if zz is not None and wa <= t <= wb],
    "live": [[round(t - T0, 3), round(v, 3)] for t, v in zip(tr, live) if wa <= t <= wb],
    "smooth": [[round(t - T0, 3), round(v, 3)] for t, v in zip(tr, smooth) if v is not None and wa <= t <= wb],
    "gt": [[round(t - T0 - m2["delay"], 3), round(v, 3)] for t, v in zip(tg, g) if wa - 1 <= t - m2["delay"] <= wb + 1],
    "delay_ms": round(m2["delay"] * 1000),
    "jitter_ratio": round(jitter(live) / jitter(smooth), 1),
    "from": round(wa - T0, 2), "to": round(wb - T0, 2),
}

# ------------------------------------------------------------------ page
def line50(model):
    out, t = [], T_FROM
    while t <= tg[-1]:
        v = model(t)
        if v is not None:
            out.append([round(t - T0, 3), round(v, 3)])
        t += 0.02
    return out

def binned_error(pts, width=0.5):
    bins = {}
    for _, gv, m in pts:
        bins.setdefault(int(gv // width), []).append(m - gv)
    return [[round((b + 0.5) * width, 2), round(sum(e) / len(e), 3), len(e)] for b, e in sorted(bins.items()) if len(e) >= 3]

# C explainer: low-speed tunnel readings against the exponent-corrected speed estimate (model 5,
# before any floor/map), with the three candidate output maps drawn on top.
b = FIT["b_exp"]
b_base = make_model(b["series"], b["delay"])
c_low = [[round(b["k"] * b_base(t), 3), round(gv, 3)] for t, gv in zip(tg, g)
         if b_base(t) is not None and b["k"] * b_base(t) < 4.5]
c_map = vdas_map_factory(FIT["c_vdas"]["params"]["sigma"])
xs = [i * 0.05 for i in range(91)]
expl_b = {"current": binned_error(m1["pts"]), "b_exp": binned_error(FIT["b_exp"]["pts"]),
          "N": FIT["b_exp"]["params"]["N"]}
expl_c = {"floor": [[round(v, 2), round(floor_map(v), 3)] for v in xs],
          "floor_rest": [[round(v, 2), round(max(v, FLOOR_REST), 3)] for v in xs],
          "vdas": [[round(v, 2), round(c_map(v), 3)] for v in xs],
          "pts": c_low, "floor_board": FLOOR, "floor_value": FLOOR_REST,
          "rest": [[e, w] for e, w in REST_E]}

data = {
    "run": os.path.basename(RUN),
    "span_from": round(T_FROM - T0, 1),
    "gt": [[round(t - T0, 3), round(v, 3)] for t, v in zip(tg, g)],
    "board": {"mse_span": board_span["mse"], "mse_all": board_all},
    "explainer": expl, "explainB": expl_b, "explainC": expl_c,
    "models": [{
        "id": key, "name": name, "formula": formula, "about": about,
        "k": m["k"], "delay_ms": round(m["delay"] * 1000), "mse": m["mse"], "rms": m["rms"], "bias": m["bias"],
        "holdout": m["holdout"], "params": m.get("params"),
        "n": m["n"], "bands": m["bands"], "line": line50(m["model"]),
        "pts": [[round(a, 3), round(b, 3), round(cc, 3)] for a, b, cc in m["pts"]],
    } for key, name, formula, about in MODELS for m in [FIT[key]]],
}

os.makedirs(PLOTS, exist_ok=True)
html_path = os.path.join(PLOTS, f"{PREFIX}_model-comparison.html")
tpl = open(os.path.join(HERE, "model_lab_template.html"), encoding="utf-8").read()
open(html_path, "w", encoding="utf-8").write(tpl.replace("/*DATA*/null", json.dumps(data)))
print("wrote", html_path)

# ------------------------------------------------------------------ PNGs (headless Edge)
if "--png" in sys.argv:
    edge = next((p for p in (os.path.expandvars(r"%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"),
                             os.path.expandvars(r"%ProgramFiles%\Microsoft\Edge\Application\msedge.exe")) if os.path.exists(p)), None)
    if not edge:
        sys.exit("Edge not found - open the HTML instead")
    url = "file:///" + html_path.replace("\\", "/")
    for old in os.listdir(PLOTS):                       # regenerate the whole set
        if old.startswith(PREFIX) and old.endswith(".png"):
            os.remove(os.path.join(PLOTS, old))
    shots = [("summary", "00_summary-all-models"), ("explainA", "01_idea-A-smoother-explained"),
             ("explainB", "02_idea-B-exponent-explained"), ("explainC", "03_idea-C-tunnel-reading-explained")] + \
            [(m["id"], f"{i + 4:02d}_model-{i + 1}_{m['id'].replace('_', '-')}") for i, m in enumerate(data["models"])]
    for sec, fname in shots:
        dom = subprocess.run([edge, "--headless=new", "--disable-gpu", "--window-size=1200,1000",
                              "--dump-dom", f"{url}#only={sec}"], capture_output=True, text=True, encoding="utf-8").stdout
        hgt = int(re.search(r'data-h="(\d+)"', dom).group(1)) if re.search(r'data-h="(\d+)"', dom) else 1600
        out = os.path.join(PLOTS, f"{PREFIX}_{fname}.png")
        subprocess.run([edge, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--force-dark-mode=false",
                        f"--window-size=1200,{hgt}", f"--screenshot={out}", f"{url}#only={sec}&theme=light"],
                       capture_output=True)
        print("wrote", out)
