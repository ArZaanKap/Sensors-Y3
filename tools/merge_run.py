"""Merge one tunnel run (app _vdas/_student CSVs) with the board raw recorder dump into
<run>_merged.csv at 100 Hz: force, rotor angle/rpm, mic, board output, VDAS speed/Pa.
Also prints how well rotor, mic and force track the tunnel speed.
Run with the PlatformIO python:  & $py tools/merge_run.py demoday_logs/run_<time> demoday_logs/raw_<time>.csv"""
import csv, bisect, sys, re
if len(sys.argv) != 3:
    sys.exit("usage: merge_run.py demoday_logs/run_<time> demoday_logs/raw_<time>.csv")
RUN, RAW = sys.argv[1], sys.argv[2]
if RUN.endswith("_aligned.csv"): RUN = RUN[: -len("_aligned.csv")]
def ser(p, c):
    t, v = [], []
    for r in csv.DictReader(open(p)):
        try: t.append(float(r["timestamp_epoch"])); v.append(float(r[c]))
        except (KeyError, ValueError): pass
    return t, v
tg, g = ser(RUN + "_vdas.csv", "value_raw_mps")
_, gpa = ser(RUN + "_vdas.csv", "pa_raw")
tu, u = ser(RUN + "_student.csv", "value_raw_mps")
rows = [r for r in csv.reader(open(RAW)) if r and not r[0].startswith("#")][1:]
tr = [float(r[0]) for r in rows]; L = [int(r[2]) for r in rows]; A = [int(r[3]) for r in rows]; M = [int(r[4]) for r in rows]
L0 = int(float(re.search(r"L0=(-?[\d.]+)", open(RAW).readline()).group(1)))
def hold(ts, vs, x):            # last value at or before x (VDAS is a step signal)
    i = bisect.bisect_right(ts, x) - 1
    return vs[i] if 0 <= i and x <= ts[-1] + 0.5 else ""
def near(ts, vs, x):
    i = bisect.bisect_left(ts, x)
    return vs[min(i, len(vs) - 1)] if ts[0] <= x <= ts[-1] else ""
# rotor speed: unwrapped angle step per 10 ms -> rpm, smoothed over 25 samples (0.25 s)
d = [0] + [(((b - a) % 16384) + 8192) % 16384 - 8192 if a >= 0 and b >= 0 else 0 for a, b in zip(A, A[1:])]
rpm_inst = [x / 16384 / 0.01 * 60 for x in d]
W = 25; rpm = []
for i in range(len(d)):
    s = d[max(0, i - W + 1): i + 1]; rpm.append(sum(s) / len(s) / 16384 / 0.01 * 60)
out = RUN + "_merged.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["timestamp_epoch", "t_rel_s", "load_raw", "force_counts_minus_L0", "ang_raw", "rotor_rpm_0p25s", "mic_env",
                "board_output_mps", "vdas_mps", "vdas_pa_raw"])
    for i in range(len(rows)):
        w.writerow([f"{tr[i]:.4f}", f"{tr[i]-tg[0]:.2f}", L[i], L[i] - L0 if L[i] != -99999 else "", A[i], f"{rpm[i]:.1f}", M[i],
                    near(tu, u, tr[i]), hold(tg, g, tr[i]), hold(tg, gpa, tr[i])])
print("merged ->", out, len(rows), "rows")
print("raw window %.1f..%.1f s rel to run start; run spans 0..%.1f s" % (tr[0] - tg[0], tr[-1] - tg[0], tg[-1] - tg[0]))
# per VDAS sample, compare rotor rpm (mean over +-0.5 s, try lags) and mic with GT
import math
def corr(x, y):
    n = len(x); mx = sum(x)/n; my = sum(y)/n
    sx = math.sqrt(sum((a-mx)**2 for a in x)); sy = math.sqrt(sum((b-my)**2 for b in y))
    return sum((a-mx)*(b-my) for a, b in zip(x, y)) / (sx*sy) if sx and sy else float("nan")
def win(arr, t0, t1):
    i, j = bisect.bisect_left(tr, t0), bisect.bisect_left(tr, t1)
    s = arr[i:j]; return sum(s)/len(s) if s else None
F = [l - L0 for l in L]
for lag in (0, 0.35, 0.7, 1.0):
    pts = [(gv, win(rpm, t - lag - .25, t - lag + .25), win(M, t - lag - .25, t - lag + .25), win(F, t-lag-.25, t-lag+.25)) for t, gv in zip(tg, g) if tr[0] + 1.5 < t < tr[-1]]
    pts = [p for p in pts if None not in p]
    print("lag %.2f s  n=%d  corr(GT, rotor rpm)=%.3f  corr(GT, |rpm|)=%.3f  corr(GT, mic)=%.3f  corr(GT, sqrt force)=%.3f" % (
        lag, len(pts), corr([p[0] for p in pts], [p[1] for p in pts]), corr([p[0] for p in pts], [abs(p[1]) for p in pts]),
        corr([p[0] for p in pts], [p[2] for p in pts]), corr([p[0] for p in pts], [math.sqrt(max(p[3], 0)) for p in pts])))
# timeline 5 s
print("t(s)  GT   rpm   mic  force")
for s in range(int(tr[0]-tg[0])+1, int(tg[-1]-tg[0]), 5):
    gg = [v for t, v in zip(tg, g) if s <= t-tg[0] < s+5]
    print("%3d  %4.1f  %6.0f  %4.0f  %6.0f" % (s, sum(gg)/len(gg) if gg else -1, win(rpm, tg[0]+s, tg[0]+s+5), win(M, tg[0]+s, tg[0]+s+5), win(F, tg[0]+s, tg[0]+s+5)))
print("ang -1 count", A.count(-1), " max |step|", max(abs(x) for x in d))

