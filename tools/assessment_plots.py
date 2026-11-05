"""Assessment run (team2 log1, 2026-11-05 12:23) - offline what-ifs + PNG plots.

Nothing is fitted on the assessment data. The what-ifs only re-map the speeds the board
logged, using settings fixed before the run (morning calibration) and the two tunnel
offsets read from the app's own files.

  N 1/2  : what ran (S 0.036196, DEAD 300, floor 1.003, delay 350)
  N 0.53 : run-2 settings prepared in the morning (S 0.026294)
  offset fix: the tunnel re-zeroed with offset 0.14 Pa instead of 1.38 Pa at calibration,
              so v_now^2 = v_cal^2 - 2*(1.38-0.14)/rho  (same as raising DEAD by ~1570 counts)

Run from Sensors-Y3:  python tools/assessment_plots.py   -> assessment_plots/*.png
"""
import csv, math, os, re, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUN = os.path.join(ROOT, "demoday_logs", "team2", "team2_log1_20261105_122331")
CAL = os.path.join(ROOT, "demoday_logs", "run_20261105_092221")
OUT = os.path.join(ROOT, "assessment_plots")
S1, FL = 0.036196, 1.003


def rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def offset(vdas):
    return sorted({round(float(r["pa_raw"]) - float(r["pa_corrected"]), 2) for r in vdas})


A = rows(RUN + "_aligned.csv")
V = rows(RUN + "_vdas.csv")
ST = rows(RUN + "_student.csv")
off_now, off_cal = offset(V), offset(rows(CAL + "_vdas.csv"))
assert len(off_now) == 1 and len(off_cal) == 1, (off_now, off_cal)
off_now, off_cal = off_now[0], off_cal[0]
rest = [r for r in V if float(r["pa_corrected"]) > 0.3]
rho = sum(2 * float(r["pa_corrected"]) / float(r["value_raw_mps"]) ** 2 for r in rest) / len(rest)
SHIFT = 2 * (off_cal - off_now) / rho
DEAD_EQ = 300 + SHIFT / S1 ** 2

T0 = float(A[0]["timestamp_epoch"])
tt = [float(r["timestamp_epoch"]) - T0 for r in A]
g = [float(r["ground_truth_mps"]) for r in A]
s = [float(r["student_mps"]) for r in A]


def remap(v, S2, N2, fix):
    if v <= FL + 1e-6:
        return FL
    w = S2 * ((v / S1) ** 2) ** N2
    if fix:
        w = math.sqrt(max(w * w - SHIFT, 0.0))
    return max(w, FL)


CFG = [  # id, label, S, N, offset fix, colour var
    ("asrun", "N ½ – what ran", S1, 0.5, False, "--series-1"),
    ("n53", "N 0.53 (run-2 settings)", 0.026294, 0.53, False, "--series-4"),
    ("fix", "N ½ + offset fix (DEAD 1869)", S1, 0.5, True, "--series-3"),
    ("both", "N 0.53 + offset fix", 0.026294, 0.53, True, "--series-2"),
]
pred = {c[0]: [remap(v, c[2], c[3], c[4]) for v in s] for c in CFG}
ALL = list(range(len(A)))
GE2 = [i for i in ALL if g[i] >= 2]


def mse(cid, ix):
    return sum((pred[cid][i] - g[i]) ** 2 for i in ix) / len(ix)


print(f"offsets: calibration {off_cal} Pa, assessment {off_now} Pa; rho {rho:.3f}; "
      f"v^2 shift {SHIFT:.2f}; equivalent DEAD {DEAD_EQ:.0f}")
print(f"readings: all {len(ALL)}, tunnel >= 2 m/s {len(GE2)}")
for c in CFG:
    print(f"  {c[1]:28s} MSE all {mse(c[0], ALL):.4f}   tunnel>=2 {mse(c[0], GE2):.4f}")

# ------------------------------------------------------------------ svg helpers
W = 1040


def esc(x):
    return str(x).replace("&", "&amp;").replace("<", "&lt;")


class Plot:
    def __init__(self, h, xr, yr, ml=56, mr=24, mt=14, mb=40):
        self.h, self.xr, self.yr, self.ml, self.mr, self.mt, self.mb = h, xr, yr, ml, mr, mt, mb
        self.parts = []

    def X(self, x):
        return self.ml + (x - self.xr[0]) / (self.xr[1] - self.xr[0]) * (W - self.ml - self.mr)

    def Y(self, y):
        return self.mt + (self.yr[1] - y) / (self.yr[1] - self.yr[0]) * (self.h - self.mt - self.mb)

    def axes(self, xt, yt, xlab, ylab, yfmt="{:g}", xfmt="{:g}"):
        p = self.parts
        p.append('<g class="grid">')
        for y in yt:
            p.append(f'<line x1="{self.ml}" x2="{W - self.mr}" y1="{self.Y(y):.1f}" y2="{self.Y(y):.1f}"/>')
        p.append("</g>")
        for y in yt:
            p.append(f'<text x="{self.ml - 8}" y="{self.Y(y) + 4:.1f}" text-anchor="end">{yfmt.format(y)}</text>')
        for x in xt:
            p.append(f'<text x="{self.X(x):.1f}" y="{self.h - self.mb + 18}" text-anchor="middle">{xfmt.format(x)}</text>')
        p.append(f'<line class="base" x1="{self.ml}" x2="{W - self.mr}" y1="{self.Y(self.yr[0]):.1f}" y2="{self.Y(self.yr[0]):.1f}"/>')
        p.append(f'<text class="lbl" x="{(self.ml + W - self.mr) / 2:.0f}" y="{self.h - 4}" text-anchor="middle">{esc(xlab)}</text>')
        p.append(f'<text class="lbl" transform="translate(14,{(self.mt + self.h - self.mb) / 2:.0f}) rotate(-90)" text-anchor="middle">{esc(ylab)}</text>')

    def line(self, xs, ys, color, width=2, dash=None):
        pts = " ".join(f"{self.X(x):.1f},{self.Y(y):.1f}" for x, y in zip(xs, ys))
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(f'<polyline fill="none" stroke="var({color})" stroke-width="{width}" stroke-linejoin="round"{d} points="{pts}"/>')

    def svg(self):
        return f'<svg viewBox="0 0 {W} {self.h}">' + "".join(self.parts) + "</svg>"


def legend(items):
    return '<div class="legend">' + "".join(f'<span><i style="background:var({c})"></i>{esc(l)}</span>' for l, c in items) + "</div>"


CSS = """
:root{--page:#f9f9f7;--surface-1:#fcfcfb;--text-primary:#0b0b0b;--text-secondary:#52514e;--muted:#898781;
--grid:#e1e0d9;--axis:#c3c2b7;--border:rgba(11,11,11,.10);--ink:#52514e;--band:#eeede7;
--series-1:#2a78d6;--series-2:#eb6834;--series-3:#1baf7a;--series-4:#eda100}
*{box-sizing:border-box}body{margin:0;background:var(--page);color:var(--text-primary);
font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
.wrap{width:1100px;padding:24px 30px 28px}
h1{font-size:21px;font-weight:600;margin:0 0 4px}h3{font-size:13px;font-weight:600;margin:0 0 6px;color:var(--text-secondary)}
.sub{color:var(--text-secondary);margin:0 0 14px;max-width:1000px}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px 16px 10px;margin-bottom:12px}
.legend{display:flex;flex-wrap:wrap;gap:18px;font-size:12px;color:var(--text-secondary);margin:0 0 6px}
.legend i{display:inline-block;width:10px;height:10px;vertical-align:middle;margin-right:6px;border-radius:2px}
svg{display:block;width:100%;height:auto}svg text{fill:var(--muted);font-size:11px;font-variant-numeric:tabular-nums}
svg .lbl{fill:var(--text-secondary);font-size:12px}svg .val{fill:var(--text-primary);font-size:12px}
.grid line{stroke:var(--grid);stroke-width:1}.base{stroke:var(--axis);stroke-width:1}
.note{color:var(--text-secondary);font-size:12.5px;margin:6px 0 0;max-width:1000px}
table{border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}
td,th{padding:5px 14px 5px 0;text-align:left;border-bottom:1px solid var(--border)}th{color:var(--text-secondary);font-weight:600}
"""

EDGE = next((p for p in (os.path.expandvars(r"%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"),
                         os.path.expandvars(r"%ProgramFiles%\Microsoft\Edge\Application\msedge.exe")) if os.path.exists(p)), None)


def page(name, title, sub, body):
    html = (f'<!DOCTYPE html><html><head><meta charset="utf-8"><title>{esc(title)}</title><style>{CSS}</style></head>'
            f'<body><div class="wrap" id="wrap"><h1>{esc(title)}</h1><p class="sub">{sub}</p>{body}</div>'
            '<script>document.body.setAttribute("data-h",Math.ceil(document.getElementById("wrap").getBoundingClientRect().height))</script>'
            "</body></html>")
    tmp = os.path.join(OUT, name + ".html")
    png = os.path.join(OUT, name + ".png")
    open(tmp, "w", encoding="utf-8").write(html)
    url = "file:///" + tmp.replace("\\", "/")
    dom = subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--window-size=1100,1000", "--dump-dom", url],
                         capture_output=True, text=True, encoding="utf-8").stdout
    m = re.search(r'data-h="(\d+)"', dom)
    subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                    f"--window-size=1100,{int(m.group(1)) + 2 if m else 1400}", f"--screenshot={png}", url], capture_output=True)
    os.remove(tmp)
    print("wrote", png)


os.makedirs(OUT, exist_ok=True)
TAG = "2026-11-05_assessment"
INTRO = ("Assessment run 1 (team2 log1, 5 Nov 2026, 127 s, white tunnel). The what-ifs are worked out offline from the speeds the board "
         "logged, using settings fixed before the run. Nothing was fitted on this data.")

# ---------------------------------------------------------------- 1. MSE table + bars
groups = [("All readings", ALL), ("Tunnel ≥ 2 m/s only", GE2)]
p = Plot(18 + 2 * 144 + 34, (0, 0.2), (0, 1), ml=210, mb=30, mt=10)
ticks = [0, 0.05, 0.1, 0.15, 0.2]
p.parts.append('<g class="grid">' + "".join(f'<line x1="{p.X(x):.1f}" x2="{p.X(x):.1f}" y1="10" y2="{p.h - 30}"/>' for x in ticks) + "</g>")
for x in ticks:
    p.parts.append(f'<text x="{p.X(x):.1f}" y="{p.h - 12}" text-anchor="middle">{x:.2f}</text>')
y = 18
for gname, ix in groups:
    p.parts.append(f'<text class="lbl" x="0" y="{y + 6}" style="font-weight:600">{esc(gname)} (n = {len(ix)})</text>')
    y += 16
    for c in CFG:
        m_ = mse(c[0], ix)
        x1 = p.X(m_)
        p.parts.append(f'<text class="lbl" x="{p.ml - 10}" y="{y + 15}" text-anchor="end">{esc(c[1])}</text>')
        p.parts.append(f'<path d="M{p.ml},{y + 4} H{x1 - 4:.1f} a4,4 0 0 1 4,4 V{y + 18} a4,4 0 0 1 -4,4 H{p.ml} Z" fill="var({c[5]})"/>')
        p.parts.append(f'<text class="val" x="{x1 + 8:.1f}" y="{y + 17}">{m_:.3f}</text>')
        y += 26
    y += 24
p.parts.append(f'<line class="base" x1="{p.ml}" x2="{p.ml}" y1="10" y2="{p.h - 30}"/>')
tbl = "<table><tr><th>Settings</th><th>MSE, all readings</th><th>MSE, tunnel ≥ 2 m/s</th><th>Change vs what ran (≥ 2 m/s)</th></tr>"
base2 = mse("asrun", GE2)
for c in CFG:
    tbl += (f"<tr><td>{esc(c[1])}</td><td>{mse(c[0], ALL):.3f}</td><td>{mse(c[0], GE2):.3f}</td>"
            f"<td>{(mse(c[0], GE2) / base2 - 1) * 100:+.0f}%</td></tr>")
tbl += "</table>"
page(f"{TAG}_01_mse-what-ifs", "Assessment run 1: MSE for what ran vs the offline what-ifs",
     INTRO + f" Below 2 m/s the tunnel only flickers between whole pascals, so the second score leaves those readings out.",
     f'<div class="card"><h3>MSE vs the tunnel (m/s)² – lower is better</h3>{p.svg()}</div><div class="card">{tbl}'
     f'<p class="note">Offset fix: the tunnel zeroed at {off_now:.2f} Pa in the assessment vs {off_cal:.2f} Pa at our calibration, so it read lower '
     f'at the same air speed. Correction: v² − {SHIFT:.2f}, which for N ½ is exactly the board command d{DEAD_EQ:.0f} (DEAD 300 → {DEAD_EQ:.0f}).</p></div>')

# ---------------------------------------------------------------- 2. timeline
ts = [float(r["timestamp_epoch"]) - T0 for r in ST][::10]
us = [float(r["value_raw_mps"]) for r in ST][::10]
ub = [remap(v, 0.026294, 0.53, True) for v in us]
tv = [float(r["timestamp_epoch"]) - T0 for r in V]
gv = [float(r["value_raw_mps"]) for r in V]


def timeline(x0, x1, h):
    q = Plot(h, (x0, x1), (0, 12))
    q.parts.append(f'<rect x="{q.ml}" y="{q.Y(2):.1f}" width="{W - q.ml - q.mr}" height="{q.Y(0) - q.Y(2):.1f}" fill="var(--band)"/>')
    step = 10 if x1 - x0 > 60 else 5
    q.axes(range(int(x0), int(x1) + 1, step), range(0, 13, 2), "time since logging started (s)", "speed (m/s)")
    sel = lambda xs, ys: [(a, b) for a, b in zip(xs, ys) if x0 <= a <= x1]
    for xs, ys, col, wd in ((tv, gv, "--ink", 2.5), (ts, us, "--series-1", 1.6), (ts, ub, "--series-2", 1.6)):
        pts = sel(xs, ys)
        q.line([a for a, _ in pts], [b for _, b in pts], col, wd)
    return q.svg()


leg = legend([("Tunnel (ground truth, 4 Hz)", "--ink"), ("Ours – what ran (N ½)", "--series-1"), ("Ours – N 0.53 + offset fix (what-if)", "--series-2")])
page(f"{TAG}_02_timeline", "Assessment run 1: our speed vs the tunnel over time",
     INTRO + " Top: the whole run. Bottom: zoom on the last 30 s (fan switched off, then on/off pulses).",
     f'<div class="card"><h3>Whole run (shaded = below 2 m/s, left out of the ≥ 2 m/s score)</h3>{leg}{timeline(0, 128, 380)}</div>'
     f'<div class="card"><h3>Zoom: 97–128 s</h3>{leg}{timeline(97, 128, 330)}'
     '<p class="note">What ran reads a little high on most plateaus (6–9 m/s) and low at full speed (~10 m/s). '
     'During spin-up and spin-down both versions read above the tunnel. The fixes shrink the plateau gaps but not that.</p></div>')

# ---------------------------------------------------------------- 3. error by speed band
bands = [(lo, [i for i in ALL if lo <= g[i] < lo + 1]) for lo in range(0, 11)]
bands = [(lo, ix) for lo, ix in bands if len(ix) >= 5]
q = Plot(400, (-0.2, 11.2), (-0.6, 1.2), mb=58)
q.parts.append(f'<rect x="{q.X(-0.2):.1f}" y="{q.mt}" width="{q.X(2) - q.X(-0.2):.1f}" height="{q.h - q.mt - q.mb}" fill="var(--band)"/>')
q.axes(range(0, 12), [-0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2], "tunnel speed (m/s), 1 m/s bands",
       "mean error, ours − tunnel (m/s)", yfmt="{:+.1f}")
q.parts.append(f'<line x1="{q.ml}" x2="{W - q.mr}" y1="{q.Y(0):.1f}" y2="{q.Y(0):.1f}" stroke="var(--ink)" stroke-width="1.2"/>')
for lo, ix in bands:
    q.parts.append(f'<text x="{q.X(lo + 0.5):.1f}" y="{q.h - q.mb + 32}" text-anchor="middle">n={len(ix)}</text>')
for c in CFG:
    xs = [lo + 0.5 for lo, _ in bands]
    ys = [sum(pred[c[0]][i] - g[i] for i in ix) / len(ix) for _, ix in bands]
    q.line(xs, ys, c[5], 2)
    for x, y_ in zip(xs, ys):
        q.parts.append(f'<circle cx="{q.X(x):.1f}" cy="{q.Y(y_):.1f}" r="4.5" fill="var({c[5]})" stroke="var(--surface-1)" stroke-width="2"/>')
page(f"{TAG}_03_error-by-speed", "Assessment run 1: where the error comes from, by speed",
     INTRO + " Each dot is the average of (ours − tunnel) for all tunnel readings in that 1 m/s band. 0 = spot on, above 0 = we read high.",
     f'<div class="card"><h3>Mean error per speed band (shaded = below 2 m/s)</h3>{legend([(c[1], c[5]) for c in CFG])}{q.svg()}'
     '<p class="note">2–6 m/s only happens while the fan spins up or down (few readings, n ≤ 8). We read high there in every version (up to +1 m/s). '
     'The fixes shrink it but do not remove it, so most of it is likely dynamics (plate vs pressure tap while the air speeds up or slows down). '
     'On the 6–9 m/s plateaus the offset fix halves the over-read. At full speed (9–11 m/s) it makes the under-read a little worse, '
     'which N 0.53 partly cancels.</p></div>')

# ---------------------------------------------------------------- 4. offset explained
pr = [x / 2 for x in range(1, 151)]                       # |pressure reading| 0.5..75 Pa (fan on: reading negative)
v_cal = [math.sqrt(2 * (x + off_cal) / rho) for x in pr]
v_now = [math.sqrt(2 * (x + off_now) / rho) for x in pr]
a = Plot(330, (0, 75), (0, 12))
a.axes(range(0, 76, 5), range(0, 13, 2), "tunnel pressure reading, fan on (−Pa)", "speed the app shows (m/s)")
a.line(pr, v_cal, "--series-1")
a.line(pr, v_now, "--series-2")
b = Plot(300, (2, 11), (0, 0.5))
b.axes(range(2, 12), [0, 0.1, 0.2, 0.3, 0.4, 0.5], "speed (m/s)", "gap (m/s)", yfmt="{:.1f}")
gp = [(n_, c_ - n_) for c_, n_ in zip(v_cal, v_now) if 2 <= n_ <= 11]
b.line([x for x, _ in gp], [y for _, y in gp], "--ink")
for vv in (3, 5, 8, 10):
    gap = math.sqrt(vv * vv + SHIFT) - vv
    b.parts.append(f'<circle cx="{b.X(vv):.1f}" cy="{b.Y(gap):.1f}" r="4.5" fill="var(--ink)" stroke="var(--surface-1)" stroke-width="2"/>')
    b.parts.append(f'<text class="val" x="{b.X(vv) + 8:.1f}" y="{b.Y(gap) - 8:.1f}">{gap:.2f} at {vv} m/s</text>')
page(f"{TAG}_04_tunnel-offset-explained", "Why the same air read differently: the tunnel re-zeroes each session",
     f"The tunnel's pressure sensor only gives whole pascals. At the start of each session the app averages a few fan-off readings and calls that "
     f"the offset, then shows speed = √(2·|reading − offset| / ρ). At our calibration the offset was {off_cal:.2f} Pa; in the assessment it was "
     f"{off_now:.2f} Pa (ρ = {rho:.3f} kg/m³ from the app's own rest readings).",
     f'<div class="card"><h3>Same pressure reading, two sessions</h3>'
     f'{legend([(f"Calibration session (offset {off_cal:.2f} Pa)", "--series-1"), (f"Assessment session (offset {off_now:.2f} Pa)", "--series-2")])}{a.svg()}</div>'
     f'<div class="card"><h3>Gap: how much higher the calibration session reads for the same air</h3>{b.svg()}'
     f'<p class="note">Our scale was learned against the calibration session, so in the assessment we read high by this gap. It matters most at low speed. '
     f'Fix with no fitting: subtract {SHIFT:.2f} from v², which for N ½ is the same as setting DEAD to {DEAD_EQ:.0f} (board command d{DEAD_EQ:.0f}) before the run.</p></div>')
