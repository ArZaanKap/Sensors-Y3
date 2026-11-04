#!/usr/bin/env python3
"""
demo_day.py - guided calibration for the `airspeed` firmware against the
white tunnel's VDAS, using the official WindTunnelLogger app.

    ~/.platformio/penv/Scripts/python tools/demo_day.py              (real board)
    ~/.platformio/penv/Scripts/python tools/demo_day.py --practice   (no board)

It walks you through every step and tells you what it is doing. No Nucleo
buttons: zeroing is done from here. Only needs pyserial (PlatformIO's Python
has it); no numpy.

Who talks to what:
  - WindTunnelLogger reads the tunnel's VDAS box and your board, and saves runs
    to Documents/Wind Tunnel Logger/logs (<run>_vdas/_student/_aligned.csv).
  - This script talks to the board (zero, read/send parameters). Only one
    program can hold the board's COM port, so the script tells you when to
    Disconnect / Connect the board in the app.

What the fit does with a saved run (VDAS ~4 Hz vs your 200 Hz stream):
  lag    VDAS reacts later than the load cell; delaying our output by the same
         amount lines the two up (matched to the reference, not hidden).
  scale  S_new = k * S_old, k = least squares of VDAS vs ours with the air moving.
  floor  with the fan off the app reads sqrt(2|dp - offset|/rho) ~ 0.7-1.3 m/s,
         never 0; the board outputs at least this much.
"""
import argparse
import csv
import glob
import math
import os
import subprocess
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing. Run with PlatformIO's Python:\n"
             "  ~/.platformio/penv/Scripts/python tools/demo_day.py")

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(os.path.expanduser("~"), "Documents", "Wind Tunnel Logger", "logs")
EXE_CANDIDATES = [os.path.join(HERE, "..", "WindTunnelLogger.exe"),
                  os.path.join(os.path.expanduser("~"), "Downloads", "WindTunnelLogger.exe")]
ST_VID = 0x0483                      # ST-Link virtual COM port

# Mini protocol for the calibration run (my design, not the course's): the
# good tunnel's speed knob gives several levels across the 2-10 m/s range, so
# the scale is fitted on more than 3 points; big up/down steps measure the
# VDAS lag; fan-off at both ends measures the floor and the zero drift.
# The fit doesn't use these timings - skip or stretch them freely.
PROTOCOL = [
    (0, "fan OFF - keep everything still"),
    (10, "FAN ON, knob to MAX"),
    (35, "knob to about 3/4"),
    (50, "knob to about 1/2"),
    (65, "knob to about 1/4 (slow, but fan still running)"),
    (80, "knob back to MAX (big step)"),
    (95, "FAN OFF"),
    (120, "press STOP & SAVE in WindTunnelLogger"),
]


# ======================================================================== ui
def say(msg=""):
    print(msg, flush=True)


def step(title):
    say("\n" + "=" * 70)
    say("  " + title)
    say("=" * 70)


def doing(msg):
    say("  ... " + msg)


def ask(msg, default="y"):
    r = input(f"  >> {msg} [{'Y/n' if default == 'y' else 'y/N'}] ").strip().lower()
    return (r or default).startswith("y")


def wait_enter(msg):
    input(f"  >> {msg}  (press Enter) ")


# ===================================================================== board
class Board:
    """The airspeed firmware over its ST-Link COM port."""

    def __init__(self, port):
        self.port = port
        self.ser = None

    def open(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.3)
                time.sleep(0.2)
                return
            except serial.SerialException as e:
                say(f"  !! cannot open {self.port}: {e}")
                say("     Usually WindTunnelLogger still has it: press DISCONNECT (Your board) in the app.")
                wait_enter("then try again")

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None

    def command(self, line, wait=0.5):
        """Send one command; return the '#' reply lines."""
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        time.sleep(wait)
        out = self.ser.read(self.ser.in_waiting or 1).decode("ascii", "replace")
        return [l.strip() for l in out.splitlines() if l.startswith("#")]

    def params(self):
        vals = {}
        for l in self.command("c", 0.6):
            if "=" in l:
                k, v = l[1:].split("=", 1)
                try:
                    vals[k.strip()] = float(v)
                except ValueError:
                    pass
        return vals

    def calibration_text(self):
        return "\n".join(self.command("c", 0.6))

    def set(self, key, value):
        # each set saves to flash: ~1-2 s with no output
        return self.command(f"{key} {value}", 2.5)

    def zero(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"z\n")
        t0 = time.time()
        buf = ""
        while time.time() - t0 < 6:
            buf += self.ser.read(self.ser.in_waiting or 1).decode("ascii", "replace")
            for l in buf.splitlines():
                if l.startswith("# zero L0"):
                    parts = l.replace("=", " ").split()
                    try:
                        return float(parts[parts.index("L0") + 1]), float(parts[parts.index("sd") + 1])
                    except (ValueError, IndexError):
                        return None, None
        return None, None

    def stream_check(self, seconds=2.0):
        self.ser.reset_input_buffer()
        t0 = time.time()
        buf = ""
        while time.time() - t0 < seconds:
            buf += self.ser.read(self.ser.in_waiting or 1).decode("ascii", "replace")
        vals = []
        for l in buf.splitlines()[1:-1]:
            try:
                vals.append(float(l.split(",")[-1]))
            except ValueError:
                pass
        return len(vals) / seconds, vals


class PracticeBoard(Board):
    """No hardware: pretends to be the board so the steps can be rehearsed."""

    def __init__(self):
        super().__init__("PRACTICE")
        self.p = {"S": 0.0509, "N": 0.5, "DEAD_counts": 300, "floor_mps": 0.0, "delay_ms": 0}

    def open(self):
        doing("[practice] pretending to open the board's COM port")

    def close(self):
        doing("[practice] pretending to close the board's COM port")

    def params(self):
        return dict(self.p)

    def calibration_text(self):
        return "\n".join(["# --- airspeed calibration (PRACTICE, not a real board) ---"] +
                         [f"# {k} = {v}" for k, v in self.p.items()] + ["# --- end ---"])

    def set(self, key, value):
        name = {"s": "S", "n": "N", "l": "delay_ms", "f": "floor_mps"}[key]
        self.p[name] = float(value)
        time.sleep(0.3)
        return [f"# set {key} {value} (practice)"]

    def zero(self):
        time.sleep(2)
        return 655432.0, 210.0

    def stream_check(self, seconds=2.0):
        time.sleep(seconds)
        return 200.0, [self.p["floor_mps"]] * 400


def find_port():
    ports = [p for p in serial.tools.list_ports.comports() if p.vid == ST_VID]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        say("  !! No Nucleo found (no ST-Link COM port). Plug in its USB cable.")
        return None
    say("  Several ST boards found:")
    for i, p in enumerate(ports):
        say(f"    {i + 1}) {p.device}  {p.description}")
    i = input("  >> which one? ").strip()
    return ports[int(i) - 1].device if i.isdigit() and 0 < int(i) <= len(ports) else None


# ================================================================ run files
def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    head = rows[0]
    return [dict(zip(head, r)) for r in rows[1:]]


def series(path, col="value_raw_mps"):
    t, v = [], []
    for r in read_csv(path):
        try:
            tt, vv = float(r["timestamp_epoch"]), float(r[col])
        except (KeyError, ValueError):
            continue
        if math.isfinite(tt) and math.isfinite(vv):
            t.append(tt)
            v.append(vv)
    return t, v


def newest_runs(n=5):
    files = sorted(glob.glob(os.path.join(LOGS, "*_aligned.csv")), key=os.path.getmtime, reverse=True)
    return files[:n]


def choose_run(after=None):
    runs = newest_runs()
    if not runs:
        say(f"  !! No saved runs in {LOGS}")
        return None
    fresh = [r for r in runs if after is None or os.path.getmtime(r) >= after]
    say("  Saved runs (newest first):")
    for i, r in enumerate(runs):
        tag = "   <- saved just now" if r in fresh and after else ""
        say(f"    {i + 1}) {os.path.basename(r)[:-12]}   "
            f"{time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(r)))}{tag}")
    i = input("  >> which run? [1] ").strip() or "1"
    return runs[int(i) - 1] if i.isdigit() and 0 < int(i) <= len(runs) else None


# ==================================================================== fitting
def interp(x, xs, ys):
    """Linear interpolation of (xs, ys) at x; xs ascending. None outside."""
    import bisect
    if x < xs[0] or x > xs[-1]:
        return None
    i = bisect.bisect_left(xs, x)
    if xs[i] == x:
        return ys[i]
    x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def fit_run(aligned_path, min_lag=0.0):
    """min_lag: most negative lag the board can apply (= -its current delay; it can't output early)."""
    base = aligned_path[: -len("_aligned.csv")]
    tg, g = series(base + "_vdas.csv")
    tu, u = series(base + "_student.csv")
    if len(tg) < 20 or len(tu) < 200:
        say("  !! run too short or empty")
        return None
    dur = tu[-1] - tu[0]
    say(f"  VDAS: {len(tg)} readings ({len(tg) / (tg[-1] - tg[0]):.1f} Hz)   "
        f"your board: {len(tu)} samples ({len(tu) / dur:.1f} Hz over {dur:.0f} s)")
    if len(tu) / dur < 196:
        say("  !! your stream was below 98% of 200 Hz - check the USB cable / port")

    def ours(lag):
        return [interp(t - lag, tu, u) for t in tg]

    def mse(pred):
        e = [(a - b) ** 2 for a, b in zip(g, pred) if b is not None]
        return sum(e) / len(e)

    def scale(x):                       # least squares with the air moving
        num = sum(a * b for a, b in zip(g, x) if b is not None and a > 2 and b > 2)
        den = sum(b * b for a, b in zip(g, x) if b is not None and a > 2 and b > 2)
        return num / den if den else 1.0

    doing(f"searching the lag between VDAS and your stream ({min_lag:+.2f} .. +1.5 s)")
    best = None
    for i in range(int(round((1.5 - min_lag) / 0.025)) + 1):
        lag = min_lag + 0.025 * i
        x = ours(lag)
        k = scale(x)
        m = mse([k * b if b is not None else None for b in x])
        if best is None or m < best[0]:
            best = (m, lag, k)
    _, lag, k = best
    x0 = ours(0.0)
    x = ours(lag)
    rest = [a for a, b in zip(g, x) if b is not None and k * b < 1.2 and a < 2.0]
    F = min(max(sum(rest) / len(rest), 0.0), 2.5) if len(rest) >= 5 else 0.0
    final = [max(k * b, F) if b is not None else None for b in x]
    res = {"lag": lag, "k": k, "floor": F,
           "mse_before": mse(x0), "mse_scale": mse([scale(x0) * b if b is not None else None for b in x0]),
           "mse_lag": best[0], "mse_after": mse(final), "n_rest": len(rest)}

    say("")
    say("  How well your board matched the tunnel in this run (MSE, lower is better):")
    say(f"    as recorded                     {res['mse_before']:.4f} m^2/s^2")
    say(f"    + scale x{k:.4f}                 {res['mse_scale']:.4f}")
    say(f"    + change output delay {lag * 1000:+5.0f} ms   {res['mse_lag']:.4f}")
    say(f"    + floor {F:.2f} m/s at rest         {res['mse_after']:.4f}   <- with the new parameters")
    say("")
    say("  Per speed band (tunnel vs yours after the fit):")
    for lo, hi in ((0, 2), (2, 4), (4, 6), (6, 8), (8, 15)):
        pairs = [(a, b) for a, b in zip(g, final) if b is not None and lo <= a < hi]
        if pairs:
            mg = sum(a for a, _ in pairs) / len(pairs)
            mo = sum(b for _, b in pairs) / len(pairs)
            say(f"    {lo:2d}-{hi:<2d} m/s  {len(pairs):4d} readings  tunnel {mg:5.2f}  yours {mo:5.2f}  "
                f"bias {mo - mg:+.2f}")
    return res


# ===================================================================== steps
def do_zero(board):
    step("ZERO THE LOAD CELL")
    say("  The load cell's resting reading drifts after handling, so it is measured")
    say("  again now. Fan OFF, air still, nobody touching the rig.")
    while True:
        wait_enter("fan OFF and rig untouched?")
        doing("sending 'z' to the board; it averages the load for 2 s")
        L0, sd = board.zero()
        if L0 is None:
            say("  !! no reply from the board. Is the airspeed firmware running, and the app Disconnected?")
            if not ask("try again?"):
                return False
            continue
        quality = "good" if sd < 600 else "NOISY - fan still spinning or rig moving?"
        say(f"  zero = {L0:.0f} counts, noise sd = {sd:.0f} counts  ({quality})")
        if sd < 600 or not ask("redo the zero?"):
            return True


def do_check(board):
    step("CHECK THE BOARD")
    doing("reading the board's parameters ('c')")
    p = board.params()
    if "S" not in p:
        say("  !! the board didn't answer. Flash the airspeed firmware:  pio run -e airspeed -t upload")
        return False
    say(f"  S = {p['S']:.6f}   N = {p.get('N', 0.5)}   floor = {p.get('floor_mps', 0):.2f} m/s   "
        f"delay = {p.get('delay_ms', 0):.0f} ms")
    doing("listening to the stream for 2 s")
    rate, vals = board.stream_check()
    if vals:
        say(f"  rate {rate:.0f} lines/s (need 200)   last reading {vals[-1]:.2f} m/s")
    else:
        say("  !! no speed lines received")
    return True


def do_record(practice):
    step("RECORD A CALIBRATION RUN IN WINDTUNNELLOGGER")
    if practice:
        say("  PRACTICE: the app runs with --demo (simulated tunnel AND simulated board).")
        say("  Its tunnel follows its own speed profile; just follow the clicks.")
    say("  In the app:")
    say("    1. Ground truth: Wind tunnel" + (" (demo: simulated)" if practice else "") +
        " - wait for 'tunnel offset' (fan OFF)")
    say("    2. Enter today's air pressure, air temperature, team name")
    say("    3. Your board: pick its COM port, press CONNECT; check the rate shows ~200 Hz")
    say("    4. Press START LOGGING, then press Enter here at the same moment")
    t_start = time.time()
    wait_enter("START LOGGING pressed?")
    t_start = time.time()
    say("  Follow the cues (they appear at the right time):")
    for t_cue, what in PROTOCOL:
        while time.time() - t_start < t_cue:
            left = t_cue - (time.time() - t_start)
            print(f"\r    next in {left:5.1f} s: {what:<45}", end="", flush=True)
            time.sleep(0.2)
        print(f"\r    [{t_cue:3d} s] NOW: {what:<55}", flush=True)
    say("  When saved: press DISCONNECT (Your board) in the app so this script can use the port.")
    wait_enter("saved and disconnected?")
    return t_start


def do_fit_apply(board, after=None):
    step("FIT THE PARAMETERS FROM THE SAVED RUN")
    doing(f"looking for runs in {LOGS}")
    run = choose_run(after)
    if not run:
        return False
    board.open()
    doing("reading the board's current parameters (the run was recorded with these)")
    cur = board.params()
    if "S" not in cur:
        say("  !! board didn't answer 'c' - is the app still connected to it?")
        board.close()
        return False
    doing(f"reading {os.path.basename(run)[:-12]}_vdas.csv and _student.csv")
    res = fit_run(run, min_lag=-cur.get("delay_ms", 0) / 1000)
    if not res:
        board.close()
        return False
    if res["n_rest"] < 5:
        say("  (not enough fan-off readings to set the floor; keeping the current one)")
    S1 = res["k"] * cur["S"]
    D1 = max(0.0, cur.get("delay_ms", 0) + res["lag"] * 1000)
    F1 = res["floor"] if res["n_rest"] >= 5 else cur.get("floor_mps", 0)
    say("")
    say("  New parameters:")
    say(f"    S     {cur['S']:.6f} -> {S1:.6f}   (scale x{res['k']:.4f})")
    say(f"    delay {cur.get('delay_ms', 0):6.0f} -> {D1:6.0f} ms")
    say(f"    floor {cur.get('floor_mps', 0):6.2f} -> {F1:6.2f} m/s")
    if abs(res["k"] - 1) > 0.5:
        say("  !! the scale changes by more than 50% - check the right run / tunnel was used")
    if ask("send these to the board (saved in its flash)?"):
        for key, val in (("s", f"{S1:.6f}"), ("l", f"{D1:.0f}"), ("f", f"{F1:.3f}")):
            doing(f"sending '{key} {val}' (board saves to flash, ~2 s pause in its output)")
            for l in board.set(key, val):
                say("      board: " + l)
        say("  Board calibration now:")
        for l in board.calibration_text().splitlines():
            say("    " + l)
    board.close()
    return True


def do_calfile(board):
    step("CALIBRATION FILE")
    board.open()
    txt = board.calibration_text()
    board.close()
    if not txt:
        say("  !! no reply")
        return
    os.makedirs(os.path.dirname(LOGS), exist_ok=True)
    path = os.path.join(os.path.dirname(LOGS), time.strftime("calibration_%Y%m%d_%H%M%S.txt"))
    with open(path, "w") as f:
        f.write(txt + "\n")
    say(txt)
    say(f"  saved to {path}")


def launch_app(practice):
    exe = next((os.path.abspath(p) for p in EXE_CANDIDATES if os.path.exists(p)), None)
    if not exe:
        say("  (WindTunnelLogger.exe not found next to the repo or in Downloads: open it yourself)")
        return
    if ask(f"open WindTunnelLogger{' in --demo mode' if practice else ''} now?"):
        subprocess.Popen([exe] + (["--demo"] if practice else []))
        doing("app starting (takes a few seconds)")


def final_instructions():
    step("READY FOR THE ASSESSMENT")
    say("  1. In WindTunnelLogger: CONNECT your board; with the fan off it should read the floor value")
    say("     (about the same as the tunnel's own reading at rest).")
    say("  2. START LOGGING, tell the professor to start.")
    say("  3. After the run: STOP & SAVE. For the calibration file, Disconnect and choose")
    say("     'Calibration file' in this script (or it is in the last step's printout).")


# ====================================================================== main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--practice", action="store_true", help="no board: rehearse with the app's --demo mode")
    ap.add_argument("--port", help="board COM port (default: find the ST-Link automatically)")
    a = ap.parse_args()

    step("AIRSPEED DEMO-DAY HELPER" + ("  (PRACTICE - no board)" if a.practice else ""))
    say("  This script zeroes and calibrates the board; WindTunnelLogger records the runs.")
    say(f"  Runs are read from: {LOGS}")

    if a.practice:
        board = PracticeBoard()
    else:
        port = a.port or find_port()
        if not port:
            sys.exit(1)
        say(f"  Board on {port}")
        board = Board(port)

    while True:
        step("MENU")
        say("  1) Full calibration   zero -> record run in app -> fit -> send -> re-zero")
        say("  2) Zero only          (fan OFF; do this right before the assessment)")
        say("  3) Fit a saved run    (pick any run in the logs folder, send the result)")
        say("  4) Check board        parameters + stream rate")
        say("  5) Calibration file   read the board's calibration, save it to a file")
        say("  6) Open WindTunnelLogger")
        say("  q) Quit")
        c = input("  >> choice: ").strip().lower()
        if c == "1":
            launch_app(a.practice)
            board.open()
            ok = do_check(board) and do_zero(board)
            board.close()
            if not ok:
                continue
            t0 = do_record(a.practice)
            if do_fit_apply(board, after=t0):
                step("RE-ZERO BEFORE THE ASSESSMENT")
                say("  Wait until the air in the tunnel is still (~20 s after fan off).")
                board.open()
                do_zero(board)
                board.close()
                final_instructions()
        elif c == "2":
            board.open()
            do_zero(board)
            board.close()
            say("  Now CONNECT the board in WindTunnelLogger.")
        elif c == "3":
            do_fit_apply(board)
        elif c == "4":
            board.open()
            do_check(board)
            board.close()
        elif c == "5":
            do_calfile(board)
        elif c == "6":
            launch_app(a.practice)
        elif c in ("q", "quit", "exit"):
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        say("\n  stopped")
