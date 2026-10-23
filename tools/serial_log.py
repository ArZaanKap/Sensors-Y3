"""Log the Nucleo's serial output to a CSV file for later analysis.

Run with the PlatformIO Python (it already has pyserial):
    ~/.platformio/penv/Scripts/python tools/serial_log.py               # summary, until Ctrl+C
    ~/.platformio/penv/Scripts/python tools/serial_log.py -m r -t 20    # raw samples for 20 s

Lines starting with '#' are firmware notes; they are kept in the file so a
log records its own settings. Load with e.g. pandas.read_csv(path, comment='#').
"""
import argparse
import datetime as dt
import pathlib
import sys
import time

import serial
import serial.tools.list_ports

ST_VID = 0x0483  # STMicroelectronics (ST-Link virtual COM port)


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == ST_VID:
            return p.device
    sys.exit("No ST-Link serial port found -- is the Nucleo plugged in? (or pass --port)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="serial port (default: auto-detect ST-Link)")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-m", "--mode", choices="srq", default="s",
                    help="s=summary (10 Hz), r=raw every sample, q=quiet")
    ap.add_argument("-t", "--seconds", type=float, help="stop after this many seconds")
    ap.add_argument("-o", "--out", help="output file (default: logs/<name>_<timestamp>.csv)")
    ap.add_argument("-n", "--name", default="as5048b", help="prefix for the default file name")
    ap.add_argument("--no-zero", action="store_true", help="don't zero the rev counter at start")
    args = ap.parse_args()

    port = args.port or find_port()
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = pathlib.Path(args.out or pathlib.Path(__file__).resolve().parent.parent / "logs"
                       / f"{args.name}_{stamp}_{args.mode}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(port, args.baud, timeout=0.2) as ser, open(out, "w", newline="") as f:
        ser.reset_input_buffer()
        if not args.no_zero:
            ser.write(b"z")
        ser.write(args.mode.encode())   # firmware answers with a status + header line
        f.write(f"# logged {stamp} from {port} @ {args.baud} baud, mode={args.mode}\n")
        print(f"logging {port} -> {out}  (Ctrl+C to stop)")

        t0 = time.time()
        buf = b""
        lines = 0
        try:
            while args.seconds is None or time.time() - t0 < args.seconds:
                buf += ser.read(ser.in_waiting or 1)
                *done, buf = buf.split(b"\n")
                for raw in done:
                    line = raw.decode(errors="replace").rstrip("\r")
                    f.write(line + "\n")
                    lines += 1
                    # Echo everything except the raw firehose (show every 100th).
                    if args.mode != "r" or line.startswith("#") or lines % 100 == 0:
                        print(line)
        except KeyboardInterrupt:
            pass
        ser.write(b"s")   # leave the board in summary mode for the next viewer

    print(f"\n{lines} lines in {time.time() - t0:.1f} s -> {out}")


if __name__ == "__main__":
    main()
