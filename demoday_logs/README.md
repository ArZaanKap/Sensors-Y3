# Demo-day logs (5 Nov 2026, white TecQuipment tunnel)

Most of the report plots come from the two good-tunnel runs below.

| Folder | What it is |
|---|---|
| `calibration_run/` | Calibration run at 09:22: the practice run we fitted S and the delay on (97 s, the knob stepped through speeds) |
| `team2/` | The assessment, run 1 at 12:23 (127 s, MSE 0.169). Saved by the TA's logger. Run 2 never happened. |
| `bench_rehearsal/` | Bench rehearsal at 09:10 with simulated ground truth, plus a failed raw dump (sensor not found). Not tunnel data. |

## Why each run has several files

| File | Made by | Contents |
|---|---|---|
| `*_vdas.csv` | Wind Tunnel Logger app | Tunnel ground truth, 4 Hz: speed, whole-Pa `pa_raw`, `pa_corrected` (= pa_raw − session offset) |
| `*_student.csv` | Wind Tunnel Logger app | Our board's speed stream, 200 Hz |
| `*_aligned.csv` | Wind Tunnel Logger app | One row per tunnel reading, paired with our nearest sample (≤ 150 ms). The scoring file. |
| `raw_*.csv` | `demo_day.py` (board 'D' dump) | Board RAM recorder, 100 Hz: raw load counts, rotor angle, mic envelope. Calibration run only; unplugging wiped the assessment one. |
| `*_merged.csv` | `tools/merge_run.py` | raw + student + vdas on one 100 Hz timeline (force, rpm, mic, our speed, tunnel speed) |
| `*_fit.txt` | `demo_day.py fit` | Parameters before and after the fit |
| `calibration_*.txt` | `demo_day.py cal` | Board settings at that moment. The `team2/` one is after switching to the run-2 settings (N 0.53, S 0.026294). |

Board settings during the runs: the calibration run used S 0.0509, delay 0 and floor 0 (the fit then set S 0.036196, delay 350, floor 1.003). The assessment run used S 0.036196, N ½, DEAD 300, floor 1.003, delay 350.
Tunnel offsets: 1.38 Pa (calibration session), 0.14 Pa (assessment session).
