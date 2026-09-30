# /// script
# dependencies = ["numpy", "openpyxl"]
# ///
"""Per-state currents from a PPK2 CSV export of tools/power_profile, and the TX burst.

  uv run tools/power_analysis.py <csv> [--start S] [--xlsx sheet.xlsx --col "V2 KIM1"]

The sketch holds each state ~33 s (announce + 30 s). The state boundaries are found
from the trace itself: state 1 (deep sleep) is the only stretch at a few mA or less
lasting > 20 s right after a wake-up spike; from its start, states follow every
STEP seconds. Each state is averaged over its steady part (announce and module boot
skipped). The TX burst is the stretch in state 7 above the state's median + 100 mA.
"""
import argparse, json, sys
import numpy as np

NAMES = ["deep sleep", "light sleep", "CPU awake", "CPU + SD", "CPU + GPS", "CPU + sat module idle",
         "CPU + sat module + 1 TX", "CPU + WiFi"]
# steady window inside each state, seconds from the state start (announce is ~3 s,
# the satellite module boots for 1 s more in states 6-7)
WIN = [(4, 31), (4, 31), (4, 31), (5, 31), (4, 31), (5, 31), (5, 31), (4, 31)]

def load(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d[:, 0] / 1000.0, d[:, 1] / 1000.0          # s, mA

def per_second(t, i):
    n = int(t[-1]); return np.array([i[(t >= s) & (t < s + 1)].mean() for s in range(n)])

def find_deep_sleep_starts(ps):
    """Seconds where a >= 20 s stretch below 3 mA begins."""
    starts, run = [], 0
    for s, v in enumerate(ps):
        run = run + 1 if v < 3 else 0
        if run == 20: starts.append(s - 19)
    return starts

def tenths(t, i):
    """100 ms means: index k covers [k/10, (k+1)/10) s."""
    n = int(t[-1] * 10); out = np.zeros(n)
    idx = (t * 10).astype(int); np.add.at(out, np.clip(idx, 0, n - 1), i)
    cnt = np.bincount(np.clip(idx, 0, n - 1), minlength=n); return out / np.maximum(cnt, 1)

def segment(t, i, deep_start):
    """State windows [from, to) in s, from the changes the trace does show:
    the deep sleep (<3 mA), the light sleep (<30 mA), the GPS switching on and off,
    and the fixed durations of the sketch where two states look alike (3/4, 6/7)."""
    tn = tenths(t, i); sec = lambda k: k / 10
    k = int(deep_start * 10)
    while k < len(tn) and tn[k] >= 3: k += 1          # deep sleep onset
    d_on = k
    while k < len(tn) and tn[k] < 30: k += 1          # wake: state 2 announce
    s2 = k
    k += 20
    while k < len(tn) and tn[k] >= 30: k += 1          # light sleep onset
    while k < len(tn) and tn[k] < 30: k += 1          # wake from light sleep = state 3 start
    s3 = k
    base = float(np.median(tn[s3 + 30:s3 + 300]))
    s4 = s3 + 322                                      # 2.2 s announce + 30 s
    k = s4 + 150
    while k < len(tn) and np.mean(tn[k:k + 10]) < base + 80: k += 1   # GPS on
    s5 = k - 30                                        # the GPS comes on after the 3.0 s announce
    gps = float(np.median(tn[k + 20:k + 200]))
    k += 250
    while k < len(tn) and np.mean(tn[k:k + 10]) > gps - 50: k += 1    # GPS off = state 6 start
    s6 = k
    s7 = s6 + 334
    s8 = s7 + 338
    k = s8 + 300
    while k < len(tn) and tn[k] >= 3: k += 1          # next deep sleep onset (state 1 again)
    end8 = k - 14                                      # its 1.4 s announce
    return {1: (sec(d_on) + 0.5, sec(s2) - 0.2), 2: (sec(s2) + 2.2, sec(s3) - 0.2), 3: (sec(s3) + 2.6, sec(s4) - 0.2),
            4: (sec(s4) + 6.0, sec(s5) - 0.2), 5: (sec(s5) + 4.0, sec(s6) - 0.2), 6: (sec(s6) + 5.0, sec(s7) - 0.2),
            7: (sec(s7) + 4.0, sec(s8) - 0.2), 8: (sec(s8) + 4.6, sec(end8) - 0.2)}

def analyse(t, i, start, step):
    win = segment(t, i, start)
    out = []
    for k in range(8):
        a, b = win[k + 1]
        m = (t >= a) & (t < b)
        x = i[m]
        out.append({"state": k + 1, "name": NAMES[k], "from_s": round(a, 1), "to_s": round(b, 1),
                    "mean_mA": round(float(x.mean()), 3) if len(x) else None,
                    "max_mA": round(float(x.max()), 1) if len(x) else None})
    # TX burst inside state 7
    a, b = win[7]
    m = (t >= a - 4) & (t < b); tt, xx = t[m], i[m]
    base = float(np.median(xx)); hot = xx > base + 100
    burst = None
    if hot.any():
        idx = np.where(hot)[0]
        # longest contiguous run
        runs = np.split(idx, np.where(np.diff(idx) > 50)[0] + 1)
        r = max(runs, key=len)
        bt, bx = tt[r[0]:r[-1] + 1], xx[r[0]:r[-1] + 1]
        dt = float(np.median(np.diff(tt)))
        burst = {"at_s": round(float(bt[0]), 2), "duration_ms": round((bt[-1] - bt[0] + dt) * 1000),
                 "mean_mA": round(float(bx.mean()), 1), "max_mA": round(float(bx.max()), 1),
                 "charge_mC": round(float(bx.sum() * dt), 1), "baseline_mA": round(base, 1),
                 "extra_charge_mC": round(float((bx - base).sum() * dt), 1)}
    return out, burst

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--start", type=float)
    ap.add_argument("--step", type=float, default=33.1); ap.add_argument("--json")
    a = ap.parse_args()
    t, i = load(a.csv); ps = per_second(t, i)
    starts = find_deep_sleep_starts(ps)
    start = a.start if a.start is not None else (starts[0] - 1 if starts else 0)
    print(f"deep-sleep stretches start at {starts} s; using state 1 start = {start} s")
    states, burst = analyse(t, i, start, a.step)
    for s in states: print(f"  {s['state']} {s['name']:<24} {s['from_s']:6.1f}-{s['to_s']:6.1f} s  mean {s['mean_mA']:9.3f} mA  max {s['max_mA']:7.1f}")
    print("  TX burst:", burst)
    if a.json: json.dump({"states": states, "burst": burst, "deep_sleep_starts": starts}, open(a.json, "w"), indent=1)
