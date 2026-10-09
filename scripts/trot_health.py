#!/usr/bin/env python3
"""Quantify how a trot destabilises, from a controller log's TROTHEALTH lines.

The trot can stay up for tens of seconds and then diverge, inconsistently from
run to run. That pattern means a quantity is accumulating rather than a single
event firing, so this finds the divergence onset and reports which term started
moving FIRST -- i.e. which one leads, not merely which one is largest once the
robot is already falling.

Usage:  trot_health.py <log> [<log> ...]
"""
import re
import sys

PAT = re.compile(
    r"TROTHEALTH t=([\d.]+) \| roll ([-+\d.]+) pitch ([-+\d.]+) z ([\d.]+) \| "
    r"\|v_xy\| ([\d.]+) yaw ([-+\d.]+) \| fz_sum mean ([\d.-]+) min ([\d.-]+) \| "
    r"airborne (\d+)% \| swing_perr ([\d.]+) \| tau_pk ([\d.]+)"
)
COLS = ["roll", "pitch", "z", "v_xy", "yaw", "fz_mean", "fz_min",
        "airborne", "perr", "tau_pk"]
# Divergence is declared on attitude, since that is the axis that actually fails.
FAIL_ROLL = 0.20   # rad


def parse(path):
    rows = []
    for line in open(path):
        m = PAT.search(line)
        if m:
            t = float(m.group(1))
            rows.append((t, [float(m.group(i)) for i in range(2, 12)]))
    return rows


def report(path):
    rows = parse(path)
    if not rows:
        print(f"{path}: no TROTHEALTH lines")
        return
    print(f"\n=== {path}  ({len(rows)} s of gait) ===")

    # Onset: first sample whose |roll| exceeds the threshold and never recovers.
    # Onset = first sample past the threshold that is followed by sustained
    # failure. Requiring EVERY later sample to stay large was wrong: a tumbling
    # robot swings back through small |roll|, so one such sample reported the whole
    # run "stable". Use the fraction of later samples instead, and treat a high
    # airborne fraction as failure too, since a robot off the ground has failed
    # regardless of its instantaneous attitude.
    onset = None
    for i, (t, v) in enumerate(rows):
        tail = rows[i:]
        bad = sum(1 for _, w in tail if abs(w[0]) > FAIL_ROLL / 2 or w[7] > 25.0)
        if (abs(v[0]) > FAIL_ROLL or v[7] > 25.0) and bad >= 0.7 * len(tail):
            onset = i
            break

    if onset is None:
        print(f"  stable for the whole log; final |roll| {abs(rows[-1][1][0]):.4f}")
        base = rows[: max(1, len(rows) // 2)]
    else:
        print(f"  divergence onset at t = {rows[onset][0]:.1f} s "
              f"(|roll| > {FAIL_ROLL})")
        base = rows[:onset]

    if len(base) < 3:
        print("  too short to trend")
        return

    # Per-column linear trend over the pre-onset window: the slope says which
    # quantity was already drifting before anything visibly failed.
    n = len(base)
    tm = sum(t for t, _ in base) / n
    print(f"  pre-onset window: {base[0][0]:.1f}-{base[-1][0]:.1f} s  (n={n})")
    print(f"  {'term':9} {'start':>10} {'end':>10} {'slope/s':>11}")
    trends = []
    for c, name in enumerate(COLS):
        ys = [v[c] for _, v in base]
        ym = sum(ys) / n
        num = sum((t - tm) * (y - ym) for (t, _), y in zip(base, ys))
        den = sum((t - tm) ** 2 for t, _ in base)
        slope = num / den if den else 0.0
        trends.append((abs(slope), name, ys[0], ys[-1], slope))
        print(f"  {name:9} {ys[0]:10.4f} {ys[-1]:10.4f} {slope:11.5f}")

    # Rank by how much each term moved relative to its own starting magnitude,
    # so a small absolute drift in a small quantity still registers.
    print("\n  leading drift (slope normalised by |start| + eps):")
    ranked = sorted(trends, key=lambda r: -r[0] / (abs(r[2]) + 1e-3))
    for mag, name, y0, y1, slope in ranked[:4]:
        print(f"    {name:9} {slope:+.5f}/s  ({y0:.4f} -> {y1:.4f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        report(p)
