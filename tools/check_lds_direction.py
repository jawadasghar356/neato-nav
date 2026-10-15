#!/usr/bin/env python3
"""Check the LDS angle direction from a drive recording, without any extra test setup.

Idea: when the robot turns on the spot by an angle d (counter-clockwise positive, from the
wheel encoders), every wall seen by the LDS moves by -d in the robot frame. If the scan
angles are counted counter-clockwise, a scan taken after the turn equals the scan before it
shifted by -d degrees. If they are counted clockwise, the shift is +d.

The script finds pairs of scans taken while the robot stood still, with a pure rotation in
between, and searches the circular shift that makes the two scans match best.

Usage: check_lds_direction.py drive.jsonl[.gz]
"""
import bisect
import gzip
import json
import math
import sys


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    opener = gzip.open if sys.argv[1].endswith(".gz") else open
    with opener(sys.argv[1], "rt", encoding="utf-8") as f:
        recs = [json.loads(line) for line in f if line.strip()]
    header = recs[0]
    wheelbase = header.get("wheelbase_m", 0.248)
    odom = [r for r in recs if r.get("type") == "odom"]
    scans = [r for r in recs if r.get("type") == "scan"]
    times = [o["t"] for o in odom]

    def odom_at(t):
        return odom[max(0, bisect.bisect_right(times, t) - 1)]

    def standing(t, window=0.5):
        a, b = odom_at(t - window), odom_at(t)
        return a["l"] == b["l"] and a["r"] == b["r"]

    still = [s for s in scans if standing(s["t"])]
    rows = []
    for a, b in zip(still, still[1:]):
        oa, ob = odom_at(a["t"]), odom_at(b["t"])
        dl, dr = ob["l"] - oa["l"], ob["r"] - oa["r"]
        if abs(dl + dr) > 10 or abs(dr - dl) < 40:  # want a pure rotation of more than ~9 degrees
            continue
        turn = math.degrees((dr - dl) / 1000.0 / wheelbase)

        best = None
        for shift in range(-180, 180):
            err, n = 0.0, 0
            for k in range(360):
                j = (k + shift) % 360
                if a["f"][j] == 0 and b["f"][k] == 0:
                    err += min(abs(a["r"][j] - b["r"][k]), 300)
                    n += 1
            if n > 150 and (best is None or err / n < best[0]):
                best = (err / n, shift)
        if best:
            # b[k] = a[k + shift]: a wall at angle x before the turn is at x - shift afterwards
            rows.append((a["t"], turn, -best[1], best[0]))

    if not rows:
        sys.exit("no on-the-spot turns between two standing scans found")

    print(" time [s]  turn from encoders  wall shift in scan  mean residual")
    for t, turn, shift, res in rows:
        print(f"{t:8.1f}  {turn:+15.1f} deg  {shift:+15d} deg  {res:9.0f} mm")
    # A half turn looks the same in both directions, so only smaller turns decide
    usable = [(turn, shift) for _, turn, shift, _ in rows if abs(turn) < 150]
    if not usable:
        sys.exit("only half turns found, the direction cannot be decided from those")
    ccw = sum(1 for turn, shift in usable if turn * shift < 0)
    print(f"\n{ccw} of {len(usable)} turns below 150 deg: walls move opposite to the turn -> ", end="")
    print("angles are counter-clockwise" if ccw > len(usable) / 2 else "angles are CLOCKWISE")


if __name__ == "__main__":
    main()
