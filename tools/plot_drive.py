#!/usr/bin/env python3
"""Plot a drive recording made by teleop (JSON Lines, format in include/datalog/drive_log.hpp).

Left: odometry path with the scan points of every 5th scan projected from the odometry pose.
Right: the last scan in the robot frame (valid points and weak points separately).

Odometry drifts, so walls in the left plot get thicker the longer the drive. That smear is
exactly what scan matching will remove later.

Usage: plot_drive.py drive.jsonl [-o plot.png] [--every N]
"""
import argparse
import json
import math
import sys


def load(path):
    header, scans, odom, events = None, [], [], []
    with open(path, encoding="utf-8") as f:
        for n, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as e:
                sys.exit(f"{path}:{n}: invalid JSON ({e})")
            kind = rec.get("type")
            if kind == "header":
                header = rec
            elif kind == "scan":
                scans.append(rec)
            elif kind == "odom":
                odom.append(rec)
            elif kind == "event":
                events.append(rec)
    if header is None:
        sys.exit(f"{path}: no header line")
    return header, scans, odom, events


def integrate(odom, wheelbase):
    """Differential-drive odometry, same equations as core::Odometry."""
    x = y = th = 0.0
    poses = []
    prev = None
    for o in odom:
        if prev is not None:
            dl = (o["l"] - prev["l"]) / 1000.0
            dr = (o["r"] - prev["r"]) / 1000.0
            ds = 0.5 * (dl + dr)
            dth = (dr - dl) / wheelbase
            x += ds * math.cos(th + 0.5 * dth)
            y += ds * math.sin(th + 0.5 * dth)
            th += dth
        poses.append((o["t"], x, y, th))
        prev = o
    return poses


def pose_at(poses, t):
    """Pose with the closest timestamp at or before t (poses are sorted by time)."""
    lo, hi = 0, len(poses) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return poses[lo]


def scan_points(scan, flag):
    pts = []
    for deg, (r, f) in enumerate(zip(scan["r"], scan["f"])):
        if f == flag and r > 0:
            a = math.radians(deg)
            pts.append((r / 1000.0 * math.cos(a), r / 1000.0 * math.sin(a)))
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("-o", "--output", help="write PNG instead of opening a window")
    ap.add_argument("--every", type=int, default=5, help="use every Nth scan in the map plot")
    args = ap.parse_args()

    import matplotlib
    if args.output:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    header, scans, odom, events = load(args.log)
    print(f"{args.log}: {header['source']}, {len(scans)} scans, {len(odom)} odom, {len(events)} events")
    if not odom:
        sys.exit("no odometry in the recording")

    wheelbase = header.get("wheelbase_m", 0.248)
    lds_offset = header.get("lds_offset_m", -0.09)
    poses = integrate(odom, wheelbase)

    fig, (ax_map, ax_scan) = plt.subplots(1, 2, figsize=(13, 6))

    # Scan points in the odometry frame
    mx, my = [], []
    for s in scans[:: max(1, args.every)]:
        _, x, y, th = pose_at(poses, s["t"])
        lx, ly = x + lds_offset * math.cos(th), y + lds_offset * math.sin(th)
        c, sn = math.cos(th), math.sin(th)
        for px, py in scan_points(s, 0):
            mx.append(lx + c * px - sn * py)
            my.append(ly + sn * px + c * py)
    ax_map.scatter(mx, my, s=1, c="0.45", label="LDS points (valid)")
    ax_map.plot([p[1] for p in poses], [p[2] for p in poses], "-", c="tab:red", lw=1.5, label="odometry")
    ax_map.plot(poses[0][1], poses[0][2], "o", c="tab:green", label="start")
    for e in events:
        if e["msg"] == "bumper":
            _, x, y, _ = pose_at(poses, e["t"])
            ax_map.plot(x, y, "x", c="tab:orange", ms=9)
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))
    ax_map.set_title(f"Odometry frame, {dist:.2f} m driven, {poses[-1][0]:.1f} s")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.set_aspect("equal")
    ax_map.grid(alpha=0.3)
    ax_map.legend(loc="best", fontsize=8)

    # Last scan in the sensor frame
    if scans:
        last = scans[-1]
        for flag, colour, name in ((0, "tab:blue", "valid"), (1, "tab:orange", "weak")):
            pts = scan_points(last, flag)
            ax_scan.scatter([p[0] for p in pts], [p[1] for p in pts], s=4, c=colour, label=f"{name} ({len(pts)})")
        missing = sum(1 for f in last["f"] if f == 2)
        ax_scan.plot(0, 0, "^", c="k", ms=8)
        ax_scan.set_title(f"Last scan, t = {last['t']:.1f} s, {missing} beams without return")
        ax_scan.legend(loc="best", fontsize=8)
    ax_scan.set_xlabel("x forward [m]")
    ax_scan.set_ylabel("y left [m] (if LDS is counter-clockwise)")
    ax_scan.set_aspect("equal")
    ax_scan.grid(alpha=0.3)

    fig.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=120)
        print(f"saved {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
