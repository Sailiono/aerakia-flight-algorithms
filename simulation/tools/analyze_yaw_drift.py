#!/usr/bin/env python3
"""Analyze yaw drift from a text capture CSV.

Input CSV format:
- header: ts_sec,line
- `line` may include `[FUSION] ... yaw=...` and optionally `CALIBRATION COMPLETE` marker.

Outputs:
- number of samples
- detected calibration timestamp (if any)
- linear drift slope in deg/s and deg/min

This script is intentionally dependency-free (no numpy/pandas).
"""

from __future__ import annotations

import argparse
import csv
import re
from statistics import mean


_FUSION_RE = re.compile(r"\[FUSION\] roll=([\d.\-]+) pitch=([\d.\-]+) yaw=([\d.\-]+)")


def _unwrap_degrees(samples_deg: list[float]) -> list[float]:
    if not samples_deg:
        return []
    out = [samples_deg[0]]
    for y in samples_deg[1:]:
        prev = out[-1]
        dy = y - prev
        while dy > 180:
            dy -= 360
        while dy < -180:
            dy += 360
        out.append(prev + dy)
    return out


def _linear_slope(xs: list[float], ys: list[float]) -> float:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("need >=2 points")
    mx = mean(xs)
    my = mean(ys)
    ssx = sum((x - mx) ** 2 for x in xs)
    if ssx == 0:
        raise ValueError("ssx=0")
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / ssx


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path", help="path to a text capture CSV")
    ap.add_argument(
        "--start",
        choices=["calib", "first_fusion"],
        default="calib",
        help="Start time reference for drift fit",
    )
    args = ap.parse_args()

    rows_t: list[float] = []
    rows_yaw: list[float] = []
    calib_t: float | None = None
    first_fusion_t: float | None = None

    with open(args.csv_path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            t = float(row["ts_sec"])
            line = row["line"]

            if calib_t is None and "CALIBRATION COMPLETE" in line:
                calib_t = t

            m = _FUSION_RE.search(line)
            if not m:
                continue

            if first_fusion_t is None:
                first_fusion_t = t

            yaw = float(m.group(3))
            rows_t.append(t)
            rows_yaw.append(yaw)

    print(f"fusion_rows={len(rows_t)}")
    print(f"first_fusion_t={first_fusion_t}")
    print(f"calib_t={calib_t}")

    if not rows_t:
        print("ERROR: no [FUSION] rows found")
        return 2

    y_unwrap = _unwrap_degrees(rows_yaw)

    if args.start == "calib" and calib_t is not None:
        start_t = calib_t
    else:
        start_t = first_fusion_t if first_fusion_t is not None else rows_t[0]

    xs: list[float] = []
    ys: list[float] = []
    for t, yu in zip(rows_t, y_unwrap):
        if t >= start_t:
            xs.append(t - start_t)
            ys.append(yu)

    if len(xs) < 2:
        print("ERROR: not enough samples after start_t")
        return 3

    slope = _linear_slope(xs, ys)
    print(f"fit_points={len(xs)} duration={xs[-1]:.3f}s")
    print(f"yaw_drift_slope={slope:.6f} deg/s  ({slope*60:.3f} deg/min)")
    print(f"yaw_start={ys[0]:.3f} deg")
    print(f"yaw_end  ={ys[-1]:.3f} deg")
    print(f"delta    ={ys[-1]-ys[0]:.3f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
