#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compare multiple recorded or synthetic runs using golden_imu.csv.

Usage:
  python compare_golden_runs.py artifacts/run_*/golden_imu.csv

Outputs a small text table with:
- dt mean/std/p99
- yaw drift (deg/min) on a default window
- latency estimate (ms)
- init yaw (deg) at a given timestamp
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class Summary:
    name: str
    dt_mean_ms: float
    dt_std_ms: float
    dt_p99_ms: float
    yaw_drift_deg_min: float
    yaw_init_deg: float
    lag_ms: float


def _load_cols(path: str):
    cols = {}
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                cols.setdefault(k, []).append(int(v) if (v or "").strip() != "" else 0)
    return {k: np.asarray(v, dtype=np.int64) for k, v in cols.items()}


def _time_s(ts_us: np.ndarray) -> np.ndarray:
    return (ts_us - ts_us[0]).astype(np.float64) * 1e-6


def _yaw_drift_deg_min(t_s: np.ndarray, yaw_deg: np.ndarray, t0: float, t1: float) -> float:
    mask = (t_s >= t0) & (t_s <= t1)
    if mask.sum() < 10:
        return 0.0
    a, _b = np.polyfit(t_s[mask], yaw_deg[mask], 1)
    return float(a) * 60.0


def _estimate_latency_ms(t_s: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    def hp(x: np.ndarray, win: int = 50) -> np.ndarray:
        if len(x) < win * 2:
            return x - np.mean(x)
        kernel = np.ones(win, dtype=np.float64) / float(win)
        lp = np.convolve(x, kernel, mode="same")
        return x - lp

    x = hp(a)
    y = hp(b)
    x -= np.mean(x)
    y -= np.mean(y)
    sx = np.std(x)
    sy = np.std(y)
    if sx < 1e-6 or sy < 1e-6:
        return 0.0
    x /= sx
    y /= sy

    max_lag = min(400, len(x) // 4)
    corr = np.correlate(x, y, mode="full")
    mid = len(corr) // 2
    window = corr[mid - max_lag : mid + max_lag + 1]
    best = int(np.argmax(window)) - max_lag

    dt_mean_s = float(np.mean(np.diff(t_s))) if len(t_s) > 1 else 0.0
    return best * dt_mean_s * 1000.0


def summarize(path: str, yaw_window: Tuple[float, float], yaw_init_t: float) -> Summary:
    cols = _load_cols(path)
    ts_us = cols["ts_us"]
    t_s = _time_s(ts_us)

    dt_ms = cols["dt_us"].astype(np.float64) / 1000.0
    yaw_deg = cols["yaw_mdeg"].astype(np.float64) / 1000.0

    acc_z = cols["raw_acc_mg_z"].astype(np.float64) / 1000.0
    g_est_z = cols["g_est_mg_z"].astype(np.float64) / 1000.0

    drift = _yaw_drift_deg_min(t_s, yaw_deg, yaw_window[0], yaw_window[1])
    lag_ms = _estimate_latency_ms(t_s, acc_z, g_est_z)

    # init yaw: average around a timestamp to reduce noise
    mask = (t_s >= yaw_init_t) & (t_s <= yaw_init_t + 0.5)
    yaw_init = float(np.mean(yaw_deg[mask])) if mask.sum() > 0 else float(yaw_deg[0])

    name = os.path.basename(os.path.dirname(path))
    return Summary(
        name=name,
        dt_mean_ms=float(dt_ms.mean()),
        dt_std_ms=float(dt_ms.std()),
        dt_p99_ms=float(np.percentile(dt_ms, 99)),
        yaw_drift_deg_min=float(drift),
        yaw_init_deg=float(yaw_init),
        lag_ms=float(lag_ms),
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+", help="golden_imu.csv paths or globs")
    ap.add_argument("--yaw-window", default="2,30")
    ap.add_argument("--yaw-init-t", type=float, default=2.0)
    args = ap.parse_args()

    paths: List[str] = []
    for p in args.paths:
        if any(ch in p for ch in "*?["):
            paths.extend(glob.glob(p))
        else:
            paths.append(p)

    paths = [os.path.abspath(p) for p in paths]
    paths = [p for p in paths if os.path.exists(p)]
    if not paths:
        raise SystemExit("No valid golden_imu.csv paths found")

    t0, t1 = [float(x.strip()) for x in args.yaw_window.split(",")]

    sums = [summarize(p, (t0, t1), args.yaw_init_t) for p in paths]

    # Print a compact table
    print("name,dt_mean_ms,dt_std_ms,dt_p99_ms,yaw_drift_deg_min,yaw_init_deg,lag_ms")
    for s in sums:
        print(
            f"{s.name},{s.dt_mean_ms:.4f},{s.dt_std_ms:.4f},{s.dt_p99_ms:.4f},"
            f"{s.yaw_drift_deg_min:.3f},{s.yaw_init_deg:.2f},{s.lag_ms:+.2f}"
        )


if __name__ == "__main__":
    main()
