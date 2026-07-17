#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Analyze a normalized golden IMU dataset.

Outputs:
- dt histogram (ms)
- yaw drift slope during a selected window
- latency estimate between measured acc_z and estimated gravity_z (phase alignment)
- innovation metrics over time

No pandas/scipy dependency: uses csv + numpy + matplotlib.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt


@dataclass
class GoldenImu:
    host_ts_us: np.ndarray
    ts_us: np.ndarray
    dt_us: np.ndarray
    raw_acc_mg: np.ndarray  # Nx3
    raw_gyro_mdps: np.ndarray  # Nx3
    raw_mag_cuT: np.ndarray  # Nx3
    g_est_mg: np.ndarray  # Nx3
    innov_acc_milli: np.ndarray
    innov_mag_milli: np.ndarray
    acc_w_milli: np.ndarray
    acc_n_milli: np.ndarray
    rpy_mdeg: np.ndarray  # Nx3


def _load_csv(path: str) -> GoldenImu:
    cols: Dict[str, List[int]] = {}

    int64_min = -(2**63)
    int64_max = 2**63 - 1

    def add(name: str, val: str) -> None:
        if name not in cols:
            cols[name] = []

        if val == "":
            cols[name].append(0)
            return

        try:
            iv = int(val)
        except ValueError:
            cols[name].append(0)
            return

        # Guard against corrupted digit concatenation producing huge integers
        # that would overflow numpy int64 conversion.
        if iv < int64_min or iv > int64_max:
            cols[name].append(0)
            return

        cols[name].append(iv)

    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                add(k, (v or "").strip())

    ts_us = np.asarray(cols["ts_us"], dtype=np.int64)
    dt_us = np.asarray(cols["dt_us"], dtype=np.int64)
    host_ts_us = np.asarray(cols.get("host_ts_us", [0] * len(ts_us)), dtype=np.int64)

    def v3(prefix: str) -> np.ndarray:
        return np.stack(
            [
                np.asarray(cols[f"{prefix}_x"], dtype=np.int64),
                np.asarray(cols[f"{prefix}_y"], dtype=np.int64),
                np.asarray(cols[f"{prefix}_z"], dtype=np.int64),
            ],
            axis=1,
        )

    raw_acc_mg = v3("raw_acc_mg")
    raw_gyro_mdps = v3("raw_gyro_mdps")
    raw_mag_cuT = v3("raw_mag_cuT")
    g_est_mg = v3("g_est_mg")
    rpy_mdeg = np.stack(
        [
            np.asarray(cols["roll_mdeg"], dtype=np.int64),
            np.asarray(cols["pitch_mdeg"], dtype=np.int64),
            np.asarray(cols["yaw_mdeg"], dtype=np.int64),
        ],
        axis=1,
    )

    return GoldenImu(
        host_ts_us=host_ts_us,
        ts_us=ts_us,
        dt_us=dt_us,
        raw_acc_mg=raw_acc_mg,
        raw_gyro_mdps=raw_gyro_mdps,
        raw_mag_cuT=raw_mag_cuT,
        g_est_mg=g_est_mg,
        innov_acc_milli=np.asarray(cols.get("innov_acc_milli", [0] * len(ts_us)), dtype=np.int64),
        innov_mag_milli=np.asarray(cols.get("innov_mag_milli", [0] * len(ts_us)), dtype=np.int64),
        acc_w_milli=np.asarray(cols.get("acc_w_milli", [0] * len(ts_us)), dtype=np.int64),
        acc_n_milli=np.asarray(cols.get("acc_n_milli", [0] * len(ts_us)), dtype=np.int64),
        rpy_mdeg=rpy_mdeg,
    )


def _time_s(ts_us: np.ndarray) -> np.ndarray:
    t0 = ts_us[0]
    return (ts_us - t0).astype(np.float64) * 1e-6


def _unwrap_u32_ts_us(ts_us: np.ndarray) -> Tuple[np.ndarray, Dict[str, int]]:
    """Unwrap uint32 microsecond timestamps into a monotonic int64 timeline.

    Some captures show wrap-like jumps near 2^32 (either forward or backward)
    due to upstream merge/formatting/concatenation artifacts.
    """

    if ts_us.size == 0:
        return ts_us.astype(np.int64), {"adj_pos": 0, "adj_neg": 0}

    ts = ts_us.astype(np.int64)
    wrap = np.int64(2**32)
    thr = wrap // 2  # 2^31

    out = np.empty_like(ts, dtype=np.int64)
    offset = np.int64(0)
    adj_pos = 0
    adj_neg = 0
    out[0] = ts[0]
    for i in range(1, int(ts.size)):
        d = ts[i] - ts[i - 1]
        if d < -thr:
            offset += wrap
            adj_pos += 1
        elif d > thr:
            offset -= wrap
            adj_neg += 1
        out[i] = ts[i] + offset

    return out, {"adj_pos": int(adj_pos), "adj_neg": int(adj_neg)}


def _segment_by_dt(dt_us: np.ndarray, max_dt_us: int) -> List[Tuple[int, int]]:
    """Return segments [i0,i1) where dt stays within (0, max_dt_us]."""

    n = int(dt_us.size) + 1
    if n <= 1:
        return [(0, n)]

    segs: List[Tuple[int, int]] = []
    i0 = 0
    for k, d in enumerate(dt_us.astype(np.int64)):
        if d <= 0 or d > max_dt_us:
            i1 = k + 1
            if (i1 - i0) >= 2:
                segs.append((i0, i1))
            i0 = k + 1
    if (n - i0) >= 2:
        segs.append((i0, n))
    if not segs:
        return [(0, n)]
    return segs

def _time_s_from_dt_us(dt_us: np.ndarray) -> np.ndarray:
    """Reconstruct a monotonic time axis from dt_us (microseconds).

    dt_us is assumed to be the time delta reported by the sample producer.
    This bypasses ts_us corruption at the cost of relying on dt_us accuracy.
    """

    if dt_us.size == 0:
        return np.zeros(0, dtype=np.float64)
    dt = dt_us.astype(np.float64) * 1e-6
    t = np.zeros(int(dt.size), dtype=np.float64)
    if int(dt.size) > 1:
        t[1:] = np.cumsum(dt[:-1])
    return t


def dt_hist(dt_us: np.ndarray, target_hz: float = 400.0) -> None:
    dt_ms = dt_us.astype(np.float64) / 1000.0
    target_ms = 1000.0 / target_hz

    plt.figure(figsize=(10, 4))
    plt.hist(dt_ms, bins=80, alpha=0.75)
    plt.axvline(target_ms, color="r", linestyle="--", label=f"target {target_ms:.3f}ms")
    plt.title("dt histogram")
    plt.xlabel("dt (ms)")
    plt.ylabel("count")
    plt.grid(True, alpha=0.3)
    plt.legend()


def _print_dt_bucket_stats(name: str, dt_us_i64: np.ndarray, dt_target_us: int) -> None:
    if dt_us_i64.size == 0:
        print(f"- {name}: EMPTY")
        return

    dt = dt_us_i64.astype(np.int64)
    dt = dt[(dt > 0) & (dt < 1_000_000)]
    if dt.size == 0:
        print(f"- {name}: EMPTY(after filter)")
        return

    dt_f = dt.astype(np.float64)
    p50 = int(np.percentile(dt_f, 50))
    p90 = int(np.percentile(dt_f, 90))
    p99 = int(np.percentile(dt_f, 99))
    mean = float(np.mean(dt_f))

    c_2500 = int(np.sum(np.abs(dt - dt_target_us) <= 200))
    c_5000 = int(np.sum(np.abs(dt - 2 * dt_target_us) <= 200))
    c_lt_1000 = int(np.sum(dt < 1000))

    print(f"- {name}: n={int(dt.size)} mean={mean:.1f}us p50={p50} p90={p90} p99={p99}")
    print(f"  - near {dt_target_us}us: {c_2500}")
    print(f"  - near {2*dt_target_us}us: {c_5000}")
    print(f"  - <1000us: {c_lt_1000}")


def _find_gaps(ts_us: np.ndarray, expected_dt_us: int) -> np.ndarray:
    if len(ts_us) < 2:
        return np.zeros(0, dtype=np.int64)
    d = np.diff(ts_us.astype(np.int64))

    # Some logs exhibit wrap-like jumps around 2^32 us due to upstream timestamp
    # formatting/merge issues. These are not real stalls and would otherwise inflate
    # discontinuity counts.
    wrap_us = np.int64(2**32)
    wrap_like = (d > (wrap_us * 9) // 10) & (d < (wrap_us * 11) // 10)

    gap = ((d <= 0) | (d > int(expected_dt_us * 1.5))) & (~wrap_like)
    return (np.where(gap)[0] + 1).astype(np.int64)


def _align_sign(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, int]:
    """Align sign of b to a.

    For static data, raw accelerometer Z often has opposite sign vs estimated gravity Z
    depending on whether one is specific force (+g) and the other is gravity (-g).
    We align using median sign for robustness.
    """

    am = float(np.median(a))
    bm = float(np.median(b))
    if am == 0.0 or bm == 0.0:
        return b, 1
    sign = -1 if (am * bm) < 0.0 else 1
    return b * float(sign), sign


def _active_segments(mask: np.ndarray, min_len: int) -> List[Tuple[int, int]]:
    segs: List[Tuple[int, int]] = []
    i = 0
    n = int(mask.size)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j < n and mask[j]:
            j += 1
        if (j - i) >= min_len:
            segs.append((i, j))  # [i, j)
        i = j
    return segs


def yaw_drift(t_s: np.ndarray, yaw_deg: np.ndarray, t_start: float, t_end: float) -> Tuple[float, float]:
    mask = (t_s >= t_start) & (t_s <= t_end)
    if mask.sum() < 10:
        return 0.0, 0.0
    x = t_s[mask]
    y = yaw_deg[mask]
    # Linear fit: y = a*x + b
    a, b = np.polyfit(x, y, 1)
    slope_deg_s = float(a)
    slope_deg_min = slope_deg_s * 60.0
    return slope_deg_min, float(b)


def estimate_latency_ms(t_s: np.ndarray, acc_z_g: np.ndarray, g_est_z_g: np.ndarray) -> float:
    # Use high-pass to emphasize dynamics; avoid static baseline dominating correlation.
    def hp(x: np.ndarray, win: int = 50) -> np.ndarray:
        if len(x) < win * 2:
            return x - np.mean(x)
        kernel = np.ones(win, dtype=np.float64) / float(win)
        lp = np.convolve(x, kernel, mode="same")
        return x - lp

    a = hp(acc_z_g)
    b = hp(g_est_z_g)

    # Normalize
    a = a - np.mean(a)
    b = b - np.mean(b)
    sa = np.std(a)
    sb = np.std(b)
    if sa < 1e-6 or sb < 1e-6:
        return 0.0
    a /= sa
    b /= sb

    # Cross-correlation
    max_lag = min(400, len(a) // 4)  # limit to avoid huge compute
    corr = np.correlate(a, b, mode="full")
    mid = len(corr) // 2
    window = corr[mid - max_lag : mid + max_lag + 1]
    best_idx = int(np.argmax(window))
    best = best_idx - max_lag

    # Sub-sample refinement: parabolic interpolation around the peak.
    # This reduces quantization when the true lag is between samples.
    best_frac = float(best)
    if 0 < best_idx < (len(window) - 1):
        y0 = float(window[best_idx])
        y1 = float(window[best_idx - 1])
        y2 = float(window[best_idx + 1])
        denom = (y1 - 2.0 * y0 + y2)
        if abs(denom) > 1e-12:
            # Vertex offset in samples, in [-1, 1] for a well-behaved peak.
            delta = 0.5 * (y1 - y2) / denom
            if -1.0 <= delta <= 1.0:
                best_frac = float(best) + float(delta)

    dt_mean_s = float(np.mean(np.diff(t_s))) if len(t_s) > 1 else 0.0
    return best_frac * dt_mean_s * 1000.0


def estimate_sensor_delay_ms(
    t_s: np.ndarray,
    raw_acc_mg: np.ndarray,
    raw_gyro_mdps: np.ndarray,
    gyro_norm_dps: np.ndarray,
    gyro_threshold_dps: float,
    min_active_s: float,
    search_ms: Tuple[float, float, float],
) -> Tuple[float, float, int, float, int]:
    """Estimate relative timing offset between gyro and accel.

    Method:
    - Integrate gyro to predict gravity direction in body frame.
    - Compare with accel direction (normalized) shifted in time.
    - Choose time shift that minimizes RMS angular error during dynamic segments.

    Convention:
    - We evaluate accel(t + shift) against gyro-predicted gravity at time t.
    - If accel is delayed by +d, best shift tends to be +d.
    """

    if t_s.size < 50:
        return 0.0, 0.0, 0, 0.0, 0

    # Build unit accel direction
    acc = raw_acc_mg.astype(np.float64) / 1000.0
    acc_norm = np.linalg.norm(acc, axis=1)
    acc_ok = acc_norm > 1e-3
    acc_dir = np.zeros_like(acc)
    acc_dir[acc_ok] = acc[acc_ok] / acc_norm[acc_ok][:, None]

    # Dynamic selection (gyro-based) and prefer near-1g magnitude to reduce linear-acc contamination.
    dt_mean_s = float(np.mean(np.diff(t_s)))
    if dt_mean_s <= 0.0:
        return 0.0, 0.0, 0

    # Instead of requiring long continuous segments (which often fails on short hand motions),
    # mark samples where gyro exceeds a threshold and dilate them over a short window.
    win_len = max(1, int(max(min_active_s, 0.05) / dt_mean_s))

    # If too few points at the requested threshold, progressively relax it.
    thr_list = [gyro_threshold_dps, gyro_threshold_dps * 0.5, gyro_threshold_dps * 0.25, 0.2]
    dyn = np.zeros(t_s.shape[0], dtype=bool)
    used_thr = float(thr_list[-1])
    for thr in thr_list:
        dyn0 = gyro_norm_dps > float(thr)
        if win_len > 1:
            kernel = np.ones(win_len, dtype=np.int64)
            dyn = (np.convolve(dyn0.astype(np.int64), kernel, mode="same") > 0)
        else:
            dyn = dyn0
        if int(np.sum(dyn)) >= 50:
            used_thr = float(thr)
            break
    if int(np.sum(dyn)) < 50:
        return 0.0, 0.0, 0, float(used_thr), int(np.sum(dyn))

    acc_1g = np.abs(acc_norm - 1.0) < 0.15
    base_mask = dyn & acc_1g
    if int(np.sum(base_mask)) < 50:
        base_mask = dyn
        if int(np.sum(base_mask)) < 50:
            return 0.0, 0.0, 0, float(used_thr), int(np.sum(dyn))

    # Predict gravity direction by integrating g_dot = -(w x g).
    gyro_rad_s = raw_gyro_mdps.astype(np.float64) * (1e-3 * (np.pi / 180.0))
    g_pred = np.zeros_like(acc_dir)
    g0 = acc_dir[0].copy()
    if np.linalg.norm(g0) < 1e-6:
        g0 = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    g0 /= np.linalg.norm(g0)
    g_pred[0] = g0
    for k in range(t_s.size - 1):
        dt = float(t_s[k + 1] - t_s[k])
        if dt <= 0.0 or dt > 0.2:
            g_pred[k + 1] = g_pred[k]
            continue
        w = gyro_rad_s[k]
        gk = g_pred[k]
        g_next = gk + (-np.cross(w, gk)) * dt
        ng = float(np.linalg.norm(g_next))
        if ng > 1e-9:
            g_next /= ng
        g_pred[k + 1] = g_next

    # Search time shift (ms)
    ms0, ms1, ms_step = search_ms
    if ms_step <= 0:
        return 0.0, 0.0, 0, float(used_thr), int(np.sum(dyn))
    shifts_ms = np.arange(ms0, ms1 + 0.5 * ms_step, ms_step, dtype=np.float64)

    best_ms = 0.0
    best_rms_deg = 1e9
    best_n = 0

    t0 = float(t_s[0])
    tN = float(t_s[-1])
    for shift_ms in shifts_ms:
        shift_s = float(shift_ms) / 1000.0
        tq = t_s + shift_s

        oob = (tq < t0) | (tq > tN)

        ax = np.interp(tq, t_s, acc_dir[:, 0])
        ay = np.interp(tq, t_s, acc_dir[:, 1])
        az = np.interp(tq, t_s, acc_dir[:, 2])
        a_shift = np.stack([ax, ay, az], axis=1)

        # Mark out-of-bounds as invalid
        if np.any(oob):
            a_shift[oob] = np.nan

        an = np.linalg.norm(a_shift, axis=1)
        ok = np.isfinite(an) & (an > 1e-6) & base_mask
        if int(np.sum(ok)) < 50:
            continue
        a_shift[ok] = a_shift[ok] / an[ok][:, None]

        dots = np.sum(g_pred[ok] * a_shift[ok], axis=1)
        dots = np.clip(dots, -1.0, 1.0)
        ang = np.arccos(dots)
        rms_deg = float(np.sqrt(np.mean(ang * ang)) * (180.0 / np.pi))

        if rms_deg < best_rms_deg:
            best_rms_deg = rms_deg
            best_ms = float(shift_ms)
            best_n = int(np.sum(ok))

    if best_rms_deg >= 1e8:
        return 0.0, 0.0, 0, float(used_thr), int(np.sum(dyn))
    return best_ms, best_rms_deg, best_n, float(used_thr), int(np.sum(dyn))


def evaluate_sensor_delay_rms_deg(
    t_s: np.ndarray,
    raw_acc_mg: np.ndarray,
    raw_gyro_mdps: np.ndarray,
    gyro_norm_dps: np.ndarray,
    gyro_threshold_dps: float,
    min_active_s: float,
    shift_ms: float,
) -> Tuple[float, int]:
    """Return (rms_deg, used_samples) for a specific accel shift (ms)."""
    if t_s.size < 50:
        return 0.0, 0

    acc = raw_acc_mg.astype(np.float64) / 1000.0
    acc_norm = np.linalg.norm(acc, axis=1)
    acc_ok = acc_norm > 1e-3
    acc_dir = np.zeros_like(acc)
    acc_dir[acc_ok] = acc[acc_ok] / acc_norm[acc_ok][:, None]

    dt_mean_s = float(np.mean(np.diff(t_s)))
    if dt_mean_s <= 0.0:
        return 0.0, 0

    win_len = max(1, int(max(min_active_s, 0.05) / dt_mean_s))
    thr_list = [gyro_threshold_dps, gyro_threshold_dps * 0.5, gyro_threshold_dps * 0.25, 0.2]
    dyn = np.zeros(t_s.shape[0], dtype=bool)
    for thr in thr_list:
        dyn0 = gyro_norm_dps > float(thr)
        if win_len > 1:
            kernel = np.ones(win_len, dtype=np.int64)
            dyn = (np.convolve(dyn0.astype(np.int64), kernel, mode="same") > 0)
        else:
            dyn = dyn0
        if int(np.sum(dyn)) >= 50:
            break
    if int(np.sum(dyn)) < 50:
        return 0.0, 0

    acc_1g = np.abs(acc_norm - 1.0) < 0.15
    base_mask = dyn & acc_1g
    if int(np.sum(base_mask)) < 50:
        base_mask = dyn
        if int(np.sum(base_mask)) < 50:
            return 0.0, 0

    gyro_rad_s = raw_gyro_mdps.astype(np.float64) * (1e-3 * (np.pi / 180.0))
    g_pred = np.zeros_like(acc_dir)
    g0 = acc_dir[0].copy()
    if np.linalg.norm(g0) < 1e-6:
        g0 = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    g0 /= np.linalg.norm(g0)
    g_pred[0] = g0
    for k in range(t_s.size - 1):
        dt = float(t_s[k + 1] - t_s[k])
        if dt <= 0.0 or dt > 0.2:
            g_pred[k + 1] = g_pred[k]
            continue
        w = gyro_rad_s[k]
        gk = g_pred[k]
        g_next = gk + (-np.cross(w, gk)) * dt
        ng = float(np.linalg.norm(g_next))
        if ng > 1e-9:
            g_next /= ng
        g_pred[k + 1] = g_next

    t0 = float(t_s[0])
    tN = float(t_s[-1])
    shift_s = float(shift_ms) / 1000.0
    tq = t_s + shift_s
    oob = (tq < t0) | (tq > tN)

    ax = np.interp(tq, t_s, acc_dir[:, 0])
    ay = np.interp(tq, t_s, acc_dir[:, 1])
    az = np.interp(tq, t_s, acc_dir[:, 2])
    a_shift = np.stack([ax, ay, az], axis=1)
    if np.any(oob):
        a_shift[oob] = np.nan

    an = np.linalg.norm(a_shift, axis=1)
    ok = np.isfinite(an) & (an > 1e-6) & base_mask
    used_n = int(np.sum(ok))
    if used_n < 50:
        return 0.0, 0
    a_shift[ok] = a_shift[ok] / an[ok][:, None]

    dots = np.sum(g_pred[ok] * a_shift[ok], axis=1)
    dots = np.clip(dots, -1.0, 1.0)
    ang = np.arccos(dots)
    rms_deg = float(np.sqrt(np.mean(ang * ang)) * (180.0 / np.pi))
    return rms_deg, used_n


def estimate_latency_ms_dynamic(
    t_s: np.ndarray,
    acc_z_g: np.ndarray,
    g_est_z_g: np.ndarray,
    gyro_norm_dps: np.ndarray,
    gyro_threshold_dps: float,
    min_active_s: float,
) -> Tuple[float, int]:
    if len(t_s) < 5:
        return 0.0, 0

    dt_mean_s = float(np.mean(np.diff(t_s)))
    if dt_mean_s <= 0:
        return 0.0, 0

    min_len = max(10, int(min_active_s / dt_mean_s))
    active = gyro_norm_dps > gyro_threshold_dps
    segs = _active_segments(active, min_len=min_len)
    if not segs:
        return estimate_latency_ms(t_s, acc_z_g, g_est_z_g), 0

    lags: List[float] = []
    for (i0, i1) in segs:
        lag = estimate_latency_ms(t_s[i0:i1], acc_z_g[i0:i1], g_est_z_g[i0:i1])
        lags.append(lag)

    return float(np.median(np.asarray(lags, dtype=np.float64))), len(segs)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("golden_csv", help="Path to golden_imu.csv")
    ap.add_argument("--target-hz", type=float, default=400.0)
    ap.add_argument(
        "--time-base",
        choices=["auto", "host", "ts", "dt"],
        default="auto",
        help=(
            "Time base: host=use host_ts_us from serial capture (robust wall time), "
            "ts=use ts_us (with unwrap+segment), dt=reconstruct from dt_us, "
            "auto=prefer host if available, else ts unless too broken"
        ),
    )
    ap.add_argument(
        "--analysis-window",
        default=None,
        help="Optional analysis window in seconds: start,end (applies to drift/lag/plots)",
    )
    ap.add_argument("--yaw-window", default="2,30", help="Yaw drift fit window in seconds: start,end")
    ap.add_argument("--gyro-threshold-dps", type=float, default=20.0, help="Threshold for dynamic segment detection")
    ap.add_argument("--min-active-s", type=float, default=0.5, help="Min dynamic segment duration")
    ap.add_argument(
        "--sensor-delay-search",
        default="-20,20,0.5",
        help="Search relative sensor delay in ms as start,end,step (gyro-integrated gravity vs accel)",
    )
    ap.add_argument(
        "--sensor-delay-eval",
        default=None,
        help="Optional list of shift(ms) to evaluate on the SAME dataset, e.g. 0,1,-1",
    )
    ap.add_argument("--show", action="store_true", help="Show plots")
    args = ap.parse_args()

    data = _load_csv(args.golden_csv)

    # Some CSV rows may be partially empty due to line wrapping/corruption during capture.
    # Those get parsed as zeros; exclude them from signal-based analysis.
    acc_norm = np.linalg.norm(data.raw_acc_mg.astype(np.float64), axis=1)
    g_norm = np.linalg.norm(data.g_est_mg.astype(np.float64), axis=1)
    valid = (acc_norm > 100.0) & (g_norm > 100.0)
    if int(np.sum(valid)) < 20:
        valid = np.ones_like(valid, dtype=bool)

    ts_us_raw = data.ts_us[valid]
    host_ts_us_raw = data.host_ts_us[valid]
    dt_us_i64 = data.dt_us[valid]
    raw_acc_mg = data.raw_acc_mg[valid]
    raw_gyro_mdps = data.raw_gyro_mdps[valid]
    g_est_mg = data.g_est_mg[valid]
    rpy_mdeg = data.rpy_mdeg[valid]
    innov_acc_milli = data.innov_acc_milli[valid]
    innov_mag_milli = data.innov_mag_milli[valid]

    # Target dt (used for continuity thresholding)
    dt_target_us = int(round(1e6 / args.target_hz))

    # Quick dt cross-checks (before choosing time base):
    # - producer dt_us column
    # - capture-side host_ts_us delta (after index interpolation)
    # - producer ts_us delta
    print("dt cross-check (raw):")
    _print_dt_bucket_stats("fw dt_us col", dt_us_i64, dt_target_us)

    host_ok_raw = host_ts_us_raw > 0
    if int(np.sum(host_ok_raw)) >= 2:
        host_dt = np.diff(host_ts_us_raw[host_ok_raw].astype(np.int64))
        _print_dt_bucket_stats("host_ts_us delta", host_dt, dt_target_us)
    else:
        print("- host_ts_us delta: MISSING")

    if ts_us_raw.size >= 2:
        ts_u, _ = _unwrap_u32_ts_us(ts_us_raw)
        ts_dt = np.diff(ts_u.astype(np.int64))
        _print_dt_bucket_stats("ts_us delta", ts_dt, dt_target_us)
    else:
        print("- ts_us delta: MISSING")

    # Decide time base early to avoid shape-mismatch bugs when ts-based logic slices arrays.
    # In auto mode, choose the time base whose dt distribution is closer to the target period.
    auto_mode = args.time_base == "auto"
    time_base = args.time_base
    if time_base == "auto":
        def _dt_score(dt_us: np.ndarray, target_us: int) -> float:
            if dt_us.size < 16:
                return float("inf")
            # Robust metric: median absolute error to target (lower is better).
            return float(np.median(np.abs(dt_us.astype(np.int64) - np.int64(target_us))))

        # ts score (always available for golden_imu.csv)
        ts_u, _ = _unwrap_u32_ts_us(ts_us_raw)
        ts_dt = np.diff(ts_u.astype(np.int64)) if ts_u.size >= 2 else np.zeros(0, dtype=np.int64)
        ts_score = _dt_score(ts_dt, dt_target_us)

        # host score (only if present)
        if int(np.sum(host_ok_raw)) >= 20:
            host_dt = np.diff(host_ts_us_raw[host_ok_raw].astype(np.int64))
            host_score = _dt_score(host_dt, dt_target_us)
        else:
            host_score = float("inf")

        time_base = "host" if host_score <= ts_score else "ts"

    # Build time axis.
    gap_idx_all = np.zeros(0, dtype=np.int64)
    gap_idx = np.zeros(0, dtype=np.int64)
    uw_stats: Dict[str, int] = {"adj_pos": 0, "adj_neg": 0}
    dt_med = dt_target_us
    max_dt_us = dt_target_us * 6

    if time_base == "ts":
        ts_us_unwrapped, uw_stats = _unwrap_u32_ts_us(ts_us_raw)
        t_s_all = _time_s(ts_us_unwrapped)
        dt_from_ts_us = (
            np.diff(ts_us_unwrapped.astype(np.int64)) if len(ts_us_unwrapped) > 1 else np.zeros(0, dtype=np.int64)
        )
        dt_pos = dt_from_ts_us[(dt_from_ts_us > 0) & (dt_from_ts_us < 100_000)]
        dt_med = int(np.median(dt_pos)) if dt_pos.size else dt_target_us
        max_dt_us = int(max(dt_med * 4, dt_target_us * 6))
        segs = _segment_by_dt(dt_from_ts_us, max_dt_us=max_dt_us)
        i0, i1 = max(segs, key=lambda ij: (ij[1] - ij[0]))
        gap_idx_all = (np.where((dt_from_ts_us <= 0) | (dt_from_ts_us > max_dt_us))[0] + 1).astype(np.int64)
        gap_idx = gap_idx_all[(gap_idx_all >= i0) & (gap_idx_all < i1)] - np.int64(i0)

        # If ts-based segment is too short, auto-fallback to dt-based timeline.
        seg_len = int(i1 - i0)
        if not auto_mode:
            use_dt_fallback = False
        else:
            use_dt_fallback = (seg_len < 200) or (seg_len < int(0.5 * int(np.sum(valid))))

        if not use_dt_fallback:
            # Slice to segment
            t_s = t_s_all[i0:i1]
            dt_us_i64 = dt_us_i64[i0:i1]
            raw_acc_mg = raw_acc_mg[i0:i1]
            raw_gyro_mdps = raw_gyro_mdps[i0:i1]
            g_est_mg = g_est_mg[i0:i1]
            rpy_mdeg = rpy_mdeg[i0:i1]
            innov_acc_milli = innov_acc_milli[i0:i1]
            innov_mag_milli = innov_mag_milli[i0:i1]
            time_base_used = "ts(seg)"
        else:
            # Fall back to dt-based full-run time axis.
            t_s = _time_s_from_dt_us(dt_us_i64)
            gap_idx = np.zeros(0, dtype=np.int64)
            time_base_used = "dt(fallback)"
    elif args.time_base == "dt":
        t_s = _time_s_from_dt_us(dt_us_i64)
        time_base_used = "dt"
    else:
        # host time base (or auto preference when available): use capture-side timestamps.
        # These reflect wall-clock elapsed time even when IMUCSV logs are decimated.
        host_ok = host_ts_us_raw > 0
        if int(np.sum(host_ok)) < 20:
            # If host timestamps are missing, fall back to dt.
            t_s = _time_s_from_dt_us(dt_us_i64)
            time_base_used = "dt(host-missing)"
        else:
            # Keep only rows with host timestamps to ensure monotonic time.
            t_s = _time_s(host_ts_us_raw[host_ok])
            dt_us_i64 = dt_us_i64[host_ok]
            raw_acc_mg = raw_acc_mg[host_ok]
            raw_gyro_mdps = raw_gyro_mdps[host_ok]
            g_est_mg = g_est_mg[host_ok]
            rpy_mdeg = rpy_mdeg[host_ok]
            innov_acc_milli = innov_acc_milli[host_ok]
            innov_mag_milli = innov_mag_milli[host_ok]
            gap_idx = np.zeros(0, dtype=np.int64)
            time_base_used = "host(auto)" if auto_mode else "host"

    yaw_deg = rpy_mdeg[:, 2].astype(np.float64) / 1000.0
    acc_z_g = raw_acc_mg[:, 2].astype(np.float64) / 1000.0
    g_est_z_g = g_est_mg[:, 2].astype(np.float64) / 1000.0

    g_est_z_aligned_g, g_sign = _align_sign(acc_z_g, g_est_z_g)
    gyro_dps = raw_gyro_mdps.astype(np.float64) / 1000.0
    gyro_norm_dps = np.linalg.norm(gyro_dps, axis=1)

    # Dynamics quick check: delay estimation is ill-posed on near-static data.
    if gyro_norm_dps.size > 0:
        p50_g = float(np.percentile(gyro_norm_dps, 50))
        p90_g = float(np.percentile(gyro_norm_dps, 90))
        p95_g = float(np.percentile(gyro_norm_dps, 95))
        p99_g = float(np.percentile(gyro_norm_dps, 99))
        gmax = float(np.max(gyro_norm_dps))

        def frac(thr: float) -> float:
            return float(np.mean(gyro_norm_dps > thr))

        print("dynamics (gyro norm):")
        print(f"- p50={p50_g:.2f} dps p90={p90_g:.2f} p95={p95_g:.2f} p99={p99_g:.2f} max={gmax:.2f}")
        print(
            "- frac(>2/5/10/20 dps)="
            f"{frac(2.0)*100.0:.1f}%/"
            f"{frac(5.0)*100.0:.1f}%/"
            f"{frac(10.0)*100.0:.1f}%/"
            f"{frac(20.0)*100.0:.1f}%"
        )

    # Optional analysis window to "re-test a segment".
    if args.analysis_window is not None:
        a0, a1 = [float(x.strip()) for x in args.analysis_window.split(",")]
        am = (t_s >= a0) & (t_s <= a1)
        if int(np.sum(am)) >= 20:
            t_s = t_s[am]
            yaw_deg = yaw_deg[am]
            acc_z_g = acc_z_g[am]
            g_est_z_g = g_est_z_g[am]
            g_est_z_aligned_g, g_sign = _align_sign(acc_z_g, g_est_z_g)
            gyro_norm_dps = gyro_norm_dps[am]
            innov_acc_milli = innov_acc_milli[am]
            innov_mag_milli = innov_mag_milli[am]
            raw_acc_mg = raw_acc_mg[am]
            raw_gyro_mdps = raw_gyro_mdps[am]
            dt_us_i64 = dt_us_i64[am]
            gap_idx = np.zeros(0, dtype=np.int64)

    # dt stats
    dt_us = dt_us_i64.astype(np.float64)
    dt_ms = dt_us / 1000.0
    print("dt stats:")
    print(f"- mean: {dt_ms.mean():.4f} ms")
    print(f"- std : {dt_ms.std():.4f} ms")
    print(f"- p99 : {np.percentile(dt_ms, 99):.4f} ms")

    print(f"time base: {time_base_used}")
    if t_s.size > 0:
        print(f"duration: {float(t_s[-1] - t_s[0]):.3f} s")

    print(f"samples used (valid): {int(np.sum(valid))}/{len(data.ts_us)}")
    if time_base_used == "ts(seg)":
        print(f"samples used (segment): {len(t_s)}/{int(np.sum(valid))}")

    t0, t1 = [float(x.strip()) for x in args.yaw_window.split(",")]
    drift_deg_min, _ = yaw_drift(t_s, yaw_deg, t0, t1)

    if uw_stats["adj_pos"] or uw_stats["adj_neg"]:
        print("ts_us unwrap:")
        print(f"- +2^32 adjustments: {uw_stats['adj_pos']}")
        print(f"- -2^32 adjustments: {uw_stats['adj_neg']}")

    # 400Hz sanity checks (dt_us should cluster near dt_target_us)
    c_2500 = int(np.sum(np.abs(dt_us_i64.astype(np.int64) - dt_target_us) <= 200))
    c_5000 = int(np.sum(np.abs(dt_us_i64.astype(np.int64) - 2 * dt_target_us) <= 200))
    c_lt_1000 = int(np.sum(dt_us_i64.astype(np.int64) < 1000))
    print("dt_us checks:")
    print(f"- near {dt_target_us}us: {c_2500}/{len(dt_us)}")
    print(f"- near {2*dt_target_us}us: {c_5000}/{len(dt_us)} (drop/skip suspect)")
    print(f"- <1000us: {c_lt_1000}/{len(dt_us)} (resample/clock suspect)")

    print("yaw drift:")
    print(f"- window: [{t0:.1f}, {t1:.1f}] s")
    print(f"- slope : {drift_deg_min:.3f} deg/min")

    lag_ms, nseg = estimate_latency_ms_dynamic(
        t_s,
        acc_z_g,
        g_est_z_aligned_g,
        gyro_norm_dps,
        gyro_threshold_dps=args.gyro_threshold_dps,
        min_active_s=args.min_active_s,
    )
    sign_note = "g_est" if g_sign > 0 else "-g_est"
    print(f"latency (acc_z vs {sign_note}_z):")
    if nseg > 0:
        print(f"- estimated lag: {lag_ms:+.2f} ms (dynamic segments: {nseg})")
    else:
        print(f"- estimated lag: {lag_ms:+.2f} ms (fallback: full run)")

    # Relative sensor delay estimate (gyro-integrated gravity vs accel)
    try:
        s0, s1, ss = [float(x.strip()) for x in args.sensor_delay_search.split(",")]
        best_ms, rms_deg, used_n, used_thr_dps, dyn_n = estimate_sensor_delay_ms(
            t_s,
            raw_acc_mg,
            raw_gyro_mdps,
            gyro_norm_dps,
            gyro_threshold_dps=args.gyro_threshold_dps,
            min_active_s=args.min_active_s,
            search_ms=(s0, s1, ss),
        )
        print("imu timing (gyro→acc gravity alignment):")
        if used_n > 0:
            print(f"- best shift: {best_ms:+.2f} ms (search {s0:+.1f}..{s1:+.1f} step {ss:.2f})")
            print(f"- rms error: {rms_deg:.3f} deg (samples: {used_n})")
            print(f"- dyn selection: used_thr>{used_thr_dps:.2f} dps, dyn_samples={dyn_n}")

            # If we had to relax too far, the result is likely driven by noise.
            if used_thr_dps <= 0.21 or dyn_n < 200:
                print("- [WARN] motion is weak; delay optimum may be flat/unreliable")
        else:
            print("- [WARN] insufficient dynamics to estimate")

        if args.sensor_delay_eval is not None:
            eval_ms = [float(x.strip()) for x in args.sensor_delay_eval.split(",") if x.strip()]
            if eval_ms:
                print("imu timing (eval same-data shifts):")
                for ms in eval_ms:
                    er, n = evaluate_sensor_delay_rms_deg(
                        t_s,
                        raw_acc_mg,
                        raw_gyro_mdps,
                        gyro_norm_dps,
                        gyro_threshold_dps=args.gyro_threshold_dps,
                        min_active_s=args.min_active_s,
                        shift_ms=ms,
                    )
                    if n > 0:
                        print(f"- shift {ms:+.2f} ms -> rms {er:.3f} deg (n={n})")
                    else:
                        print(f"- shift {ms:+.2f} ms -> [insufficient]")
    except Exception:
        print("imu timing (gyro→acc gravity alignment):")
        print("- [WARN] failed to estimate (bad --sensor-delay-search?)")

    # Timestamp continuity summary (based on unwrapped ts)
    # Timestamp continuity summary (based on unwrapped ts)
    if time_base_used == "ts(seg)" and int(gap_idx_all.size) > 0:
        print(
            f"[WARN] Detected {int(gap_idx_all.size)} ts_us discontinuities (ts-based, max_dt_us={max_dt_us}, median_dt_us={dt_med})"
        )

    # Plots
    dt_hist(dt_us_i64, target_hz=args.target_hz)

    plt.figure(figsize=(12, 5))
    plt.plot(t_s, yaw_deg, label="yaw (deg)")
    for gi in gap_idx:
        plt.axvline(t_s[int(gi)], color="k", alpha=0.15)
    plt.title("Yaw")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.figure(figsize=(12, 5))
    plt.plot(t_s, innov_acc_milli.astype(np.float64) / 1000.0, label="innov_acc")
    plt.plot(t_s, innov_mag_milli.astype(np.float64) / 1000.0, label="innov_mag")
    for gi in gap_idx:
        plt.axvline(t_s[int(gi)], color="k", alpha=0.15)
    plt.title("Innovation (dimensionless)")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.figure(figsize=(12, 5))
    plt.plot(t_s, acc_z_g, label="acc_z (g)")
    plt.plot(t_s, g_est_z_aligned_g, label=f"{sign_note}_z (g, unit)")
    for gi in gap_idx:
        plt.axvline(t_s[int(gi)], color="k", alpha=0.15)
    plt.title("Delay check: acc vs estimated gravity")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    # Gyro norm (helps validate dynamic segment detection)
    plt.figure(figsize=(12, 4))
    plt.plot(t_s, gyro_norm_dps, label="|gyro| (dps)")
    plt.axhline(args.gyro_threshold_dps, color="r", linestyle="--", label="threshold")
    for gi in gap_idx:
        plt.axvline(t_s[int(gi)], color="k", alpha=0.15)
    plt.title("Gyro activity")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)
    plt.legend()

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
