#!/usr/bin/env python3
"""Run a reviewed multi-seed navigation consistency baseline."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np


LIMITS = {
    "position_rmse_m": 0.75,
    "velocity_rmse_m_s": 0.40,
    "attitude_rmse_deg": 2.00,
    "navigation_recoveries": 0.0,
    "minimum_healthy_ratio": 1.0,
}


def parse_seeds(expression: str) -> list[int]:
    seeds: list[int] = []
    for item in expression.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            start_text, stop_text = item.split(":", 1)
            seeds.extend(range(int(start_text), int(stop_text)))
        else:
            seeds.append(int(item))
    unique = list(dict.fromkeys(seeds))
    if not unique:
        raise ValueError("at least one seed is required")
    return unique


def extract_trial(seed: int, metrics: dict[str, object]) -> dict[str, float | int]:
    navigation = metrics["navigation"]
    integrity = metrics["eskf_integrity"]
    consistency = metrics["eskf_consistency"]
    alignment = metrics["cold_start_alignment"]
    assert isinstance(navigation, dict)
    assert isinstance(integrity, dict)
    assert isinstance(consistency, dict)
    assert isinstance(alignment, dict)
    position_nis = consistency["position_nis"]
    velocity_nis = consistency["velocity_nis"]
    navigation_nees = consistency["navigation_nees"]
    assert isinstance(position_nis, dict)
    assert isinstance(velocity_nis, dict)
    assert isinstance(navigation_nees, dict)
    return {
        "seed": seed,
        "position_rmse_m": float(navigation["position_rmse_m"]),
        "velocity_rmse_m_s": float(navigation["velocity_rmse_m_s"]),
        "attitude_rmse_deg": float(alignment["post_alignment_attitude_rmse_deg"]),
        "position_nis_mean": float(position_nis["mean"]),
        "velocity_nis_mean": float(velocity_nis["mean"]),
        "navigation_nees_mean": float(navigation_nees["mean"]),
        "healthy_ratio": float(integrity["healthy_ratio"]),
        "navigation_recoveries": int(integrity["navigation_recoveries"]),
    }


def summarize_trials(trials: list[dict[str, float | int]]) -> dict[str, object]:
    metric_names = [
        "position_rmse_m",
        "velocity_rmse_m_s",
        "attitude_rmse_deg",
        "position_nis_mean",
        "velocity_nis_mean",
        "navigation_nees_mean",
        "healthy_ratio",
        "navigation_recoveries",
    ]
    aggregates: dict[str, dict[str, float]] = {}
    for name in metric_names:
        values = np.asarray([float(trial[name]) for trial in trials], dtype=np.float64)
        aggregates[name] = {
            "minimum": float(np.min(values)),
            "mean": float(np.mean(values)),
            "p05": float(np.percentile(values, 5)),
            "p95": float(np.percentile(values, 95)),
            "maximum": float(np.max(values)),
        }

    failures: list[dict[str, object]] = []
    for trial in trials:
        reasons: list[str] = []
        for name in ("position_rmse_m", "velocity_rmse_m_s", "attitude_rmse_deg"):
            if float(trial[name]) > LIMITS[name]:
                reasons.append(f"{name}={float(trial[name]):.6g}>{LIMITS[name]:.6g}")
        if float(trial["healthy_ratio"]) < LIMITS["minimum_healthy_ratio"]:
            reasons.append(
                f"healthy_ratio={float(trial['healthy_ratio']):.6g}"
                f"<{LIMITS['minimum_healthy_ratio']:.6g}"
            )
        if float(trial["navigation_recoveries"]) > LIMITS["navigation_recoveries"]:
            reasons.append(
                f"navigation_recoveries={int(trial['navigation_recoveries'])}"
                f">{int(LIMITS['navigation_recoveries'])}"
            )
        if reasons:
            failures.append({"seed": int(trial["seed"]), "reasons": reasons})
    return {"aggregates": aggregates, "failures": failures}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/monte-carlo"))
    parser.add_argument("--seeds", default="0:20", help="comma list and/or half-open ranges")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--accel-bias-std-m-s2", type=float, default=0.05)
    parser.add_argument("--gyro-bias-std-deg-s", type=float, default=0.20)
    parser.add_argument("--timestamp-jitter-std-us", type=float, default=250.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    seeds = parse_seeds(args.seeds)
    environment = os.environ.copy()
    environment["MPLCONFIGDIR"] = str((args.out_dir / ".matplotlib").resolve())
    Path(environment["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    trials: list[dict[str, float | int]] = []
    for seed in seeds:
        trial_dir = args.out_dir / f"seed-{seed:04d}"
        subprocess.run(
            [
                sys.executable,
                str(root / "validation/run_suite.py"),
                "--runner", str(args.runner.resolve()),
                "--out-dir", str(trial_dir),
                "--scenarios", "navigation_outage",
                "--duration", str(args.duration),
                "--rate", str(args.rate),
                "--seed", str(seed),
                "--accel-bias-std-m-s2", str(args.accel_bias_std_m_s2),
                "--gyro-bias-std-deg-s", str(args.gyro_bias_std_deg_s),
                "--timestamp-jitter-std-us", str(args.timestamp_jitter_std_us),
            ],
            cwd=root,
            env=environment,
            check=True,
            stdout=subprocess.DEVNULL,
        )
        metrics = json.loads((trial_dir / "navigation_outage/metrics.json").read_text(
            encoding="utf-8"
        ))
        trial = extract_trial(seed, metrics)
        generation = json.loads((trial_dir / "navigation_outage/input-metadata.json").read_text(
            encoding="utf-8"
        ))
        trial["generation"] = generation
        trials.append(trial)
        print(
            f"seed={seed:4d} position={trial['position_rmse_m']:.3f} m "
            f"velocity={trial['velocity_rmse_m_s']:.3f} m/s "
            f"NEES={trial['navigation_nees_mean']:.3f}"
        )

    summary = summarize_trials(trials)
    output = {
        "schema_version": 1,
        "scope": {
            "scenario": "navigation_outage",
            "seeds": seeds,
            "duration_s": args.duration,
            "rate_hz": args.rate,
            "accel_bias_std_m_s2": args.accel_bias_std_m_s2,
            "gyro_bias_std_deg_s": args.gyro_bias_std_deg_s,
            "timestamp_jitter_std_us": args.timestamp_jitter_std_us,
            "covered": [
                "independent randomized IMU, magnetometer, GNSS position, and GNSS velocity noise",
                "randomized constant three-axis accelerometer and gyroscope bias",
                "monotonic per-interval IMU timestamp jitter",
                "cold-start alignment",
                "five-second GNSS aiding outage and recovery",
                "navigation NIS and posterior velocity-position NEES",
            ],
            "not_covered": [
                "temperature-dependent or time-varying IMU bias",
                "transport delay, reordering, or dropped IMU samples",
                "independent physical truth or flight qualification",
            ],
        },
        "limits": LIMITS,
        "trials": trials,
        **summary,
    }
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "summary.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    aggregates = summary["aggregates"]
    assert isinstance(aggregates, dict)
    lines = [
        "# Navigation Monte Carlo baseline",
        "",
        f"Seeds: `{seeds[0]}` through `{seeds[-1]}` ({len(seeds)} trials).",
        "",
        "| Metric | Mean | P05 | P95 | Maximum |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name in (
        "position_rmse_m", "velocity_rmse_m_s", "attitude_rmse_deg",
        "position_nis_mean", "velocity_nis_mean", "navigation_nees_mean",
    ):
        values = aggregates[name]
        lines.append(
            f"| {name} | {values['mean']:.4f} | {values['p05']:.4f} "
            f"| {values['p95']:.4f} | {values['maximum']:.4f} |"
        )
    failures = summary["failures"]
    lines.extend([
        "",
        f"Acceptance failures: **{len(failures)}**.",
        "",
        "P05/P95 are empirical percentiles across seeds, not independent-sample theoretical "
        "confidence bounds. This baseline randomizes measurement noise, constant IMU bias, and "
        "monotonic timestamp jitter while exercising the fixed five-second aiding outage. "
        "Temperature drift and transport faults remain separate P0 work.",
    ])
    (args.out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if failures:
        raise SystemExit(f"Monte Carlo acceptance failed for {len(failures)} seed(s)")
    print(f"Monte Carlo baseline passed: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
