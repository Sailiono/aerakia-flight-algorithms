#!/usr/bin/env python3
"""Characterize false and structural-positive behavior of the G0 analyzer.

This campaign deliberately does not tune the ESKF or promote the analyzer to
runtime logic.  It uses a frozen stationary trajectory with generated sensor
noise to estimate the analyzer's structural-ready false-positive rate, then
uses zero-measurement-noise excited trajectories only as geometry controls.
The latter are not noisy true-positive or flight-readiness evidence.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_rate_sensitivity as sensitivity  # noqa: E402


STATIC_MOTION = "bias_cv_static_hold"
POSITIVE_MOTIONS = (
    "bias_cv_takeoff_box_land",
    "bias_cv_yaw_quadrant_hover",
)


def frozen_protocol(motion: str) -> dict[str, object]:
    protocol = json.loads(sensitivity.PROTOCOL.read_text(encoding="utf-8"))
    duration = 32.0 if motion == STATIC_MOTION else {
        "bias_cv_takeoff_box_land": 38.0,
        "bias_cv_yaw_quadrant_hover": 40.0,
    }[motion]
    protocol["trajectory"] = {
        "motion": motion,
        "duration_s": duration,
        "seed_start": 0,
    }
    return protocol


def run_campaign(
    runner: Path,
    work_dir: Path,
    rates: list[float],
    seed_start: int,
    seed_count: int,
    keep_work: bool,
    jobs: int,
) -> dict[str, object]:
    def run_static(seed: int, rate: float) -> dict[str, object]:
        case = sensitivity.run_case(
                "continuous_density",
                rate,
                runner,
                work_dir / "static-null" / f"seed-{seed:05d}",
                frozen_protocol(STATIC_MOTION),
                seed,
                keep_work,
                static_prefix_end_s=32.0,
            )
        analyzer = case["analyzer"]
        assert isinstance(analyzer, dict)
        return {
            "seed": seed,
            "rate_hz": rate,
            "structural_ready": bool(analyzer["structural_ready_window_count"]),
            "full_rank": bool(analyzer["effective_full_rank_window_count"]),
            "maximum_effective_rank": analyzer["maximum_effective_rank"],
            "final_effective_rank": analyzer["final_effective_rank"],
            "estimator_healthy_ratio": case["estimator_healthy_ratio"],
            "navigation_recoveries": case["estimator_max_navigation_recovery_count"],
        }

    static_inputs = [
        (seed, rate)
        for seed in range(seed_start, seed_start + seed_count)
        for rate in rates
    ]
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        static_cases = list(executor.map(lambda item: run_static(*item), static_inputs))
    static_cases.sort(key=lambda case: (int(case["seed"]), float(case["rate_hz"])))

    def run_positive(motion: str, rate: float) -> dict[str, object]:
        case = sensitivity.run_case(
                "zero_all_measurement_noise",
                rate,
                runner,
                work_dir / "zero-noise-positive" / motion,
                frozen_protocol(motion),
                seed_start,
                keep_work,
                static_prefix_end_s=3.0,
            )
        analyzer = case["analyzer"]
        assert isinstance(analyzer, dict)
        return {
            "motion": motion,
            "rate_hz": rate,
            "structural_ready": bool(analyzer["structural_ready_window_count"]),
            "full_rank": bool(analyzer["effective_full_rank_window_count"]),
            "maximum_effective_rank": analyzer["maximum_effective_rank"],
            "final_effective_rank": analyzer["final_effective_rank"],
            "estimator_healthy_ratio": case["estimator_healthy_ratio"],
            "navigation_recoveries": case["estimator_max_navigation_recovery_count"],
        }

    positive_inputs = [(motion, rate) for motion in POSITIVE_MOTIONS for rate in rates]
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        positive_cases = list(executor.map(lambda item: run_positive(*item), positive_inputs))
    positive_cases.sort(key=lambda case: (str(case["motion"]), float(case["rate_hz"])))

    static_total = len(static_cases)
    static_ready = sum(bool(case["structural_ready"]) for case in static_cases)
    static_full_rank = sum(bool(case["full_rank"]) for case in static_cases)
    upper_95 = 1.0 - math.pow(0.05, 1.0 / static_total) if static_total else None
    return {
        "schema_version": 1,
        "status": "diagnostic_only_not_estimator_gate",
        "study_id": "g0-bias-observability-gate-characterization-v1",
        "null_definition": "32 s stationary trajectory, causal stationarity, 10 Hz GNSS P/V, continuous-density generated sensor noise",
        "positive_control_definition": "takeoff-box and yaw-quadrant trajectories with all generated measurement noise set to zero",
        "rates_hz": rates,
        "seed_range": [seed_start, seed_start + seed_count - 1],
        "static_null": {
            "cases": static_cases,
            "total_cases": static_total,
            "structural_ready_cases": static_ready,
            "effective_full_rank_cases": static_full_rank,
            "structural_ready_rate": static_ready / static_total if static_total else None,
            "effective_full_rank_rate": static_full_rank / static_total if static_total else None,
            "one_sided_95pct_upper_bound_if_zero": upper_95 if static_ready == 0 else None,
        },
        "zero_noise_positive_controls": {
            "cases": positive_cases,
            "structural_ready_cases": sum(bool(case["structural_ready"]) for case in positive_cases),
            "effective_full_rank_cases": sum(bool(case["full_rank"]) for case in positive_cases),
        },
        "interpretation": [
            "A stationary false-positive result is evidence against the analyzer's current gate, not proof of a safe runtime gate.",
            "Zero-noise structural positives nominate excitation trajectories only; they do not establish noisy true-positive power or bias convergence.",
            "The analyzer remains invalid as an estimator correction trigger until process/preintegration covariance, reset semantics, and persistence are modeled and independently calibrated.",
        ],
        "protocol_sha256": sensitivity.sha256(sensitivity.PROTOCOL),
        "generator_sha256": sensitivity.sha256(sensitivity.GENERATOR),
        "analyzer_sha256": sensitivity.sha256(sensitivity.ANALYZER),
        "runner_sha256": sensitivity.sha256(runner),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("validation/public/g0_gate_characterization.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/g0-gate-characterization"))
    parser.add_argument("--rates", default="50,100,200,400")
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=16)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--keep-work", action="store_true")
    args = parser.parse_args()
    rates = [float(value) for value in args.rates.split(",")]
    if not rates or any(rate <= 0.0 for rate in rates) or len(set(rates)) != len(rates):
        parser.error("--rates must contain unique positive values")
    if args.seed_count <= 0:
        parser.error("--seed-count must be positive")
    if args.jobs <= 0:
        parser.error("--jobs must be positive")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    output = args.out if args.out.is_absolute() else ROOT / args.out
    summary = run_campaign(
        runner, work_dir, rates, args.seed_start, args.seed_count, args.keep_work, args.jobs
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
