#!/usr/bin/env python3
"""Validate a development-only analyzer score threshold on new seeds.

The baseline campaign supplies a calibration split (seeds 0--7) and an
internal validation split (seeds 8--15). This runner evaluates the same
stationary null on a disjoint seed range without re-fitting the ESKF. The
candidate score is the maximum minimum eigenvalue observed in a causal window;
the threshold is three times the calibration p99. The result is explicitly
diagnostic and is not a runtime or flight qualification gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_rate_sensitivity as sensitivity  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values: list[float], probability: float) -> float:
    if not values:
        raise ValueError("cannot compute a percentile of an empty set")
    ordered = sorted(float(value) for value in values)
    location = probability * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    fraction = location - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def static_protocol() -> dict[str, object]:
    protocol = json.loads(sensitivity.PROTOCOL.read_text(encoding="utf-8"))
    protocol["trajectory"] = {
        "motion": "bias_cv_static_hold",
        "duration_s": 32.0,
        "seed_start": 0,
    }
    return protocol


def run_validation(
    runner: Path,
    work_dir: Path,
    rates: list[float],
    seed_start: int,
    seed_count: int,
    threshold: float,
    jobs: int,
) -> list[dict[str, object]]:
    def one(seed: int, rate: float) -> dict[str, object]:
        case = sensitivity.run_case(
            "continuous_density",
            rate,
            runner,
            work_dir / f"seed-{seed:05d}",
            static_protocol(),
            seed,
            False,
            static_prefix_end_s=32.0,
        )
        analyzer = case["analyzer"]
        assert isinstance(analyzer, dict)
        score = float(analyzer["maximum_minimum_eigenvalue"])
        return {
            "seed": seed,
            "rate_hz": rate,
            "score_maximum_minimum_eigenvalue": score,
            "score_pass": score >= threshold,
            "current_structural_ready": bool(analyzer["structural_ready_window_count"]),
            "current_full_rank": bool(analyzer["effective_full_rank_window_count"]),
            "estimator_healthy_ratio": case["estimator_healthy_ratio"],
            "navigation_recoveries": case["estimator_max_navigation_recovery_count"],
        }

    inputs = [
        (seed, rate)
        for seed in range(seed_start, seed_start + seed_count)
        for rate in rates
    ]
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        cases = list(executor.map(lambda item: one(*item), inputs))
    return sorted(cases, key=lambda case: (int(case["seed"]), float(case["rate_hz"])))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument(
        "--baseline", type=Path,
        default=Path("validation/public/g0_gate_characterization.json"),
    )
    parser.add_argument(
        "--out", type=Path,
        default=Path("validation/public/g0_gate_null_split_validation.json"),
    )
    parser.add_argument("--work-dir", type=Path, default=Path("build/g0-null-split-validation"))
    parser.add_argument("--rates", default="50,100,200,400")
    parser.add_argument("--calibration-seed-stop", type=int, default=8)
    parser.add_argument("--validation-seed-start", type=int, default=100)
    parser.add_argument("--validation-seed-count", type=int, default=16)
    parser.add_argument("--p99-multiplier", type=float, default=3.0)
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    rates = [float(value) for value in args.rates.split(",")]
    if not rates or any(rate <= 0.0 for rate in rates) or len(set(rates)) != len(rates):
        parser.error("--rates must contain unique positive values")
    if args.calibration_seed_stop <= 0 or args.validation_seed_count <= 0:
        parser.error("seed ranges must be positive")
    if args.validation_seed_start < args.calibration_seed_stop:
        parser.error("validation seeds must be disjoint from calibration seeds")
    if args.p99_multiplier <= 0.0 or args.jobs <= 0:
        parser.error("p99 multiplier and jobs must be positive")
    baseline = args.baseline if args.baseline.is_absolute() else ROOT / args.baseline
    runner = args.runner.resolve()
    if not baseline.is_file() or not runner.is_file():
        parser.error("baseline and runner must exist")
    baseline_data = json.loads(baseline.read_text(encoding="utf-8"))
    calibration_cases = [
        item for item in baseline_data["static_null"]["cases"]
        if int(item["seed"]) < args.calibration_seed_stop
    ]
    scores = [float(item["maximum_minimum_eigenvalue"]) for item in calibration_cases]
    calibration_p99 = percentile(scores, 0.99)
    threshold = args.p99_multiplier * calibration_p99
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    cases = run_validation(
        runner, work_dir, rates, args.validation_seed_start, args.validation_seed_count,
        threshold, args.jobs,
    )
    summary = {
        "schema_version": 1,
        "status": "diagnostic_candidate_threshold_not_runtime_gate",
        "study_id": "g0-bias-observability-null-split-validation-v1",
        "score_definition": "maximum minimum eigenvalue over causal analyzer windows",
        "threshold_definition": "p99 of calibration stationary-null scores multiplied by the declared multiplier",
        "p99_multiplier": args.p99_multiplier,
        "calibration": {
            "source_path": str(baseline.relative_to(ROOT)),
            "source_sha256": sha256(baseline),
            "seed_stop_exclusive": args.calibration_seed_stop,
            "case_count": len(calibration_cases),
            "p99_score": calibration_p99,
            "candidate_threshold": threshold,
        },
        "validation": {
            "seed_start": args.validation_seed_start,
            "seed_count": args.validation_seed_count,
            "case_count": len(cases),
            "candidate_score_passes": sum(bool(case["score_pass"]) for case in cases),
            "candidate_false_positive_rate": sum(bool(case["score_pass"]) for case in cases) / len(cases),
            "current_structural_ready_rate": sum(bool(case["current_structural_ready"]) for case in cases) / len(cases),
            "cases": cases,
        },
        "limitations": [
            "The threshold is calibrated on synthetic stationary data and is not valid for FCOne sensors until their noise, timing, and covariance contract is measured.",
            "A clean stationary null does not establish a noisy true-positive rate or bias convergence.",
            "This score is not connected to ESKF correction, supervisor qualification, or flight-control authority.",
        ],
        "protocol_sha256": sensitivity.sha256(sensitivity.PROTOCOL),
        "generator_sha256": sensitivity.sha256(sensitivity.GENERATOR),
        "analyzer_sha256": sensitivity.sha256(sensitivity.ANALYZER),
        "runner_sha256": sha256(runner),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    }
    output = args.out if args.out.is_absolute() else ROOT / args.out
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
