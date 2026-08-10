#!/usr/bin/env python3
"""Score noisy maneuver candidates against a pre-calibrated G0 null threshold.

This is the companion to the stationary null split. It checks whether the
candidate minimum-eigenvalue score survives generated sensor noise on two
maneuvers that were structural positives in the zero-noise screen. It does not
measure bias convergence and does not authorize a correction.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation" / "tools"))

import generate_synthetic_imu as generator  # noqa: E402
import run_bias_observability_rate_sensitivity as sensitivity  # noqa: E402


MOTIONS = (
    "bias_cv_takeoff_box_land",
    "bias_cv_yaw_quadrant_hover",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def protocol_for(motion: str) -> dict[str, object]:
    protocol = json.loads(sensitivity.PROTOCOL.read_text(encoding="utf-8"))
    protocol["trajectory"] = {
        "motion": motion,
        "duration_s": generator.BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S[motion],
        "seed_start": 0,
    }
    return protocol


def run_campaign(
    runner: Path,
    work_dir: Path,
    baseline: Path,
    rates: list[float],
    seed_start: int,
    seed_count: int,
    jobs: int,
) -> dict[str, object]:
    baseline_data = json.loads(baseline.read_text(encoding="utf-8"))
    calibration = [
        item for item in baseline_data["static_null"]["cases"]
        if int(item["seed"]) < 8
    ]
    ordered = sorted(float(item["maximum_minimum_eigenvalue"]) for item in calibration)
    location = 0.99 * (len(ordered) - 1)
    low = int(location)
    high = min(low + 1, len(ordered) - 1)
    p99 = ordered[low] + (ordered[high] - ordered[low]) * (location - low)
    threshold = 3.0 * p99

    def one(motion: str, seed: int, rate: float) -> dict[str, object]:
        case = sensitivity.run_case(
            "continuous_density",
            rate,
            runner,
            work_dir / motion / f"seed-{seed:05d}",
            protocol_for(motion),
            seed,
            False,
            static_prefix_end_s=3.0,
        )
        analyzer = case["analyzer"]
        assert isinstance(analyzer, dict)
        score = float(analyzer["maximum_minimum_eigenvalue"])
        return {
            "motion": motion,
            "seed": seed,
            "rate_hz": rate,
            "score_maximum_minimum_eigenvalue": score,
            "score_pass": score >= threshold,
            "current_structural_ready": bool(analyzer["structural_ready_window_count"]),
            "current_full_rank": bool(analyzer["effective_full_rank_window_count"]),
            "maximum_effective_rank": analyzer["maximum_effective_rank"],
            "estimator_healthy_ratio": case["estimator_healthy_ratio"],
            "navigation_recoveries": case["estimator_max_navigation_recovery_count"],
        }

    inputs = [
        (motion, seed, rate)
        for motion in MOTIONS
        for seed in range(seed_start, seed_start + seed_count)
        for rate in rates
    ]
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        cases = list(executor.map(lambda item: one(*item), inputs))
    cases.sort(key=lambda case: (str(case["motion"]), int(case["seed"]), float(case["rate_hz"])))
    by_motion: dict[str, dict[str, object]] = {}
    for motion in MOTIONS:
        selected = [case for case in cases if case["motion"] == motion]
        by_motion[motion] = {
            "case_count": len(selected),
            "candidate_score_passes": sum(bool(case["score_pass"]) for case in selected),
            "candidate_score_pass_rate": sum(bool(case["score_pass"]) for case in selected) / len(selected),
            "current_structural_ready_rate": sum(bool(case["current_structural_ready"]) for case in selected) / len(selected),
            "minimum_score": min(float(case["score_maximum_minimum_eigenvalue"]) for case in selected),
        }
    return {
        "schema_version": 1,
        "status": "diagnostic_candidate_threshold_not_runtime_gate",
        "study_id": "g0-bias-observability-excitation-score-campaign-v1",
        "score_definition": "maximum minimum eigenvalue over causal analyzer windows",
        "threshold_definition": "three times the p99 score from stationary calibration seeds 0--7",
        "candidate_threshold": threshold,
        "calibration": {
            "source_path": str(baseline.relative_to(ROOT)),
            "source_sha256": sha256(baseline),
            "p99_score": p99,
        },
        "dataset": "two frozen maneuver candidates with continuous-density generated sensor noise",
        "rates_hz": rates,
        "seed_range": [seed_start, seed_start + seed_count - 1],
        "motions": by_motion,
        "cases": cases,
        "limitations": [
            "Generated maneuvers are not physical flight evidence and do not prove online bias convergence.",
            "The threshold is calibrated on synthetic stationary noise and is not valid for FCOne until its sensor contract is measured.",
            "The score is not connected to ESKF correction, supervisor qualification, or flight-control authority.",
        ],
        "protocol_sha256": sensitivity.sha256(sensitivity.PROTOCOL),
        "generator_sha256": sensitivity.sha256(sensitivity.GENERATOR),
        "analyzer_sha256": sensitivity.sha256(sensitivity.ANALYZER),
        "runner_sha256": sha256(runner),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, default=Path("validation/public/g0_gate_characterization.json"))
    parser.add_argument("--out", type=Path, default=Path("validation/public/g0_excitation_score_campaign.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/g0-excitation-score"))
    parser.add_argument("--rates", default="50,100,200,400")
    parser.add_argument("--seed-start", type=int, default=100)
    parser.add_argument("--seed-count", type=int, default=16)
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    rates = [float(value) for value in args.rates.split(",")]
    if not rates or any(rate <= 0.0 for rate in rates) or len(set(rates)) != len(rates):
        parser.error("--rates must contain unique positive values")
    if args.seed_count <= 0 or args.jobs <= 0:
        parser.error("seed-count and jobs must be positive")
    baseline = args.baseline if args.baseline.is_absolute() else ROOT / args.baseline
    runner = args.runner.resolve()
    if not baseline.is_file() or not runner.is_file():
        parser.error("baseline and runner must exist")
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    result = run_campaign(runner, work_dir, baseline, rates, args.seed_start, args.seed_count, args.jobs)
    output = args.out if args.out.is_absolute() else ROOT / args.out
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
