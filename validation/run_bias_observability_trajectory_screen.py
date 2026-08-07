#!/usr/bin/env python3
"""Screen frozen G0 trajectories with every generated measurement noise set to zero.

The screen answers only a structural simulation question: whether a prescribed
motion can produce five-dimensional local information before noise is allowed
to perturb the estimated linearization.  It is not an estimator tuning,
flight-policy, or correction-promotion tool.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation" / "tools"))

import generate_synthetic_imu as generator  # noqa: E402
import run_bias_observability_rate_sensitivity as sensitivity  # noqa: E402


DEFAULT_MOTIONS = (
    "bias_cv_hover_axis_pulses",
    "bias_cv_takeoff_box_land",
    "bias_cv_yaw_quadrant_hover",
    "bias_cv_early_transition_s_curve",
)


def current_commit() -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("validation/public/g0_trajectory_screen.json"))
    parser.add_argument("--work-dir", type=Path, default=Path("build/g0-trajectory-screen"))
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--keep-work", action="store_true")
    args = parser.parse_args()
    if args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    protocol = json.loads(sensitivity.PROTOCOL.read_text(encoding="utf-8"))
    output = args.out if args.out.is_absolute() else ROOT / args.out
    work_dir = args.work_dir if args.work_dir.is_absolute() else ROOT / args.work_dir
    cases: list[dict[str, object]] = []
    for motion in DEFAULT_MOTIONS:
        duration_s = generator.BIAS_OBSERVABILITY_TRAJECTORY_DURATIONS_S[motion]
        case_protocol = {
            **protocol,
            "trajectory": {
                "motion": motion,
                "duration_s": duration_s,
                "seed_start": args.seed,
            },
        }
        case = sensitivity.run_case(
            "zero_all_measurement_noise",
            args.rate_hz,
            runner,
            work_dir / motion,
            case_protocol,
            args.seed,
            args.keep_work,
            static_prefix_end_s=3.0,
        )
        cases.append({"motion": motion, "duration_s": duration_s, **case})
    summary = {
        "schema_version": 1,
        "status": "diagnostic_only_not_estimator_gate",
        "study_id": "g0-bias-observability-trajectory-screen-v1",
        "dataset": "G0 zero-noise trajectory structural screen",
        "sequence": "four frozen trajectories; 100 Hz; all generated measurement noise zero",
        "samples": len(cases),
        "rate_hz": args.rate_hz,
        "profile": "zero_all_measurement_noise",
        "git_commit": current_commit(),
        "protocol_sha256": sensitivity.sha256(sensitivity.PROTOCOL),
        "generator_sha256": sensitivity.sha256(sensitivity.GENERATOR),
        "analyzer_sha256": sensitivity.sha256(sensitivity.ANALYZER),
        "runner_sha256": sensitivity.sha256(runner),
        "cases": cases,
        "limitations": [
            "Zero generated measurement noise is a structural control, not a physical sensor model.",
            "The analyzer omits process noise, preintegration covariance, aiding correlations, and reset Jacobians.",
            "A full-rank result would only nominate a future test trajectory; it would not authorize a correction.",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
