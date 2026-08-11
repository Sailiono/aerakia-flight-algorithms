#!/usr/bin/env python3
"""Run and summarize the non-promoting v8 diagnostic-lane screen.

The screen reuses the frozen v7 synthetic residual oracle only as a host-side
NIS provider.  It evaluates the existing v8 policy comparator plus its new
orthogonal diagnostic snapshot.  The full trace is intentionally kept under
``build/``; the compact summary is the only public evidence candidate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
VALIDATION = ROOT / "validation"
PROTOCOL_PATH = VALIDATION / "airspeed_wind_residual_persistence_v8_diagnostic_lane_protocol.json"
IMPLEMENTATION_PATH = VALIDATION / "diagnose_airspeed_wind_residual_persistence_v8.py"

if str(VALIDATION) not in sys.path:
    sys.path.insert(0, str(VALIDATION))
import diagnose_airspeed_wind_residual_persistence_v8 as v8  # noqa: E402


PERSISTENT_CASES = (
    "persistent_tas_offset_positive",
    "persistent_tas_offset_negative",
    "persistent_tas_offset_positive_bounded_jitter",
    "persistent_tas_offset_negative_bounded_jitter",
)
NOMINAL_CASES = ("nominal", "nominal_bounded_jitter")
STRUCTURAL_CASES = ("gap_then_tas_offset_pulse_1p0s",)
SCREEN_CASES = (
    "nominal",
    "nominal_high_noise_calibrated",
    "nominal_bounded_jitter",
    "persistent_tas_offset_positive",
    "persistent_tas_offset_negative",
    "persistent_tas_offset_positive_bounded_jitter",
    "persistent_tas_offset_negative_bounded_jitter",
    "persistent_tas_scale_positive",
    "persistent_tas_scale_negative",
    "persistent_horizontal_wind_step",
    "persistent_vertical_wind",
    "tas_offset_pulse_0p25s",
    "tas_offset_pulse_0p5s",
    "tas_offset_pulse_1p0s",
    "tas_offset_pulse_1p5s",
    "tas_offset_pulse_2p0s",
    "gap_then_tas_offset_pulse_1p0s",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _candidate_summary(screen: dict[str, object], ages: Sequence[float]) -> dict[str, object]:
    records = screen["records"]
    assert isinstance(records, list)
    by_key = {
        (int(record["seed"]), str(record["case"])): record for record in records
    }
    seeds = list(range(
        int(screen["manifest"]["seed_start"]),
        int(screen["manifest"]["seed_end"]) + 1,
    ))
    summary: dict[str, object] = {}
    for age in ages:
        age_key = f"{float(age):g}s"
        per_policy: dict[str, object] = {}
        for policy in v8.PolicyKind:
            def result_for(seed: int, case: str) -> dict[str, object]:
                record = by_key[(seed, case)]
                return record["candidate_policies"][age_key][policy.value]

            diagnostic_results = [
                result_for(seed, case)
                for seed in seeds
                for case in NOMINAL_CASES + PERSISTENT_CASES + STRUCTURAL_CASES
            ]
            all_results = [result_for(seed, case) for seed in seeds for case in SCREEN_CASES]
            persistent_results = [
                result_for(seed, case) for seed in seeds for case in PERSISTENT_CASES
            ]
            nominal_results = [result_for(seed, case) for seed in seeds for case in NOMINAL_CASES]
            structural_results = [
                result_for(seed, case) for seed in seeds for case in STRUCTURAL_CASES
            ]

            diagnostic_snapshots = [
                result for result in diagnostic_results if result["diagnostic_snapshot"] is not None
            ]
            persistent_snapshots = [
                result for result in persistent_results if result["diagnostic_snapshot"] is not None
            ]
            clean_persistent_snapshots = []
            ambiguous_persistent_snapshots = []
            for seed in seeds:
                for case in PERSISTENT_CASES:
                    result = result_for(seed, case)
                    snapshot = result["diagnostic_snapshot"]
                    if snapshot is None:
                        continue
                    injection = int(by_key[(seed, case)]["injection_start_source_timestamp_us"])
                    if int(snapshot["episode_start_source_timestamp_us"]) < injection:
                        ambiguous_persistent_snapshots.append((seed, case))
                    else:
                        clean_persistent_snapshots.append((seed, case))

            per_policy[policy.value] = {
                "diagnostic_snapshot_count_all_cases": sum(
                    result["diagnostic_snapshot"] is not None for result in all_results
                ),
                "diagnostic_snapshot_trials_all_cases": len(all_results),
                "diagnostic_snapshot_by_case": {
                    case: sum(
                        result_for(seed, case)["diagnostic_snapshot"] is not None
                        for seed in seeds
                    )
                    for case in SCREEN_CASES
                },
                "diagnostic_snapshot_count": len(diagnostic_snapshots),
                "diagnostic_snapshot_trials": len(diagnostic_results),
                "diagnostic_snapshot_rate": (
                    len(diagnostic_snapshots) / len(diagnostic_results)
                    if diagnostic_results else 0.0
                ),
                "diagnostic_on_nominal_count": sum(
                    result["diagnostic_snapshot"] is not None for result in nominal_results
                ),
                "diagnostic_on_structural_gap_count": sum(
                    result["diagnostic_snapshot"] is not None for result in structural_results
                ),
                "persistent_diagnostic_count": len(persistent_snapshots),
                "persistent_diagnostic_trials": len(persistent_results),
                "persistent_clean_diagnostic_count": len(clean_persistent_snapshots),
                "persistent_ambiguous_diagnostic_count": len(ambiguous_persistent_snapshots),
                "persistent_clean_diagnostic_members": [
                    {"seed": seed, "case": case}
                    for seed, case in clean_persistent_snapshots
                ],
                "persistent_ambiguous_diagnostic_members": [
                    {"seed": seed, "case": case}
                    for seed, case in ambiguous_persistent_snapshots
                ],
                "control_latch_count": sum(result["latched"] for result in diagnostic_results),
                "diagnostic_without_control_count": sum(
                    result["diagnostic_snapshot"] is not None and not result["latched"]
                    for result in diagnostic_results
                ),
            }
        summary[age_key] = per_policy
    return summary


def run_screen(
    *,
    seed_start: int,
    seed_count: int,
    jobs: int,
    ages: Sequence[float],
    full_out: Path,
    summary_out: Path,
) -> dict[str, object]:
    full_out = full_out.resolve()
    summary_out = summary_out.resolve()
    screen = v8.run_fresh_screen(
        seed_start=seed_start,
        seed_count=seed_count,
        jobs=jobs,
        boundary_ages_s=ages,
        case_names=SCREEN_CASES,
    )
    full_payload = json.dumps(screen, indent=2, sort_keys=True) + "\n"
    full_out.parent.mkdir(parents=True, exist_ok=True)
    full_out.write_text(full_payload, encoding="utf-8")
    summary = {
        "schema_version": 1,
        "study_id": "aerakia-v8-diagnostic-lane-screen",
        "status": "completed_non_promoting_diagnostic_screen",
        "execution": {
            "git_commit": _git_commit(),
            "implementation_sha256": _sha256(IMPLEMENTATION_PATH),
            "protocol_sha256": _sha256(PROTOCOL_PATH),
            "full_trace_path": str(full_out.relative_to(ROOT)),
            "full_trace_sha256": _sha256(full_out),
            "seed_start": seed_start,
            "seed_count": seed_count,
            "seed_end": seed_start + seed_count - 1,
            "boundary_ages_s": [float(age) for age in ages],
            "jobs": jobs,
        },
        "source_screen_manifest": screen["manifest"],
        "diagnostic_summary": _candidate_summary(screen, ages),
        "interpretation": [
            "A diagnostic snapshot is not a fault classification or control qualification.",
            "Nominal and structural-gap diagnostic counts are retained as nuisance/continuity evidence, not hidden.",
            "Persistent cases whose episode starts before the offline injection are ambiguous and are not credited as clean.",
            "No policy, age, threshold, estimator state, FCOne authority, or flight-control action is promoted.",
        ],
    }
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed-start", type=int, default=74301)
    parser.add_argument("--seed-count", type=int, default=64)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--ages", default="1,2,3")
    parser.add_argument(
        "--full-out",
        type=Path,
        default=ROOT / "build" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64.json",
    )
    parser.add_argument(
        "--summary-out",
        type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64_summary.json",
    )
    args = parser.parse_args(argv)
    ages = tuple(float(item.strip()) for item in str(args.ages).split(","))
    summary = run_screen(
        seed_start=args.seed_start,
        seed_count=args.seed_count,
        jobs=args.jobs,
        ages=ages,
        full_out=args.full_out,
        summary_out=args.summary_out,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
