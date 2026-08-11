#!/usr/bin/env python3
"""Paired replay of the v8 diagnostic-provenance fix on an existing screen.

The original 64-family full trace contains the causal input stream in each
probe event.  This tool replays those exact source/arrival/NIS streams through
the current probe, verifies that every primary policy result is unchanged,
and measures only the diagnostic-provenance delta.  The opened screen remains
development evidence; this is not a fresh train, tune, or holdout.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from collections import defaultdict
from dataclasses import replace
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
VALIDATION = ROOT / "validation"
if str(VALIDATION) not in sys.path:
    sys.path.insert(0, str(VALIDATION))

import audit_v8_diagnostic_lane_screen as audit  # noqa: E402
import diagnose_airspeed_wind_residual_persistence_v8 as v8  # noqa: E402


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def causal_inputs_from_record(record: dict[str, object]) -> tuple[v8.ProbeInput, ...]:
    """Recover the screen's causal runtime input from one retained trace."""

    source = record["candidate_policies"]["1s"][v8.PolicyKind.RECENT_BOUNDARY.value]
    events = source["trace"]
    return tuple(
        v8.ProbeInput(
            source_timestamp_us=int(event["source_timestamp_us"]),
            arrival_timestamp_us=int(event["arrival_timestamp_us"]),
            nis=None if event["nis"] is None else float(event["nis"]),
            source_valid=event["nis"] is not None,
            source_epoch=0,
        )
        for event in events
    )


def replay_record(
    record: dict[str, object], *, ages_s: Sequence[float], base_config: v8.ProbeConfig
) -> dict[str, dict[str, v8.ProbeResult]]:
    inputs = causal_inputs_from_record(record)
    results: dict[str, dict[str, v8.ProbeResult]] = {}
    for age_s in ages_s:
        age_key = f"{float(age_s):g}s"
        config = replace(base_config, recent_quiet_boundary_max_age_s=float(age_s))
        results[age_key] = {
            policy.value: v8.run_probe(policy, inputs, config) for policy in v8.PolicyKind
        }
    return results


def primary_comparator_matches(
    old: dict[str, object], new: v8.ProbeResult
) -> bool:
    return (
        bool(old["latched"]) == new.latched
        and old["final_state"] == new.final_state
        and old["latch_source_timestamp_us"] == new.latch_source_timestamp_us
        and old["episode_start_source_timestamp_us"]
        == new.episode_start_source_timestamp_us
        and old["boundary_source_timestamp_us"] == new.boundary_source_timestamp_us
        and old["reset_count"] == new.reset_count
        and old["retry_aborts_used"] == new.retry_aborts_used
        and old["partial_retry_aborts_used"] == new.partial_retry_aborts_used
        and old["partial_retry_exhausted"] == new.partial_retry_exhausted
    )


def run_replay(
    *,
    full_trace: Path,
    screen_summary_path: Path,
    old_audit_path: Path,
    output_path: Path,
) -> dict[str, object]:
    full_trace = full_trace.resolve()
    screen_summary_path = screen_summary_path.resolve()
    old_audit_path = old_audit_path.resolve()
    output_path = output_path.resolve()
    screen_summary = json.loads(screen_summary_path.read_text(encoding="utf-8"))
    old_audit = json.loads(old_audit_path.read_text(encoding="utf-8"))
    actual_full_sha = _sha256(full_trace)
    if actual_full_sha != screen_summary["execution"]["full_trace_sha256"]:
        raise ValueError("full trace SHA-256 mismatch")

    v7_protocol = json.loads(
        (VALIDATION / "airspeed_wind_residual_persistence_protocol_v7.json").read_text(
            encoding="utf-8"
        )
    )
    base_config = v8._config_from_v7_protocol(v7_protocol)
    ages_s = tuple(float(age) for age in screen_summary["execution"]["boundary_ages_s"])
    ages = tuple(f"{age:g}s" for age in ages_s)
    policies = tuple(policy.value for policy in v8.PolicyKind)

    persistent_members: dict[
        tuple[str, str], list[tuple[int, str, dict[str, bool]]]
    ] = defaultdict(list)
    diagnostic_by_case: dict[tuple[str, str, str], int] = defaultdict(int)
    primary_checks = 0
    primary_mismatches: list[dict[str, object]] = []
    record_count = 0
    for record in audit.iter_screen_records(full_trace):
        record_count += 1
        replayed = replay_record(record, ages_s=ages_s, base_config=base_config)
        case = str(record["case"])
        seed = int(record["seed"])
        injection_us = int(record["injection_start_source_timestamp_us"])
        for age in ages:
            for policy in policies:
                old_result = record["candidate_policies"][age][policy]
                new_result = replayed[age][policy]
                primary_checks += 1
                if not primary_comparator_matches(old_result, new_result):
                    primary_mismatches.append(
                        {"seed": seed, "case": case, "age": age, "policy": policy}
                    )
                if new_result.diagnostic_snapshot is not None:
                    diagnostic_by_case[(age, policy, case)] += 1
                if case in audit.PERSISTENT_CASES:
                    payload = v8._result_payload(new_result)
                    payload["source_detection_delay_s"] = old_result[
                        "source_detection_delay_s"
                    ]
                    flags = audit.classify_member(
                        result=payload,
                        injection_source_us=injection_us,
                        maximum_delay_s=6.0,
                    )
                    persistent_members[(age, policy)].append((seed, case, flags))

    expected_records = int(screen_summary["source_screen_manifest"]["record_count"])
    if record_count != expected_records:
        raise ValueError(
            f"record count mismatch: replayed {record_count}, expected {expected_records}"
        )
    summary = {
        age: {
            policy: audit.summarize_members(persistent_members[(age, policy)])
            for policy in policies
        }
        for age in ages
    }
    deltas = {}
    for age in ages:
        deltas[age] = {}
        for policy in policies:
            before = old_audit["summary"][age][policy]
            after = summary[age][policy]
            deltas[age][policy] = {
                "diagnostic_clean_member_delta": (
                    after["diagnostic_clean_members"]
                    - before["diagnostic_clean_members"]
                ),
                "descriptive_union_clean_member_delta": (
                    after["descriptive_union_clean_members"]
                    - before["descriptive_union_clean_members"]
                ),
                "descriptive_union_clean_family_delta": (
                    after["descriptive_union_clean_family_passes"]
                    - before["descriptive_union_clean_family_passes"]
                ),
            }
    result = {
        "schema_version": 1,
        "study_id": "aerakia-v8-diagnostic-provenance-fix-paired-replay",
        "status": (
            "completed_primary_invariant_paired_replay"
            if not primary_mismatches
            else "failed_primary_invariant_paired_replay"
        ),
        "execution": {
            "replay_git_commit": _git_commit(),
            "source_screen_git_commit": screen_summary["execution"]["git_commit"],
            "full_trace_path": str(full_trace.relative_to(ROOT)),
            "full_trace_sha256": actual_full_sha,
            "screen_summary_sha256": _sha256(screen_summary_path),
            "old_attribution_audit_sha256": _sha256(old_audit_path),
            "record_count": record_count,
            "primary_comparator_checks": primary_checks,
            "primary_comparator_mismatch_count": len(primary_mismatches),
            "primary_comparator_mismatches": primary_mismatches,
        },
        "summary": summary,
        "delta_from_initial_diagnostic_lane": deltas,
        "diagnostic_snapshot_by_case": {
            age: {
                policy: {
                    case: diagnostic_by_case[(age, policy, case)]
                    for case in screen_summary["source_screen_manifest"]["case_names"]
                }
                for policy in policies
            }
            for age in ages
        },
        "limitations": [
            "This replay reuses an already opened development screen and is not fresh validation evidence.",
            "The source trace uses source_epoch zero and maps a missing NIS to source_valid false, matching the frozen screen generator.",
            "Primary policy output must remain identical; only diagnostic provenance is allowed to change.",
            "Descriptive union remains diagnostic/logging coverage and is not control authority or physical fault classification."
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--full-trace",
        type=Path,
        default=ROOT / "build" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64.json",
    )
    parser.add_argument(
        "--screen-summary",
        type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64_summary.json",
    )
    parser.add_argument(
        "--old-audit",
        type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64_audit.json",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v8_diagnostic_provenance_fix_paired_replay.json",
    )
    args = parser.parse_args(argv)
    result = run_replay(
        full_trace=args.full_trace,
        screen_summary_path=args.screen_summary,
        old_audit_path=args.old_audit,
        output_path=args.out,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "completed_primary_invariant_paired_replay" else 1


if __name__ == "__main__":
    raise SystemExit(main())
