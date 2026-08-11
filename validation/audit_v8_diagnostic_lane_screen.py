#!/usr/bin/env python3
"""Stream-audit a v8 diagnostic-lane screen without loading its full trace.

The full 64-family trace is intentionally large.  This tool verifies its hash,
streams the ``records`` array one object at a time, and separates raw control
latches, clean post-injection control attribution, pre-existing ambiguity,
diagnostic-only persistence, and their descriptive union.  The union is never
reported as control qualification.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Iterator, Sequence

ROOT = Path(__file__).resolve().parents[1]
PERSISTENT_CASES = (
    "persistent_tas_offset_positive",
    "persistent_tas_offset_negative",
    "persistent_tas_offset_positive_bounded_jitter",
    "persistent_tas_offset_negative_bounded_jitter",
)
NOMINAL_DIAGNOSTIC_CASES = (
    "nominal",
    "nominal_high_noise_calibrated",
    "nominal_bounded_jitter",
)
STRUCTURAL_GAP_CASE = "gap_then_tas_offset_pulse_1p0s"
PULSE_1P5_CASE = "tas_offset_pulse_1p5s"
PULSE_2P0_CASE = "tas_offset_pulse_2p0s"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def binomial_cdf(events: int, trials: int, probability: float) -> float:
    if events >= trials or probability == 0.0:
        return 1.0
    if probability == 1.0:
        return 0.0
    return float(
        sum(
            math.comb(trials, index)
            * math.pow(probability, index)
            * math.pow(1.0 - probability, trials - index)
            for index in range(events + 1)
        )
    )


def clopper_pearson_upper(events: int, trials: int, confidence: float) -> float:
    if trials <= 0 or not 0 <= events <= trials or not 0.0 < confidence < 1.0:
        raise ValueError("invalid Clopper-Pearson upper-bound arguments")
    if events == trials:
        return 1.0
    target = 1.0 - confidence
    low = events / trials
    high = 1.0
    for _ in range(100):
        middle = 0.5 * (low + high)
        if binomial_cdf(events, trials, middle) > target:
            low = middle
        else:
            high = middle
    return high


def git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def iter_screen_records(path: Path) -> Iterator[dict[str, object]]:
    """Yield objects from the top-level ``records`` array incrementally."""

    decoder = json.JSONDecoder()
    marker = '"records": ['
    with path.open("r", encoding="utf-8") as stream:
        buffer = ""
        while marker not in buffer:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                raise ValueError("full trace has no records array")
            buffer += chunk
            if marker not in buffer and len(buffer) > len(marker) * 4:
                buffer = buffer[-len(marker) * 2 :]
        buffer = buffer.split(marker, 1)[1]
        while True:
            buffer = buffer.lstrip()
            if buffer.startswith(","):
                buffer = buffer[1:]
                continue
            if buffer.startswith("]"):
                return
            try:
                record, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    raise ValueError("truncated record in full trace")
                buffer += chunk
                continue
            if not isinstance(record, dict):
                raise ValueError("screen record must be an object")
            yield record
            buffer = buffer[end:]


def classify_member(
    *, result: dict[str, object], injection_source_us: int, maximum_delay_s: float
) -> dict[str, bool]:
    raw_control_latch = bool(result["latched"])
    episode_start = result["episode_start_source_timestamp_us"]
    delay = result["source_detection_delay_s"]
    control_preexisting_ambiguous = bool(
        raw_control_latch
        and episode_start is not None
        and int(episode_start) < injection_source_us
    )
    control_clean = bool(
        raw_control_latch
        and episode_start is not None
        and int(episode_start) >= injection_source_us
        and delay is not None
        and float(delay) <= maximum_delay_s
    )
    control_late = bool(
        raw_control_latch
        and episode_start is not None
        and int(episode_start) >= injection_source_us
        and delay is not None
        and float(delay) > maximum_delay_s
    )
    snapshot = result["diagnostic_snapshot"]
    diagnostic_present = snapshot is not None
    diagnostic_preexisting_ambiguous = bool(
        snapshot is not None
        and int(snapshot["episode_start_source_timestamp_us"]) < injection_source_us
    )
    diagnostic_clean = bool(
        snapshot is not None
        and int(snapshot["episode_start_source_timestamp_us"]) >= injection_source_us
    )
    return {
        "raw_control_latch": raw_control_latch,
        "control_clean": control_clean,
        "control_preexisting_ambiguous": control_preexisting_ambiguous,
        "control_late": control_late,
        "diagnostic_present": diagnostic_present,
        "diagnostic_clean": diagnostic_clean,
        "diagnostic_preexisting_ambiguous": diagnostic_preexisting_ambiguous,
        "descriptive_union_clean": control_clean or diagnostic_clean,
    }


def summarize_members(
    members: Iterable[tuple[int, str, dict[str, bool]]]
) -> dict[str, object]:
    rows = list(members)
    by_seed: dict[int, list[tuple[str, dict[str, bool]]]] = defaultdict(list)
    for seed, case, flags in rows:
        by_seed[seed].append((case, flags))
    for seed, values in by_seed.items():
        names = {case for case, _ in values}
        if names != set(PERSISTENT_CASES):
            raise ValueError(f"seed {seed} does not contain the complete persistent family")

    def count(flag: str) -> int:
        return sum(bool(flags[flag]) for _, _, flags in rows)

    control_family_passes = sum(
        all(flags["control_clean"] for _, flags in values)
        for values in by_seed.values()
    )
    union_family_passes = sum(
        all(flags["descriptive_union_clean"] for _, flags in values)
        for values in by_seed.values()
    )
    rescued_families = []
    unresolved_families = []
    for seed, values in sorted(by_seed.items()):
        control_ok = all(flags["control_clean"] for _, flags in values)
        union_ok = all(flags["descriptive_union_clean"] for _, flags in values)
        member_status = [
            {
                "case": case,
                "control_clean": flags["control_clean"],
                "diagnostic_clean": flags["diagnostic_clean"],
                "descriptive_union_clean": flags["descriptive_union_clean"],
            }
            for case, flags in sorted(values)
        ]
        if not control_ok and union_ok:
            rescued_families.append({"seed": seed, "members": member_status})
        elif not union_ok:
            unresolved_families.append({"seed": seed, "members": member_status})

    return {
        "persistent_member_trials": len(rows),
        "persistent_family_trials": len(by_seed),
        "raw_control_latches": count("raw_control_latch"),
        "control_clean_members": count("control_clean"),
        "control_preexisting_ambiguous_members": count(
            "control_preexisting_ambiguous"
        ),
        "control_late_members": count("control_late"),
        "diagnostic_present_members": count("diagnostic_present"),
        "diagnostic_clean_members": count("diagnostic_clean"),
        "diagnostic_preexisting_ambiguous_members": count(
            "diagnostic_preexisting_ambiguous"
        ),
        "descriptive_union_clean_members": count("descriptive_union_clean"),
        "control_clean_family_passes": control_family_passes,
        "descriptive_union_clean_family_passes": union_family_passes,
        "rescued_families": rescued_families,
        "unresolved_families": unresolved_families,
    }


def audit_screen(
    *, full_trace: Path, screen_summary_path: Path, output_path: Path
) -> dict[str, object]:
    full_trace = full_trace.resolve()
    screen_summary_path = screen_summary_path.resolve()
    output_path = output_path.resolve()
    screen_summary = json.loads(screen_summary_path.read_text(encoding="utf-8"))
    declared_sha256 = str(screen_summary["execution"]["full_trace_sha256"])
    actual_sha256 = sha256_file(full_trace)
    if actual_sha256 != declared_sha256:
        raise ValueError("full trace SHA-256 does not match the retained screen summary")

    ages = [f"{float(age):g}s" for age in screen_summary["execution"]["boundary_ages_s"]]
    policies = tuple(
        screen_summary["diagnostic_summary"][ages[0]].keys()
    )
    protocol_path = ROOT / "validation" / "airspeed_wind_residual_persistence_protocol_v7.json"
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    diagnostic_protocol_path = (
        ROOT
        / "validation"
        / "airspeed_wind_residual_persistence_v8_diagnostic_lane_protocol.json"
    )
    diagnostic_protocol = json.loads(
        diagnostic_protocol_path.read_text(encoding="utf-8")
    )
    maximum_delay_by_case = {
        str(case["name"]): float(case.get("maximum_source_detection_delay_s", 6.0))
        for case in protocol["case_matrix"]
    }
    grouped: dict[tuple[str, str], list[tuple[int, str, dict[str, bool]]]] = defaultdict(list)
    diagnostic_presence: dict[tuple[str, str, int, str], bool] = {}
    seeds: set[int] = set()
    record_count = 0
    for record in iter_screen_records(full_trace):
        record_count += 1
        case = str(record["case"])
        seed = int(record["seed"])
        seeds.add(seed)
        injection_us = int(record["injection_start_source_timestamp_us"])
        candidate_policies = record["candidate_policies"]
        for age in ages:
            for policy in policies:
                result = candidate_policies[age][policy]
                diagnostic_presence[(age, policy, seed, case)] = (
                    result["diagnostic_snapshot"] is not None
                )
                if case not in PERSISTENT_CASES:
                    continue
                flags = classify_member(
                    result=result,
                    injection_source_us=injection_us,
                    maximum_delay_s=maximum_delay_by_case[case],
                )
                grouped[(age, policy)].append((seed, case, flags))

    expected_records = int(screen_summary["source_screen_manifest"]["record_count"])
    if record_count != expected_records:
        raise ValueError(
            f"record count mismatch: streamed {record_count}, expected {expected_records}"
        )
    result_summary = {
        age: {
            policy: summarize_members(grouped[(age, policy)]) for policy in policies
        }
        for age in ages
    }
    confidence = float(
        diagnostic_protocol.get("fresh_confirmation", {}).get("confidence", 0.975)
    )
    diagnostic_statistics: dict[str, object] = {}
    for age in ages:
        diagnostic_statistics[age] = {}
        for policy in policies:
            nuisance_family_events = sum(
                any(
                    diagnostic_presence.get((age, policy, seed, case), False)
                    for case in NOMINAL_DIAGNOSTIC_CASES
                )
                for seed in seeds
            )
            trials = len(seeds)
            diagnostic_statistics[age][policy] = {
                "nuisance_family_events": nuisance_family_events,
                "nuisance_family_trials": trials,
                "nuisance_family_point_rate": (
                    nuisance_family_events / trials if trials else 0.0
                ),
                "nuisance_family_one_sided_clopper_pearson_upper": (
                    clopper_pearson_upper(
                        nuisance_family_events,
                        trials,
                        confidence,
                    )
                    if trials
                    else None
                ),
                "diagnostic_events_by_case": {
                    case: sum(
                        diagnostic_presence.get((age, policy, seed, case), False)
                        for seed in seeds
                    )
                    for case in screen_summary["source_screen_manifest"]["case_names"]
                },
            }

    confirmation = diagnostic_protocol.get("fresh_confirmation")
    confirmation_gate: dict[str, object] = {"applicable": False}
    if isinstance(confirmation, dict):
        manifest = screen_summary["source_screen_manifest"]
        applicable = (
            int(manifest["seed_start"]) == int(confirmation["seed_start"])
            and int(manifest["seed_count"]) == int(confirmation["seed_count"])
            and int(manifest["seed_end"]) == int(confirmation["seed_end"])
            and list(manifest["case_names"]) == list(confirmation["case_names"])
        )
        confirmation_gate = {"applicable": applicable}
        if applicable:
            selected = confirmation["selected_comparator"]
            selected_age = f"{float(selected['recent_quiet_boundary_max_age_s']):g}s"
            selected_policy = str(selected["policy"])
            selected_summary = result_summary[selected_age][selected_policy]
            selected_diagnostics = diagnostic_statistics[selected_age][selected_policy]
            nuisance_contract = confirmation["diagnostic_nuisance_family"]
            persistent_contract = confirmation["persistent_descriptive_union"]
            deterministic = confirmation["deterministic_boundaries"]
            family_rate = (
                selected_summary["descriptive_union_clean_family_passes"]
                / selected_summary["persistent_family_trials"]
            )
            diagnostic_events = selected_diagnostics["diagnostic_events_by_case"]
            checks = {
                "nuisance_point_rate": (
                    selected_diagnostics["nuisance_family_point_rate"]
                    <= float(nuisance_contract["maximum_point_diagnostic_rate"])
                ),
                "nuisance_confidence_upper": (
                    selected_diagnostics[
                        "nuisance_family_one_sided_clopper_pearson_upper"
                    ]
                    <= float(
                        nuisance_contract[
                            "maximum_one_sided_clopper_pearson_upper_rate"
                        ]
                    )
                ),
                "persistent_descriptive_union_family_rate": (
                    family_rate
                    >= float(persistent_contract["minimum_complete_family_rate"])
                ),
                "structural_gap_diagnostic_events": (
                    diagnostic_events[STRUCTURAL_GAP_CASE]
                    <= int(deterministic["maximum_structural_gap_diagnostic_events"])
                ),
                "pulse_1p5s_diagnostic_events": (
                    diagnostic_events[PULSE_1P5_CASE]
                    <= int(deterministic["maximum_1p5s_pulse_diagnostic_events"])
                ),
            }
            confirmation_gate = {
                "applicable": True,
                "selected_age": selected_age,
                "selected_policy": selected_policy,
                "checks": checks,
                "passed_registered_development_checks": all(checks.values()),
                "persistent_descriptive_union_family_rate": family_rate,
                "nuisance_family": selected_diagnostics,
                "pulse_2p0s_diagnostic_events_descriptive": diagnostic_events[
                    PULSE_2P0_CASE
                ],
                "control_promotion_authorized": False,
            }
    result = {
        "schema_version": 1,
        "study_id": "aerakia-v8-diagnostic-lane-attribution-audit",
        "status": "completed_read_only_non_promoting_audit",
        "execution": {
            "audit_git_commit": git_commit(),
            "full_trace_path": str(full_trace.relative_to(ROOT)),
            "full_trace_sha256": actual_sha256,
            "screen_summary_path": str(screen_summary_path.relative_to(ROOT)),
            "screen_summary_sha256": sha256_file(screen_summary_path),
            "screen_execution_git_commit": screen_summary["execution"]["git_commit"],
            "streamed_record_count": record_count,
            "diagnostic_protocol_path": str(
                diagnostic_protocol_path.relative_to(ROOT)
            ),
            "diagnostic_protocol_sha256": sha256_file(diagnostic_protocol_path),
        },
        "summary": result_summary,
        "diagnostic_statistics": diagnostic_statistics,
        "fresh_confirmation_gate": confirmation_gate,
        "nuisance_and_continuity_from_screen": {
            age: {
                policy: {
                    "diagnostic_on_nominal_count": screen_summary["diagnostic_summary"][age][policy]["diagnostic_on_nominal_count"],
                    "diagnostic_on_structural_gap_count": screen_summary["diagnostic_summary"][age][policy]["diagnostic_on_structural_gap_count"],
                    "diagnostic_snapshot_trials_all_cases": screen_summary["diagnostic_summary"][age][policy]["diagnostic_snapshot_trials_all_cases"],
                }
                for policy in policies
            }
            for age in ages
        },
        "limitations": [
            "The descriptive union combines a normal clean latch with a diagnostic-only observation; it is not control qualification.",
            "Offline injection time is used only for post-run attribution and never enters the runtime probe.",
            "Synthetic TAS offsets do not prove a pitot fault classifier, wind model, physical calibration, or flight authority.",
            "No v8 policy, age, threshold, estimator state, FCOne authority, or holdout is promoted by this audit."
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
        "--out",
        type=Path,
        default=ROOT / "validation" / "public" / "airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64_audit.json",
    )
    args = parser.parse_args(argv)
    result = audit_screen(
        full_trace=args.full_trace,
        screen_summary_path=args.screen_summary,
        output_path=args.out,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
