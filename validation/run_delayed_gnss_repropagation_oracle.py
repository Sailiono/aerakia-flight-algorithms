#!/usr/bin/env python3
"""Run the bounded delayed-GNSS repropagation oracle.

The native oracle is deliberately host-only and research-only. It exercises
exact source/delivery timestamp pairs for delayed GNSS position/velocity
observations. The overlap mode has two source epochs whose delivery windows
overlap and arrive in reverse order. Its newer event must remain different
from the zero-delay baseline while the earlier source is pending; only the
fully delivered lane must become equivalent. The gate is replay correctness,
not an accuracy or flight-readiness claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RATES = (100, 200, 400)
DEFAULT_DELAYS = (20, 50, 100, 150)
OVERLAP_DEFAULT_DELAYS = (50, 100, 150)
OVERLAP_MIN_DELAY_MS = 50
EXPECTED_STATUS = "host_only_research_oracle_not_flight_feature"
EXPECTED_INPUT_CONTRACT = "synthetic_exact_timestamp_pv_only"
SINGLE_SCENARIO = "single_isolated"
SEQUENTIAL_SCENARIO = "sequential_two_non_overlapping"
OVERLAPPING_SCENARIO = "overlapping_reordered_two_event"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
    }


def parse_integer_list(value: str, label: str) -> tuple[int, ...]:
    try:
        values = tuple(int(item.strip()) for item in value.split(",") if item.strip())
    except ValueError as error:
        raise ValueError(f"{label} must contain integers") from error
    if not values or values != tuple(sorted(set(values))) or any(item <= 0 for item in values):
        raise ValueError(f"{label} must be unique, increasing, and positive")
    return values


def parse_delays(scenario: str, value: str | None) -> tuple[int, ...]:
    """Parse scenario-specific delays and reject a non-late overlap schedule."""

    if value is None:
        return OVERLAP_DEFAULT_DELAYS if scenario == "overlap" else DEFAULT_DELAYS
    delays = parse_integer_list(value, "--delays-ms")
    if scenario == "overlap" and any(delay < OVERLAP_MIN_DELAY_MS for delay in delays):
        raise ValueError(
            f"overlap requires delays of at least {OVERLAP_MIN_DELAY_MS} ms "
            "so the earlier event remains pending when the newer event arrives"
        )
    return delays


def finite_number(value: object) -> float | None:
    """Return a finite number without accepting malformed native JSON silently."""

    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def integer(value: object) -> int | None:
    """Accept JSON integer fields only; bool and float values are malformed."""

    return value if isinstance(value, int) and not isinstance(value, bool) else None


def numeric_at_most(value: object, tolerance: float) -> bool:
    number = finite_number(value)
    return number is not None and math.isfinite(tolerance) and number <= tolerance


def numeric_above(value: object, tolerance: float) -> bool:
    number = finite_number(value)
    return number is not None and math.isfinite(tolerance) and number > tolerance


def event_progress_checks(event: dict[str, Any], prefix: str,
                          state_tolerance: float) -> list[dict[str, Any]]:
    """Check one delayed event before deciding whether equivalence is due."""

    label = f"{prefix}_" if prefix else ""
    return [
        {
            "name": f"{label}source_update_accepted",
            "passed": event.get("source_gps_accepted") is True,
        },
        {
            "name": f"{label}repropagation_executed",
            "passed": event.get("repropagated") is True,
        },
        {
            "name": f"{label}pre_delivery_difference_is_observable",
            "passed": numeric_above(
                event.get("pre_delivery_max_state_difference"), state_tolerance
            ),
        },
    ]


def event_equivalence_checks(event: dict[str, Any], prefix: str,
                             state_tolerance: float,
                             covariance_tolerance: float) -> list[dict[str, Any]]:
    """Check the zero-delay equivalence that is valid only after all sources arrive."""

    label = f"{prefix}_" if prefix else ""
    return [
        {
            "name": f"{label}post_delivery_state_matches_zero_delay",
            "passed": numeric_at_most(
                event.get("post_delivery_max_state_difference"), state_tolerance
            ),
        },
        {
            "name": f"{label}post_delivery_covariance_matches_zero_delay",
            "passed": numeric_at_most(
                event.get("post_delivery_max_covariance_difference"), covariance_tolerance
            ),
        },
        {
            "name": f"{label}post_delivery_metadata_matches_zero_delay",
            "passed": event.get("post_delivery_metadata_match") is True,
        },
    ]


def event_checks(event: dict[str, Any], prefix: str, state_tolerance: float,
                 covariance_tolerance: float) -> list[dict[str, Any]]:
    """Return complete checks for a fully delivered delayed event."""

    return event_progress_checks(event, prefix, state_tolerance) + event_equivalence_checks(
        event, prefix, state_tolerance, covariance_tolerance
    )


def validate_case(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Return named, fail-closed checks for one native oracle result."""

    state_tolerance = finite_number(payload.get("state_tolerance"))
    covariance_tolerance = finite_number(payload.get("covariance_tolerance"))
    valid_tolerances = (
        state_tolerance is not None
        and covariance_tolerance is not None
        and state_tolerance >= 0.0
        and covariance_tolerance >= 0.0
    )
    checked_state_tolerance = state_tolerance if state_tolerance is not None else float("nan")
    checked_covariance_tolerance = (
        covariance_tolerance if covariance_tolerance is not None else float("nan")
    )
    scenario = payload.get("scenario", SINGLE_SCENARIO)
    checks: list[dict[str, Any]] = [
        {
            "name": "research_only_status_is_explicit",
            "passed": payload.get("status") == EXPECTED_STATUS,
        },
        {
            "name": "synthetic_exact_timestamp_pv_contract_is_explicit",
            "passed": payload.get("input_contract") == EXPECTED_INPUT_CONTRACT,
        },
        {
            "name": "finite_nonnegative_tolerances_are_explicit",
            "passed": valid_tolerances,
        },
    ]
    if scenario == SINGLE_SCENARIO:
        checks.extend(event_checks(
            payload, "", checked_state_tolerance, checked_covariance_tolerance
        ))
    elif scenario == SEQUENTIAL_SCENARIO:
        events = payload.get("events")
        first = events[0] if isinstance(events, list) and len(events) >= 1 else {}
        second = events[1] if isinstance(events, list) and len(events) >= 2 else {}
        checks.extend([
            {
                "name": "sequential_mode_is_explicit",
                "passed": payload.get("scenario") == SEQUENTIAL_SCENARIO,
            },
            {
                "name": "sequential_mode_has_exactly_two_events",
                "passed": (
                    isinstance(events, list)
                    and len(events) == 2
                    and payload.get("event_count") == 2
                ),
            },
            {
                "name": "second_source_is_after_first_delivery",
                "passed": (
                    isinstance(first, dict)
                    and isinstance(second, dict)
                    and integer(second.get("source_timestamp_us")) is not None
                    and integer(first.get("delivery_timestamp_us")) is not None
                    and integer(second.get("source_timestamp_us"))
                    > integer(first.get("delivery_timestamp_us"))
                    and payload.get("sequential_non_overlapping") is True
                ),
            },
        ])
        checks.extend(event_checks(
            first, "first", checked_state_tolerance, checked_covariance_tolerance
        ))
        checks.extend(event_checks(
            second, "second", checked_state_tolerance, checked_covariance_tolerance
        ))
        checks.extend([
            {
                "name": "final_state_matches_zero_delay",
                "passed": numeric_at_most(
                    payload.get("final_max_state_difference"), checked_state_tolerance
                ),
            },
            {
                "name": "final_covariance_matches_zero_delay",
                "passed": numeric_at_most(
                    payload.get("final_max_covariance_difference"),
                    checked_covariance_tolerance,
                ),
            },
            {
                "name": "final_metadata_matches_zero_delay",
                "passed": payload.get("final_metadata_match") is True,
            },
        ])
    elif scenario == OVERLAPPING_SCENARIO:
        events = payload.get("events")
        older = events[0] if isinstance(events, list) and len(events) >= 1 else {}
        newer = events[1] if isinstance(events, list) and len(events) >= 2 else {}
        older_source = integer(older.get("source_timestamp_us")) if isinstance(older, dict) else None
        newer_source = integer(newer.get("source_timestamp_us")) if isinstance(newer, dict) else None
        older_delivery = (
            integer(older.get("delivery_timestamp_us")) if isinstance(older, dict) else None
        )
        newer_delivery = (
            integer(newer.get("delivery_timestamp_us")) if isinstance(newer, dict) else None
        )
        checks.extend([
            {
                "name": "overlap_mode_is_explicit",
                "passed": payload.get("scenario") == OVERLAPPING_SCENARIO,
            },
            {
                "name": "overlap_mode_has_exactly_two_events",
                "passed": (
                    isinstance(events, list)
                    and len(events) == 2
                    and payload.get("event_count") == 2
                ),
            },
            {
                "name": "overlap_source_order_is_chronological",
                "passed": (
                    older_source is not None
                    and newer_source is not None
                    and older_source < newer_source
                ),
            },
            {
                "name": "overlap_delivery_windows_are_explicit_and_real",
                "passed": (
                    payload.get("overlapping_delivery_windows") is True
                    and older_source is not None
                    and newer_source is not None
                    and older_delivery is not None
                    and newer_source < older_delivery
                ),
            },
            {
                "name": "overlap_delivery_order_is_reversed",
                "passed": (
                    payload.get("delivery_order_reversed") is True
                    and older_delivery is not None
                    and newer_delivery is not None
                    and newer_delivery < older_delivery
                ),
            },
        ])
        checks.extend(event_checks(
            older, "older", checked_state_tolerance, checked_covariance_tolerance
        ))
        checks.extend(event_progress_checks(newer, "newer", checked_state_tolerance))
        checks.extend([
            {
                "name": "newer_delivery_reports_one_pending_earlier_event",
                "passed": (
                    isinstance(newer, dict)
                    and newer.get("pending_earlier_event_count_at_delivery") == 1
                ),
            },
            {
                "name": "newer_delivery_does_not_prematurely_require_zero_delay_equivalence",
                "passed": (
                    isinstance(newer, dict)
                    and newer.get("zero_delay_equivalence_required_after_delivery") is False
                ),
            },
            {
                "name": "newer_delivery_remains_observably_different_while_older_event_is_pending",
                "passed": (
                    isinstance(newer, dict)
                    and numeric_above(
                        newer.get("post_delivery_max_state_difference"),
                        checked_state_tolerance,
                    )
                ),
            },
            {
                "name": "older_delivery_has_no_pending_earlier_event",
                "passed": (
                    isinstance(older, dict)
                    and older.get("pending_earlier_event_count_at_delivery") == 0
                ),
            },
            {
                "name": "older_delivery_requires_zero_delay_equivalence",
                "passed": (
                    isinstance(older, dict)
                    and older.get("zero_delay_equivalence_required_after_delivery") is True
                ),
            },
            {
                "name": "final_state_matches_zero_delay",
                "passed": numeric_at_most(
                    payload.get("final_max_state_difference"), checked_state_tolerance
                ),
            },
            {
                "name": "final_covariance_matches_zero_delay",
                "passed": numeric_at_most(
                    payload.get("final_max_covariance_difference"),
                    checked_covariance_tolerance,
                ),
            },
            {
                "name": "final_metadata_matches_zero_delay",
                "passed": payload.get("final_metadata_match") is True,
            },
        ])
    else:
        checks.append({"name": "scenario_is_known", "passed": False})
    checks.extend([
        {
            "name": "finite_healthy_state",
            "passed": payload.get("healthy") is True,
        },
        {
            "name": "covariance_remains_psd",
            "passed": payload.get("covariance_psd") is True,
        },
        {
            "name": "native_case_gate_passed",
            "passed": payload.get("pass") is True,
        },
    ])
    return checks


def run_case(oracle: Path, rate_hz: int, delay_ms: int, scenario: str) -> dict[str, Any]:
    command = [
        str(oracle),
        "--rate-hz",
        str(rate_hz),
        "--delay-ms",
        str(delay_ms),
        "--scenario",
        scenario,
    ]
    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"native oracle emitted invalid JSON (returncode={completed.returncode}):\n"
            f"{completed.stdout}"
        ) from error
    if not isinstance(payload, dict):
        raise RuntimeError("native oracle result must be a JSON object")
    checks = validate_case(payload)
    return {
        "rate_hz": rate_hz,
        "delay_ms": delay_ms,
        "scenario": scenario,
        "command": command,
        "returncode": completed.returncode,
        "checks": checks,
        "passed": completed.returncode == 0 and all(check["passed"] for check in checks),
        "result": payload,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oracle", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("build/delayed-gnss-reprop/oracle-v1.json"))
    parser.add_argument("--rates", default=",".join(str(value) for value in DEFAULT_RATES))
    parser.add_argument(
        "--delays-ms",
        default=None,
        help=(
            "comma-separated increasing delays; defaults to 20,50,100,150 for "
            "single/sequential and 50,100,150 for overlap"
        ),
    )
    parser.add_argument("--scenario", choices=("single", "sequential", "overlap"), default="single")
    args = parser.parse_args()
    oracle = args.oracle.resolve()
    if not oracle.is_file():
        parser.error(f"oracle does not exist: {oracle}")
    try:
        rates = parse_integer_list(args.rates, "--rates")
        delays = parse_delays(args.scenario, args.delays_ms)
    except ValueError as error:
        parser.error(str(error))

    cases: list[dict[str, Any]] = []
    for rate_hz in rates:
        for delay_ms in delays:
            cases.append(run_case(oracle, rate_hz, delay_ms, args.scenario))
    passed = all(case["passed"] for case in cases)
    output = args.out if args.out.is_absolute() else ROOT / args.out
    output.parent.mkdir(parents=True, exist_ok=True)
    document: dict[str, Any] = {
        "schema_version": 1,
        "status": EXPECTED_STATUS,
        "scope": {
            "single": "synthetic exact-timestamp isolated GNSS P/V rewind/repropagation oracle",
            "sequential": (
                "synthetic exact-timestamp sequential non-overlapping GNSS P/V "
                "rewind/repropagation oracle"
            ),
            "overlap": (
                "synthetic exact-timestamp overlapping reordered two-event GNSS P/V "
                "rewind/repropagation oracle"
            ),
        }[args.scenario],
        "scenario": args.scenario,
        "input_contract": EXPECTED_INPUT_CONTRACT,
        "rates_hz": list(rates),
        "delays_ms": list(delays),
        "case_count": len(cases),
        "passed": passed,
        "oracle": fingerprint(oracle),
        "oracle_source": fingerprint(ROOT / "validation" / "delayed_gnss_reprop_oracle.c"),
        "campaign_runner": fingerprint(Path(__file__).resolve()),
        "cases": cases,
        "limitations": [
            {
                "single": "Only one isolated delayed GNSS P/V event is replayed per case.",
                "sequential": (
                    "Exactly two sequential, non-overlapping delayed GNSS P/V events are "
                    "replayed per case."
                ),
                "overlap": (
                    "Exactly two overlapping GNSS P/V events are replayed per case, with the "
                    "newer source delivered first."
                ),
            }[args.scenario],
            "The ring is a host validation implementation and is not a production rewind API.",
            "No delayed heading, barometer, multi-IMU, supervisor, sensor-delay, or controller path is covered.",
            "The synthetic truth creates the P/V measurement only; the replay logic receives no truth fields.",
            "Equivalence to a zero-delay reference is a replay correctness result, not flight readiness.",
        ],
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
    }
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(document, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
