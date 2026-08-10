#!/usr/bin/env python3
"""Run the bounded delayed-GNSS repropagation oracle.

The native oracle is deliberately host-only and research-only.  It exercises
one exact source/delivery timestamp pair per case, with a single delayed GNSS
position/velocity observation and all other P/V observations on time.  A
sequential mode exercises exactly two non-overlapping delayed events.  The
strict gate is equivalence to the zero-delay reference after replay, not an
accuracy or flight-readiness claim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RATES = (100, 200, 400)
DEFAULT_DELAYS = (20, 50, 100, 150)
EXPECTED_STATUS = "host_only_research_oracle_not_flight_feature"
EXPECTED_INPUT_CONTRACT = "synthetic_exact_timestamp_pv_only"
SINGLE_SCENARIO = "single_isolated"
SEQUENTIAL_SCENARIO = "sequential_two_non_overlapping"


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


def event_checks(event: dict[str, Any], prefix: str, state_tolerance: float,
                 covariance_tolerance: float) -> list[dict[str, Any]]:
    """Return per-event checks shared by the isolated and sequential scenarios."""

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
            "passed": float(event.get("pre_delivery_max_state_difference", 0.0))
            > state_tolerance,
        },
        {
            "name": f"{label}post_delivery_state_matches_zero_delay",
            "passed": float(event.get("post_delivery_max_state_difference", float("inf")))
            <= state_tolerance,
        },
        {
            "name": f"{label}post_delivery_covariance_matches_zero_delay",
            "passed": float(event.get("post_delivery_max_covariance_difference", float("inf")))
            <= covariance_tolerance,
        },
        {
            "name": f"{label}post_delivery_metadata_matches_zero_delay",
            "passed": event.get("post_delivery_metadata_match") is True,
        },
    ]


def validate_case(payload: dict[str, Any]) -> list[dict[str, Any]]:
    """Return named, fail-closed checks for one native oracle result."""

    state_tolerance = float(payload.get("state_tolerance", float("nan")))
    covariance_tolerance = float(payload.get("covariance_tolerance", float("nan")))
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
    ]
    if scenario == SINGLE_SCENARIO:
        checks.extend(event_checks(payload, "", state_tolerance, covariance_tolerance))
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
                    and int(second.get("source_timestamp_us", 0))
                    > int(first.get("delivery_timestamp_us", 0))
                    and payload.get("sequential_non_overlapping") is True
                ),
            },
        ])
        checks.extend(event_checks(first, "first", state_tolerance, covariance_tolerance))
        checks.extend(event_checks(second, "second", state_tolerance, covariance_tolerance))
        checks.extend([
            {
                "name": "final_state_matches_zero_delay",
                "passed": float(payload.get("final_max_state_difference", float("inf")))
                <= state_tolerance,
            },
            {
                "name": "final_covariance_matches_zero_delay",
                "passed": float(payload.get("final_max_covariance_difference", float("inf")))
                <= covariance_tolerance,
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
    parser.add_argument("--delays-ms", default=",".join(str(value) for value in DEFAULT_DELAYS))
    parser.add_argument("--scenario", choices=("single", "sequential"), default="single")
    args = parser.parse_args()
    oracle = args.oracle.resolve()
    if not oracle.is_file():
        parser.error(f"oracle does not exist: {oracle}")
    try:
        rates = parse_integer_list(args.rates, "--rates")
        delays = parse_integer_list(args.delays_ms, "--delays-ms")
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
        "scope": (
            "synthetic exact-timestamp sequential GNSS P/V rewind/repropagation oracle"
            if args.scenario == "sequential"
            else "synthetic exact-timestamp isolated GNSS P/V rewind/repropagation oracle"
        ),
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
            (
                "Exactly two sequential, non-overlapping delayed GNSS P/V events are replayed per case."
                if args.scenario == "sequential"
                else "Only one isolated delayed GNSS P/V event is replayed per case."
            ),
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
