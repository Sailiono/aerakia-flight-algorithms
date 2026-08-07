#!/usr/bin/env python3
"""Promote a complete barometer campaign into a compact public evidence record."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ARM_NAMES = (
    "imu_only",
    "imu_baro_raw",
    "imu_baro_supervised",
    "imu_baro_shadow_failover",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def compact_row(row: dict[str, Any]) -> dict[str, Any]:
    metric_name = "outage_vertical_position_rmse_m"
    metric = row.get(metric_name)
    shadow = row.get("shadow_failover")
    if not isinstance(metric, dict) or not isinstance(shadow, dict):
        raise ValueError("aggregate row is missing vertical RMSE or shadow-failover metrics")
    values = {arm: finite(metric[f"{arm}_mean"], f"{row.get('fault')} {metric_name} {arm}")
              for arm in ARM_NAMES}
    return {
        "fault": str(row["fault"]),
        "outage_duration_s": finite(row["outage_duration_s"], "outage duration"),
        "trials": int(row["trials"]),
        "vertical_position_rmse_m": values,
        "raw_baro_acceptance_ratio": finite(
            row["outage_baro_acceptance_ratio"]["imu_baro_raw_mean"], "raw acceptance"
        ),
        "supervised_baro_acceptance_ratio": finite(
            row["outage_baro_acceptance_ratio"]["imu_baro_supervised_mean"],
            "supervised acceptance",
        ),
        "shadow_failover": {
            "switch_trials": int(shadow["switch_count"]),
            "detection_delay_p95_s": shadow.get("detection_delay_p95_s"),
            "position_reset_abs_p95_m": shadow.get("position_reset_abs_p95_m"),
            "position_reset_abs_max_m": shadow.get("position_reset_abs_max_m"),
        },
    }


def promote(summary: dict[str, Any], *, summary_sha256: str, manifest_sha256: str) -> dict[str, Any]:
    if summary.get("status") != "completed" or summary.get("provenance", {}).get("completeness_checked") is not True:
        raise ValueError("source campaign is incomplete or was not completeness-checked")
    faults = [str(item) for item in summary.get("faults", [])]
    outages = [finite(item, "outage") for item in summary.get("outages_s", [])]
    seeds = int(summary.get("seeds", 0))
    if not faults or not outages or seeds <= 0:
        raise ValueError("source campaign has no valid matrix definition")
    expected_trials = len(faults) * len(outages) * seeds
    if int(summary.get("trial_count", -1)) != expected_trials:
        raise ValueError("source campaign trial count does not match its declared matrix")
    rows = [compact_row(row) for row in summary.get("aggregate", [])]
    expected_rows = {(fault, outage) for fault in faults for outage in outages}
    actual_rows = {(row["fault"], row["outage_duration_s"]) for row in rows}
    if actual_rows != expected_rows or any(row["trials"] != seeds for row in rows):
        raise ValueError("aggregate rows are incomplete, duplicated, or have the wrong seed count")

    trial_arms = [
        finite(trial[arm]["healthy_ratio"], f"trial {arm} healthy ratio")
        for trial in summary["trials"]
        for arm in ARM_NAMES
    ]
    shadow_trials = [trial["imu_baro_shadow_failover"] for trial in summary["trials"]]
    return {
        "schema_version": 1,
        "status": "completed",
        "campaign": {
            "id": "aerakia-synthetic-barometer-outage-v1",
            "scope": summary["scope"],
            "rate_hz": finite(summary["rate_hz"], "rate_hz"),
            "faults": faults,
            "outages_s": outages,
            "seeds_per_cell": seeds,
            "completed_trials": expected_trials,
            "aggregate_cells": len(rows),
            "arms": {
                "imu_only": "barometer disabled",
                "imu_baro_raw": "current scalar relative-height update",
                "imu_baro_supervised": "experimental causal source supervisor",
                "imu_baro_shadow_failover": "offline, partial-output diagnostic mux only",
            },
        },
        "result": {
            "all_arm_healthy_ratio_min": min(trial_arms),
            "supervisor_latch_event_trials": sum(
                int(trial["shadow_supervisor_latch_count"]) > 0 for trial in shadow_trials
            ),
            "supervisor_latch_event_during_outage_trials": sum(
                int(trial["shadow_supervisor_latch_during_outage_count"]) > 0
                for trial in shadow_trials
            ),
            "aggregate": rows,
        },
        "provenance": {
            "source_campaign_summary_sha256": summary_sha256,
            "source_campaign_manifest_sha256": manifest_sha256,
            "runner_sha256": summary["provenance"]["shared_runner_sha256"],
            "generator_sha256": summary["provenance"]["shared_generator_sha256"],
            "barometer_supervisor_sha256": summary["provenance"]["shared_barometer_supervisor_sha256"],
            "single_shard_script_sha256": summary["provenance"]["shared_campaign_script_sha256"],
        },
        "limitations": [
            *summary.get("limitations", []),
            "Shadow scores only recognize a 0-to-1 latch transition inside the declared GNSS-outage window.",
            "The shadow arm switches only vertical position, velocity, and health in offline replay; it does not validate a complete state/covariance/controller handoff.",
            "This result is diagnostic evidence, not a flight-promotion or source-supervisor release gate.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, default=Path("build/baro-outage-release/summary.json"),
        help="merged complete campaign summary",
    )
    parser.add_argument(
        "--manifest", type=Path, default=None,
        help="campaign-manifest.json; defaults beside --input",
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path("validation/public/barometer_outage_campaign_v1.json"),
    )
    args = parser.parse_args()
    source = args.input.resolve()
    manifest = (args.manifest or source.with_name("campaign-manifest.json")).resolve()
    output = args.output.resolve()
    try:
        summary = json.loads(source.read_text(encoding="utf-8"))
        json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SystemExit(f"cannot read campaign input: {error}") from error
    evidence = promote(summary, summary_sha256=sha256(source), manifest_sha256=sha256(manifest))
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    print(f"Promoted compact barometer evidence: {output.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
