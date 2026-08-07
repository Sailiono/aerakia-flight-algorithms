#!/usr/bin/env python3
"""Run the hardware-independent v2 IMU input-contract smoke matrix.

This is a contract and provenance gate, not a capability claim.  It checks
that interval delta fields reconstruct the published quantized sensor stream,
that stationarity is causal and truth-independent, and that the existing host
runner can consume the same CSV at several IMU rates without numerical health
failures.  It intentionally does not tune or modify the ESKF.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "simulation/tools/generate_synthetic_imu.py"
DEFAULT_PROTOCOL = ROOT / "validation/bias_observability_input_contract_v2.json"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} contains no data rows")
    return rows


def verify_delta_contract(rows: list[dict[str, str]]) -> dict[str, object]:
    required = {
        "ts_us", "raw_acc_mg_x", "raw_acc_mg_y", "raw_acc_mg_z",
        "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
        "delta_interval_us", "delta_angle_rad_x", "delta_angle_rad_y",
        "delta_angle_rad_z", "delta_velocity_m_s_x", "delta_velocity_m_s_y",
        "delta_velocity_m_s_z",
    }
    missing = sorted(required - set(rows[0]))
    if missing:
        raise ValueError("v2 replay is missing columns: " + ", ".join(missing))
    timestamps = np.asarray([int(row["ts_us"]) for row in rows], dtype=np.int64)
    intervals = np.asarray([int(row["delta_interval_us"]) for row in rows], dtype=np.int64)
    if intervals[0] != 0 or np.any(np.diff(timestamps) <= 0):
        raise ValueError("timestamps must be increasing and the first delta interval must be zero")
    if not np.array_equal(intervals[1:], np.diff(timestamps)):
        raise ValueError("delta_interval_us does not match timestamp intervals")
    acceleration = np.asarray(
        [[float(row[f"raw_acc_mg_{axis}"]) for axis in "xyz"] for row in rows],
        dtype=np.float64,
    ) * 9.80665 / 1000.0
    angular_rate = np.asarray(
        [[float(row[f"raw_gyro_mdps_{axis}"]) for axis in "xyz"] for row in rows],
        dtype=np.float64,
    ) / 1000.0
    interval_s = intervals.astype(np.float64)[:, None] * 1.0e-6
    delta_velocity = np.asarray(
        [[float(row[f"delta_velocity_m_s_{axis}"]) for axis in "xyz"] for row in rows],
        dtype=np.float64,
    )
    delta_angle = np.asarray(
        [[float(row[f"delta_angle_rad_{axis}"]) for axis in "xyz"] for row in rows],
        dtype=np.float64,
    )
    np.testing.assert_allclose(delta_velocity, acceleration * interval_s, atol=1.0e-12, rtol=0.0)
    np.testing.assert_allclose(
        delta_angle, np.radians(angular_rate) * interval_s, atol=1.0e-12, rtol=0.0
    )
    return {
        "rows": len(rows),
        "duration_s": float((timestamps[-1] - timestamps[0]) * 1.0e-6),
        "interval_min_us": int(np.min(intervals[1:])),
        "interval_max_us": int(np.max(intervals[1:])),
        "delta_fields_match_published_rates": True,
    }


def verify_metadata(metadata: dict[str, object], rate_hz: float) -> dict[str, object]:
    expected = {
        "measurement_contract": "delta_interval_v2",
        "acceleration_time_semantics": "interval_start_zoh",
        "stationarity_source": "causal_imu_window",
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise ValueError(f"metadata {key}={metadata.get(key)!r}, expected {value!r}")
    if metadata.get("truth_derived_stationarity") is not False:
        raise ValueError("v2 stationarity must not be truth-derived")
    if float(metadata.get("rate_hz", -1.0)) != rate_hz:
        raise ValueError("metadata rate does not match requested rate")
    return {
        "measurement_contract": metadata["measurement_contract"],
        "stationarity_source": metadata["stationarity_source"],
        "truth_derived_stationarity": metadata["truth_derived_stationarity"],
        "stationarity_flagged_samples": int(metadata["stationarity_flagged_samples"]),
        "effective_accel_noise_m_s2": float(metadata["accel_noise_m_s2"]),
        "effective_gyro_noise_deg_s": float(metadata["gyro_noise_deg_s"]),
    }


def run_case(
    rate_hz: float,
    runner: Path,
    out_dir: Path,
    seed: int,
    protocol: dict[str, object],
    *,
    compact: bool,
) -> dict[str, object]:
    case_dir = out_dir / f"rate-{int(rate_hz):03d}hz"
    case_dir.mkdir(parents=True, exist_ok=True)
    input_path = case_dir / "input.csv"
    metadata_path = case_dir / "input-metadata.json"
    results_path = case_dir / "results.csv"
    measurement = protocol["measurement_contract"]
    stationarity = protocol["stationarity"]
    noise = protocol["noise"]
    trajectory = protocol["trajectory"]
    assert isinstance(measurement, dict)
    assert isinstance(stationarity, dict)
    assert isinstance(noise, dict)
    assert isinstance(trajectory, dict)
    generator_command = [
        sys.executable, str(GENERATOR),
        "--out", str(input_path), "--metadata", str(metadata_path),
        "--duration", str(trajectory["duration_s"]), "--rate", str(rate_hz),
        "--seed", str(seed), "--motion", str(trajectory["motion"]),
        "--anomaly", "none", "--static-hint",
        "--stationarity-source", "causal_imu_window",
        "--measurement-contract", str(measurement["id"]),
        "--accel-time-semantics", str(measurement["acceleration_time_semantics"]),
        "--accel-noise-density-m-s2-sqrt-hz", str(noise["accel_density_m_s2_sqrt_hz"]),
        "--gyro-noise-density-rad-s-sqrt-hz", str(noise["gyro_density_rad_s_sqrt_hz"]),
        "--rate-invariant-streams",
    ]
    (case_dir / "01-generator-command.txt").write_text(
        " ".join(generator_command) + "\n", encoding="utf-8"
    )
    with (case_dir / "01-generator.log").open("w", encoding="utf-8") as log:
        subprocess.run(
            generator_command, cwd=ROOT, check=True, text=True,
            stdout=log, stderr=subprocess.STDOUT,
        )
    rows = read_rows(input_path)
    delta_metrics = verify_delta_contract(rows)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata_metrics = verify_metadata(metadata, rate_hz)
    filter_command = [str(runner), "--cold-start", str(input_path), str(results_path)]
    (case_dir / "02-filter-command.txt").write_text(
        " ".join(filter_command) + "\n", encoding="utf-8"
    )
    with (case_dir / "02-filter.log").open("w", encoding="utf-8") as log:
        subprocess.run(
            filter_command, cwd=ROOT, check=True, text=True,
            stdout=log, stderr=subprocess.STDOUT,
        )
    result_rows = read_rows(results_path)
    aligned = np.asarray([float(row["eskf_static_aligned"]) for row in result_rows]) > 0.5
    healthy = np.asarray([float(row["eskf_healthy"]) for row in result_rows]) > 0.5
    recoveries = np.asarray(
        [int(float(row["eskf_navigation_recovery_count"])) for row in result_rows],
        dtype=np.int64,
    )
    static_flags = np.asarray([int(row["static_hint"]) for row in rows], dtype=np.int64)
    first_flag = int(np.flatnonzero(static_flags)[0]) if np.any(static_flags) else None
    if first_flag is None or first_flag < int(round(rate_hz * float(stationarity["window_s"]))):
        raise ValueError("causal stationarity asserted before its one-second history window")
    result = {
        "rate_hz": rate_hz,
        "seed": seed,
        "input_sha256": file_sha256(input_path),
        "runner_sha256": file_sha256(runner),
        "generator_sha256": file_sha256(GENERATOR),
        "input": delta_metrics | metadata_metrics,
        "first_stationarity_flag_sample": first_flag,
        "static_alignment_completed": bool(np.any(aligned)),
        "static_alignment_first_sample": int(np.flatnonzero(aligned)[0]) if np.any(aligned) else None,
        "healthy_ratio": float(np.mean(healthy)),
        "navigation_recoveries": int(np.max(recoveries)),
        "result_rows": len(result_rows),
    }
    if not result["static_alignment_completed"] or result["healthy_ratio"] < 1.0 \
            or result["navigation_recoveries"] != 0:
        raise ValueError(f"v2 rate case failed host integrity gate: {result}")
    if not compact:
        return result
    input_path.unlink(missing_ok=True)
    results_path.unlink(missing_ok=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out-dir", type=Path, default=Path("build/bias-input-contract-v2"))
    parser.add_argument("--rates", default=None, help="override the protocol rate list")
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    protocol_path = args.protocol if args.protocol.is_absolute() else ROOT / args.protocol
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("status") != "frozen_smoke_not_capability":
        parser.error("input contract protocol must be frozen_smoke_not_capability")
    declared_rates = protocol.get("rates_hz")
    if not isinstance(declared_rates, list) or not declared_rates:
        parser.error("protocol must declare rates_hz")
    rates = [float(value) for value in (args.rates.split(",") if args.rates else declared_rates)]
    if not rates or any(rate <= 0.0 for rate in rates) or len(set(rates)) != len(rates):
        parser.error("--rates must contain unique positive values")
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner does not exist: {runner}")
    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = [
        run_case(rate, runner, out_dir, args.seed + index, protocol, compact=args.compact)
        for index, rate in enumerate(rates)
    ]
    summary = {
        "schema_version": 1,
        "status": "passed",
        "protocol_id": "aerakia-bias-observability-input-contract-v2-smoke",
        "protocol_sha256": file_sha256(protocol_path),
        "measurement_contract": protocol["measurement_contract"]["id"],
        "stationarity_source": protocol["stationarity"]["source"],
        "truth_derived_stationarity": protocol["stationarity"]["truth_derived"],
        "rates_hz": rates,
        "cases": cases,
        "limitations": [
            "This is a contract/integrity smoke, not evidence that the estimator is bias-convergent.",
            "The public ESKF still consumes rate/specific-force samples; delta fields are adapter evidence.",
            "The causal IMU detector cannot distinguish rest from constant-velocity translation.",
            "Timestamp delay/jitter, quantization stress, bias random walk, thermal drift, and lever arm remain later campaigns.",
        ],
    }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
