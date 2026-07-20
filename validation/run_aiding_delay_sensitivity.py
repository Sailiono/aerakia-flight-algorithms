#!/usr/bin/env python3
"""Measure timestamped GNSS delivery delay without pretending to support OOSM fusion.

The ESKF adapter checks source timestamps and observation age, but it deliberately does not
rewind state/covariance to fuse an out-of-sequence measurement.  This runner keeps physical
source time separate from delivery time, exercises 0--200 ms delay at the reviewed IMU rates,
and reports both:

* the current best-effort age-window behaviour; and
* a strict zero-age policy that rejects every delayed observation before it can mutate state.

It is an input-contract and sensitivity study, not evidence that the current ESKF compensates
transport latency.  Do not enable a nonzero age window in a flight adapter solely because this
host study completes.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any


GPS_FIELDS = (
    "gps_position_n_m",
    "gps_position_e_m",
    "gps_position_d_m",
    "gps_velocity_n_m_s",
    "gps_velocity_e_m_s",
    "gps_velocity_d_m_s",
    "gps_position_variance_m2",
    "gps_velocity_variance_m2_s2",
)
STATUS_OK = 0
STATUS_STALE_MEASUREMENT = -5


def run(command: list[str], root: Path) -> None:
    completed = subprocess.run(
        command,
        cwd=root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"{' '.join(command)}\n{completed.stdout}")


def parse_int_list(value: str, *, label: str, minimum: int) -> list[int]:
    try:
        parsed = [int(item) for item in value.split(",") if item.strip()]
    except ValueError as error:
        raise ValueError(f"{label} must contain integers") from error
    if not parsed or parsed != sorted(set(parsed)) or any(item < minimum for item in parsed):
        raise ValueError(f"{label} must be unique increasing integers no smaller than {minimum}")
    return parsed


def schedule_delayed_gps(source: Path, destination: Path, delay_ms: int) -> dict[str, int]:
    """Move GNSS deliveries while preserving their physical sample timestamp.

    IMU/truth rows remain at delivery time.  Only the GNSS update marker and payload are moved;
    `gps_timestamp_us` identifies when the copied measurement was acquired.  Later source rows
    may replace an earlier row only if the caller asks for a delay that maps two observations to
    the same IMU row, which this function rejects instead of silently dropping evidence.
    """
    with source.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    required = {"ts_us", "position_update", *GPS_FIELDS}
    missing = sorted(required - set(fields))
    if missing:
        raise ValueError(f"input replay lacks required GNSS fields: {', '.join(missing)}")
    if "gps_timestamp_us" not in fields:
        fields.append("gps_timestamp_us")

    timestamps = [int(row["ts_us"]) for row in rows]
    if timestamps != sorted(timestamps) or len(set(timestamps)) != len(timestamps):
        raise ValueError("input IMU timestamps must be strictly increasing")
    delay_us = delay_ms * 1000
    source_updates: list[dict[str, str]] = [
        row.copy() for row in rows if int(float(row["position_update"])) != 0
    ]
    for row in rows:
        row["position_update"] = "0"
        row["gps_timestamp_us"] = row["ts_us"]

    delivery_rows: set[int] = set()
    dropped = 0
    for source_row in source_updates:
        target_us = int(source_row["ts_us"]) + delay_us
        delivery_index = next(
            (index for index, timestamp_us in enumerate(timestamps) if timestamp_us >= target_us),
            None,
        )
        if delivery_index is None:
            dropped += 1
            continue
        if delivery_index in delivery_rows:
            raise ValueError(
                f"delay {delay_ms} ms maps multiple GNSS observations to row {delivery_index}"
            )
        delivery_rows.add(delivery_index)
        delivery = rows[delivery_index]
        delivery["position_update"] = "1"
        delivery["gps_timestamp_us"] = source_row["ts_us"]
        for field in GPS_FIELDS:
            delivery[field] = source_row[field]

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return {
        "source_updates": len(source_updates),
        "scheduled_updates": len(delivery_rows),
        "dropped_at_end": dropped,
        "delay_ms": delay_ms,
    }


def load_status_counts(results_csv: Path) -> dict[str, int]:
    with results_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    attempted = [row for row in rows if int(float(row["input_position_update"])) != 0]
    counts = Counter(int(float(row["input_gps_status"])) for row in attempted)
    return {
        "delivery_attempts": len(attempted),
        "transport_accepted": counts[STATUS_OK],
        "stale_rejected": counts[STATUS_STALE_MEASUREMENT],
        "other_transport_status": sum(
            count for status, count in counts.items()
            if status not in (STATUS_OK, STATUS_STALE_MEASUREMENT)
        ),
        "statuses": {str(status): count for status, count in sorted(counts.items())},
    }


def summary_metrics(metrics: dict[str, Any]) -> dict[str, float | None]:
    navigation = metrics.get("navigation") or {}
    integrity = metrics.get("eskf_integrity") or {}
    algorithms = metrics.get("algorithms") or {}
    eskf = algorithms.get("eskf") or {}
    return {
        "attitude_rmse_deg": float(eskf["overall_attitude_rmse_deg"]),
        "position_rmse_m": float(navigation["position_rmse_m"]),
        "velocity_rmse_m_s": float(navigation["velocity_rmse_m_s"]),
        "healthy_ratio": float(integrity["healthy_ratio"]),
        "navigation_recoveries": float(integrity["navigation_recoveries"]),
    }


def case_directory(out_dir: Path, policy: str, rate_hz: int, delay_ms: int) -> Path:
    return out_dir / policy / f"{rate_hz}hz" / f"delay-{delay_ms:03d}ms"


def run_case(
    *,
    root: Path,
    runner: Path,
    input_csv: Path,
    out_dir: Path,
    maximum_aiding_age_s: float,
    resume: bool,
) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    results_csv = out_dir / "results.csv"
    metrics_json = out_dir / "metrics.json"
    if resume and results_csv.is_file() and metrics_json.is_file():
        metrics = json.loads(metrics_json.read_text(encoding="utf-8"))
        return {
            "metrics": summary_metrics(metrics),
            "transport": load_status_counts(results_csv),
        }
    run(
        [
            str(runner),
            "--cold-start",
            "--maximum-aiding-age-s",
            f"{maximum_aiding_age_s:.9g}",
            str(input_csv),
            str(results_csv),
        ],
        root,
    )
    run(
        [
            sys.executable,
            str(root / "validation" / "analyze_results.py"),
            str(results_csv),
            "--out-dir",
            str(out_dir),
            "--scenario",
            "timestamped_gnss_delay",
            "--no-plots",
        ],
        root,
    )
    metrics = json.loads((out_dir / "metrics.json").read_text(encoding="utf-8"))
    return {
        "metrics": summary_metrics(metrics),
        "transport": load_status_counts(results_csv),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/aiding-delay-sensitivity"))
    parser.add_argument("--rates", default="100,200,400,1000")
    parser.add_argument("--delays-ms", default="0,25,50,100,200")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument(
        "--resume", action="store_true",
        help="reuse complete per-policy result/metric pairs in --out-dir",
    )
    parser.add_argument(
        "--maximum-aiding-age-s",
        type=float,
        default=0.2,
        help="best-effort policy to study; this does not add delayed-state fusion",
    )
    args = parser.parse_args()
    if args.duration <= 2.0:
        parser.error("--duration must exceed the static-alignment window")
    if not math.isfinite(args.maximum_aiding_age_s) or args.maximum_aiding_age_s < 0.0:
        parser.error("--maximum-aiding-age-s must be finite and non-negative")
    try:
        rates = parse_int_list(args.rates, label="--rates", minimum=100)
        delays_ms = parse_int_list(args.delays_ms, label="--delays-ms", minimum=0)
    except ValueError as error:
        parser.error(str(error))

    root = Path(__file__).resolve().parents[1]
    runner = args.runner.resolve()
    if not runner.is_file():
        parser.error(f"runner not found: {runner}")
    out_dir = args.out_dir.resolve()
    records: list[dict[str, Any]] = []
    for rate_hz in rates:
        base_dir = out_dir / "inputs" / f"{rate_hz}hz"
        source_input = base_dir / "source.csv"
        noise_scale = math.sqrt(rate_hz / 100.0)
        run(
            [
                sys.executable,
                str(root / "simulation" / "tools" / "generate_synthetic_imu.py"),
                "--out", str(source_input),
                "--duration", str(args.duration),
                "--rate", str(rate_hz),
                "--seed", str(args.seed),
                "--motion", "navigation_outage",
                "--static-hint",
                "--disable-magnetometer",
                "--mag-rate-hz", "100",
                "--rate-invariant-streams",
                "--accel-noise-m-s2", str(0.02 * noise_scale),
                "--gyro-noise-deg-s", str(0.05 * noise_scale),
            ],
            root,
        )
        for delay_ms in delays_ms:
            input_csv = base_dir / f"delay-{delay_ms:03d}ms.csv"
            schedule = schedule_delayed_gps(source_input, input_csv, delay_ms)
            for policy, maximum_age_s in (
                ("best_effort_age_window", args.maximum_aiding_age_s),
                ("strict_zero_age", 0.0),
            ):
                case_dir = case_directory(out_dir, policy, rate_hz, delay_ms)
                result = run_case(
                    root=root,
                    runner=runner,
                    input_csv=input_csv,
                    out_dir=case_dir,
                    maximum_aiding_age_s=maximum_age_s,
                    resume=args.resume,
                )
                record = {
                    "rate_hz": rate_hz,
                    "delay_ms": delay_ms,
                    "policy": policy,
                    "maximum_aiding_age_s": maximum_age_s,
                    "schedule": schedule,
                    **result,
                }
                records.append(record)

    checks: list[dict[str, Any]] = []
    for record in records:
        transport = record["transport"]
        metrics = record["metrics"]
        delayed = record["delay_ms"] > 0
        expected_accept = record["policy"] == "best_effort_age_window"
        expected_transport = transport["delivery_attempts"]
        checks.append({
            "rate_hz": record["rate_hz"],
            "delay_ms": record["delay_ms"],
            "policy": record["policy"],
            "name": "all_deliveries_have_expected_transport_outcome",
            "passed": (
                transport["transport_accepted"] == expected_transport
                if expected_accept or not delayed
                else transport["stale_rejected"] == expected_transport
            ),
        })
        checks.append({
            "rate_hz": record["rate_hz"],
            "delay_ms": record["delay_ms"],
            "policy": record["policy"],
            "name": "estimator_remains_numerically_healthy",
            "passed": metrics["healthy_ratio"] == 1.0,
        })
    passed = all(check["passed"] for check in checks)
    document = {
        "schema_version": 1,
        "scope": (
            "Timestamped GNSS delivery-delay sensitivity. The estimator does not implement "
            "out-of-sequence measurement rewind/replay."
        ),
        "rates_hz": rates,
        "delays_ms": delays_ms,
        "duration_s": args.duration,
        "seed": args.seed,
        "best_effort_maximum_aiding_age_s": args.maximum_aiding_age_s,
        "records": records,
        "checks": checks,
        "passed": passed,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    lines = [
        "# Timestamped GNSS delivery-delay sensitivity",
        "",
        "The best-effort age-window rows are **not delayed fusion**: the current state is updated "
        "with an older observation. Strict-zero-age rows demonstrate the fail-closed alternative.",
        "",
        "| Policy | IMU rate | Delay | Transport accepted / attempted | Position RMSE | Velocity RMSE | Attitude RMSE | Health |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for record in records:
        transport = record["transport"]
        metrics = record["metrics"]
        lines.append(
            f"| `{record['policy']}` | {record['rate_hz']} Hz | {record['delay_ms']} ms | "
            f"{transport['transport_accepted']} / {transport['delivery_attempts']} | "
            f"{metrics['position_rmse_m']:.4f} m | {metrics['velocity_rmse_m_s']:.4f} m/s | "
            f"{metrics['attitude_rmse_deg']:.4f} deg | {metrics['healthy_ratio']:.3f} |"
        )
    lines.extend([
        "",
        f"Overall transport/health checks: **{'PASS' if passed else 'FAIL'}**.",
        "",
        "A passing best-effort row only proves numerical execution under the declared age gate. "
        "It must not be read as a latency-compensation claim. FCOne v2 must preserve physical "
        "timestamps and reject delayed aiding until a separately validated delayed-state design exists.",
    ])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if not passed:
        raise SystemExit("aiding delay sensitivity transport/health gate failed")
    print(f"Aiding delay sensitivity passed -> {out_dir}")


if __name__ == "__main__":
    main()
