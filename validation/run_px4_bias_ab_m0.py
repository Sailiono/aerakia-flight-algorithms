#!/usr/bin/env python3
"""Fail-closed M0 preflight and canonical-output comparison for PX4 bias A/B."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


class BlockedError(RuntimeError):
    """A required local dependency or artifact is unavailable."""


class ProtocolError(RuntimeError):
    """An artifact was supplied but violated the frozen protocol."""


HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^[0-9a-f]{64}$")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProtocolError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ProtocolError(f"JSON root must be an object: {path}")
    return value


def protocol_fingerprint(manifest_path: Path, schema_path: Path, runner_path: Path) -> str:
    digest = hashlib.sha256()
    for path in (manifest_path, schema_path, runner_path):
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def validate_manifest(manifest: dict[str, Any], schema: dict[str, Any]) -> None:
    if manifest.get("schema_version") != 1 or schema.get("schema_version") != 1:
        raise ProtocolError("only PX4 bias A/B schema version 1 is supported")
    commit = manifest.get("px4_commit")
    if not isinstance(commit, str) or not HEX40.fullmatch(commit):
        raise ProtocolError("manifest px4_commit must be one lowercase 40-character SHA")
    required_files = manifest.get("required_px4_files")
    if not isinstance(required_files, dict) or not required_files:
        raise ProtocolError("manifest must freeze required_px4_files")
    if any(not isinstance(path, str) or not isinstance(digest, str)
           or not HEX64.fullmatch(digest) for path, digest in required_files.items()):
        raise ProtocolError("every frozen PX4 source file needs a SHA-256 digest")
    profiles = manifest.get("profiles")
    if not isinstance(profiles, dict) or "stock" not in profiles:
        raise ProtocolError("manifest must contain the stock comparison profile")
    for estimator in ("aerakia", "px4"):
        if not isinstance(profiles["stock"].get(estimator), dict):
            raise ProtocolError(f"stock profile is missing {estimator} configuration")
    comparison = manifest.get("comparison", {})
    for name in ("bias_vector_error_limit_m_s2", "convergence_budget_s",
                 "minimum_input_duration_s", "settling_hold_s"):
        value = comparison.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ProtocolError(f"comparison.{name} must be finite and positive")
    for group in ("input", "output"):
        section = schema.get(group)
        if not isinstance(section, dict) or not section.get("required_columns"):
            raise ProtocolError(f"schema is missing {group}.required_columns")


def run_git(source: Path, *arguments: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(source), *arguments],
            check=False, capture_output=True, text=True, timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BlockedError(f"cannot inspect local PX4 Git tree: {error}") from error
    if result.returncode != 0:
        raise BlockedError(
            f"local PX4 dependency is not an inspectable Git tree: {result.stderr.strip()}"
        )
    return result.stdout.strip()


def validate_px4_source(source: Path | None, manifest: dict[str, Any]) -> dict[str, Any]:
    if source is None:
        raise BlockedError("--px4-source is required; network download is intentionally unsupported")
    if not source.is_dir():
        raise BlockedError(f"local PX4 source directory is missing: {source}")
    actual_commit = run_git(source, "rev-parse", "HEAD")
    expected_commit = manifest["px4_commit"]
    if actual_commit != expected_commit:
        raise ProtocolError(
            f"local PX4 SHA {actual_commit} does not match frozen SHA {expected_commit}"
        )
    dirty = run_git(source, "status", "--porcelain", "--untracked-files=no")
    if dirty:
        raise ProtocolError("frozen PX4 source has tracked modifications")
    verified: dict[str, str] = {}
    for relative, expected_hash in manifest["required_px4_files"].items():
        path = source / relative
        if not path.is_file():
            raise BlockedError(f"frozen PX4 source is incomplete: missing {relative}")
        actual_hash = sha256_file(path)
        if actual_hash != expected_hash:
            raise ProtocolError(
                f"PX4 source hash mismatch for {relative}: {actual_hash} != {expected_hash}"
            )
        verified[relative] = actual_hash
    return {"path": str(source.resolve()), "commit": actual_commit, "files": verified}


def parse_finite(value: str, column: str, row_number: int) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise ProtocolError(f"row {row_number} column {column} is not numeric") from error
    if not math.isfinite(parsed):
        raise ProtocolError(f"row {row_number} column {column} is non-finite")
    return parsed


def read_canonical_csv(path: Path, section: dict[str, Any]) -> list[dict[str, float]]:
    if not path.is_file():
        raise BlockedError(f"required canonical CSV is missing: {path}")
    required = list(section["required_columns"])
    boolean_columns = set(section.get("boolean_columns", []))
    positive_columns = set(section.get("positive_columns", []))
    nonnegative_columns = set(section.get("nonnegative_columns", []))
    time_column = section["time_column"]
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames or []
        missing = [column for column in required if column not in header]
        if missing:
            raise ProtocolError(f"{path} is missing required columns: {', '.join(missing)}")
        previous_time: int | None = None
        for row_number, raw in enumerate(reader, start=2):
            parsed = {column: parse_finite(raw[column], column, row_number) for column in required}
            timestamp_float = parsed[time_column]
            timestamp = int(timestamp_float)
            if timestamp_float != timestamp or timestamp < 0:
                raise ProtocolError(f"row {row_number} has a non-integral timestamp")
            if previous_time is not None and timestamp <= previous_time:
                raise ProtocolError(f"row {row_number} timestamp is not strictly increasing")
            previous_time = timestamp
            for column in boolean_columns:
                if parsed[column] not in (0.0, 1.0):
                    raise ProtocolError(f"row {row_number} column {column} must be 0 or 1")
            for column in positive_columns:
                if parsed[column] <= 0.0:
                    raise ProtocolError(f"row {row_number} column {column} must be positive")
            for column in nonnegative_columns:
                if parsed[column] < 0.0:
                    raise ProtocolError(f"row {row_number} column {column} must be nonnegative")
            rows.append(parsed)
    if not rows:
        raise ProtocolError(f"canonical CSV has no samples: {path}")
    return rows


def validate_input_metadata(metadata: dict[str, Any], input_path: Path, row_count: int) -> None:
    required = ("schema_version", "input_sha256", "row_count", "trajectory_id",
                "bias_case_id", "generator_commit", "generator_sha256")
    missing = [name for name in required if name not in metadata]
    if missing:
        raise ProtocolError(f"input metadata is missing: {', '.join(missing)}")
    if metadata["schema_version"] != 1:
        raise ProtocolError("input metadata schema_version must be 1")
    if metadata["input_sha256"] != sha256_file(input_path):
        raise ProtocolError("input metadata hash does not match canonical input CSV")
    if metadata["row_count"] != row_count:
        raise ProtocolError("input metadata row_count does not match canonical input CSV")
    if not HEX40.fullmatch(str(metadata["generator_commit"])):
        raise ProtocolError("input generator_commit must be a 40-character SHA")
    if not HEX64.fullmatch(str(metadata["generator_sha256"])):
        raise ProtocolError("input generator_sha256 must be SHA-256")


def validate_output_metadata(
    metadata: dict[str, Any], estimator: str, output_path: Path, row_count: int,
    input_hash: str, fingerprint: str, profile: str, manifest: dict[str, Any],
    required_fields: list[str],
) -> None:
    missing = [name for name in required_fields if name not in metadata]
    if missing:
        raise ProtocolError(f"{estimator} metadata is missing: {', '.join(missing)}")
    expected_config_hash = canonical_sha256(manifest["profiles"][profile][estimator])
    expected = {
        "schema_version": 1,
        "estimator_id": estimator,
        "source_dirty": False,
        "input_sha256": input_hash,
        "output_sha256": sha256_file(output_path),
        "protocol_fingerprint": fingerprint,
        "profile": profile,
        "config_sha256": expected_config_hash,
        "output_time_semantics": manifest["output_time_semantics"],
        "row_count": row_count,
    }
    for name, value in expected.items():
        if metadata.get(name) != value:
            raise ProtocolError(f"{estimator} metadata mismatch for {name}")
    if not HEX40.fullmatch(str(metadata["source_commit"])):
        raise ProtocolError(f"{estimator} source_commit must be a 40-character SHA")
    if estimator == "px4" and metadata["source_commit"] != manifest["px4_commit"]:
        raise ProtocolError("PX4 output metadata was not produced by the frozen commit")
    if not HEX64.fullmatch(str(metadata["runner_sha256"])):
        raise ProtocolError(f"{estimator} runner_sha256 must be SHA-256")


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def bias_metrics(
    output: list[dict[str, float]], truth_by_time: dict[int, tuple[float, float, float]],
    start_us: int, error_limit: float, hold_s: float,
) -> dict[str, Any]:
    errors: list[float] = []
    nees: list[float] = []
    covered = 0
    false_confidence = 0
    healthy = 0
    valid_estimates = 0
    valid_covariance_samples = 0
    inhibited = 0
    inhibit_supported = 0
    timestamps: list[int] = []
    estimate_validity: list[bool] = []
    for row in output:
        timestamp = int(row["fusion_horizon_timestamp_us"])
        if timestamp not in truth_by_time:
            raise ProtocolError(f"output timestamp {timestamp} is absent from canonical input")
        truth = truth_by_time[timestamp]
        error = tuple(row[f"accel_bias_{axis}_m_s2"] - truth[index]
                      for index, axis in enumerate(("x", "y", "z")))
        variance = tuple(row[f"accel_bias_variance_{axis}_m2_s4"]
                         for axis in ("x", "y", "z"))
        norm = math.sqrt(sum(value * value for value in error))
        errors.append(norm)
        timestamps.append(timestamp)
        estimate_valid = row["accel_bias_valid"] == 1.0
        covariance_valid = estimate_valid and all(value > 0.0 for value in variance)
        estimate_validity.append(estimate_valid)
        valid_estimates += int(estimate_valid)
        valid_covariance_samples += int(covariance_valid)
        axis_covered = covariance_valid and all(
            abs(error[index]) <= 3.0 * math.sqrt(variance[index]) for index in range(3)
        )
        covered += int(axis_covered)
        false_confidence += int(covariance_valid and norm > error_limit and not axis_covered)
        if covariance_valid:
            nees.append(sum(error[index] * error[index] / variance[index] for index in range(3)))
        healthy += int(row["healthy"] == 1.0)
        if row["accel_bias_learning_inhibit_supported"] == 1.0:
            inhibit_supported += 1
            inhibited += int(row["accel_bias_learning_inhibited"] == 1.0)

    settling_time_s: float | None = None
    end_us = timestamps[-1]
    hold_us = int(round(hold_s * 1.0e6))
    for index, timestamp in enumerate(timestamps):
        if (
            end_us - timestamp >= hold_us
            and all(estimate_validity[index:])
            and all(value <= error_limit for value in errors[index:])
        ):
            settling_time_s = (timestamp - start_us) * 1.0e-6
            break
    return {
        "samples": len(errors),
        "bias_error_terminal_m_s2": errors[-1],
        "bias_error_rmse_m_s2": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "bias_error_p95_m_s2": percentile(errors, 0.95),
        "settling_time_s": settling_time_s,
        "accel_bias_valid_ratio": valid_estimates / len(errors),
        "valid_covariance_samples": valid_covariance_samples,
        "three_sigma_coverage": (
            covered / valid_covariance_samples if valid_covariance_samples else None
        ),
        "bias_nees_mean": sum(nees) / len(nees) if nees else None,
        "false_confidence_samples": false_confidence,
        "healthy_ratio": healthy / len(errors),
        "learning_inhibit_fraction": inhibited / inhibit_supported if inhibit_supported else None,
    }


def compare_exports(
    input_rows: list[dict[str, float]], aerakia_rows: list[dict[str, float]],
    px4_rows: list[dict[str, float]], manifest: dict[str, Any],
) -> dict[str, Any]:
    aerakia_times = [int(row["fusion_horizon_timestamp_us"]) for row in aerakia_rows]
    px4_times = [int(row["fusion_horizon_timestamp_us"]) for row in px4_rows]
    tolerance = int(manifest["comparison"].get("time_alignment_tolerance_us", 0))
    if len(aerakia_times) != len(px4_times):
        raise ProtocolError("canonical estimator outputs have different row counts")
    if any(abs(left - right) > tolerance for left, right in zip(aerakia_times, px4_times)):
        raise ProtocolError("canonical estimator outputs are not aligned at one fusion horizon")
    start_us = int(input_rows[0]["timestamp_us"])
    duration_s = (int(input_rows[-1]["timestamp_us"]) - start_us) * 1.0e-6
    minimum_duration = float(manifest["comparison"]["minimum_input_duration_s"])
    if duration_s < minimum_duration:
        raise ProtocolError(f"input duration {duration_s:.6f}s is below {minimum_duration:.6f}s")
    truth = {
        int(row["timestamp_us"]): (
            row["truth_accel_bias_x_m_s2"], row["truth_accel_bias_y_m_s2"],
            row["truth_accel_bias_z_m_s2"],
        ) for row in input_rows
    }
    limit = float(manifest["comparison"]["bias_vector_error_limit_m_s2"])
    hold = float(manifest["comparison"]["settling_hold_s"])
    budget = float(manifest["comparison"]["convergence_budget_s"])
    result = {
        "duration_s": duration_s,
        "aerakia": bias_metrics(aerakia_rows, truth, start_us, limit, hold),
        "px4": bias_metrics(px4_rows, truth, start_us, limit, hold),
    }
    for metrics in (result["aerakia"], result["px4"]):
        settling = metrics["settling_time_s"]
        metrics["absolute_convergence_budget_met"] = settling is not None and settling <= budget
    return result


def write_run_manifest(out_dir: Path, record: dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "run-manifest.json").write_text(
        json.dumps(record, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path("validation/px4_bias_ab_m0_manifest.json"))
    parser.add_argument("--px4-source", type=Path)
    parser.add_argument("--profile", default="stock")
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--input-metadata", type=Path)
    parser.add_argument("--aerakia-output", type=Path)
    parser.add_argument("--aerakia-metadata", type=Path)
    parser.add_argument("--px4-output", type=Path)
    parser.add_argument("--px4-metadata", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("build/px4-bias-ab-m0"))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest
    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    record: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "claim_status": "no_px4_parity_claim",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    try:
        manifest = load_json(manifest_path)
        schema_path_raw = Path(manifest.get("schema_path", ""))
        schema_path = schema_path_raw if schema_path_raw.is_absolute() else root / schema_path_raw
        schema = load_json(schema_path)
        validate_manifest(manifest, schema)
        if args.profile not in manifest["profiles"]:
            raise ProtocolError(f"profile is not frozen in manifest: {args.profile}")
        fingerprint = protocol_fingerprint(manifest_path, schema_path, Path(__file__).resolve())
        record.update({
            "protocol_id": manifest["protocol_id"],
            "protocol_fingerprint": fingerprint,
            "manifest_sha256": sha256_file(manifest_path),
            "schema_sha256": sha256_file(schema_path),
            "profile": args.profile,
            "claim_boundary": manifest["claim_boundary"],
        })
        record["px4_source"] = validate_px4_source(args.px4_source, manifest)
        artifacts = (args.input_csv, args.input_metadata, args.aerakia_output,
                     args.aerakia_metadata, args.px4_output, args.px4_metadata)
        if any(path is None for path in artifacts):
            raise BlockedError(
                "canonical input, both estimator exports, and all metadata sidecars are required; "
                "M0 does not fabricate or substitute estimator output"
            )
        input_path, input_metadata_path, aerakia_path, aerakia_metadata_path, px4_path, px4_metadata_path = artifacts
        input_rows = read_canonical_csv(input_path, schema["input"])
        aerakia_rows = read_canonical_csv(aerakia_path, schema["output"])
        px4_rows = read_canonical_csv(px4_path, schema["output"])
        input_metadata = load_json(input_metadata_path)
        validate_input_metadata(input_metadata, input_path, len(input_rows))
        input_hash = sha256_file(input_path)
        required_metadata = list(schema["metadata"]["required_fields"])
        validate_output_metadata(
            load_json(aerakia_metadata_path), "aerakia", aerakia_path, len(aerakia_rows),
            input_hash, fingerprint, args.profile, manifest, required_metadata,
        )
        validate_output_metadata(
            load_json(px4_metadata_path), "px4", px4_path, len(px4_rows), input_hash,
            fingerprint, args.profile, manifest, required_metadata,
        )
        record["comparison"] = compare_exports(input_rows, aerakia_rows, px4_rows, manifest)
        record["input"] = {
            "path": str(input_path), "sha256": input_hash,
            "rows": len(input_rows), "metadata": input_metadata,
        }
        record["status"] = "completed"
        return_code = 0
    except BlockedError as error:
        record["status"] = "blocked"
        record["reason"] = str(error)
        return_code = 2
    except (ProtocolError, OSError, KeyError, TypeError, ValueError) as error:
        record["status"] = "failed"
        record["reason"] = str(error)
        return_code = 1
    finally:
        record["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        write_run_manifest(out_dir, record)
    print(json.dumps({"status": record["status"], "reason": record.get("reason")}, sort_keys=True))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
