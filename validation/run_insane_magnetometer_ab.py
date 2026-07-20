#!/usr/bin/env python3
"""Run a reproducible INSANE magnetometer-on versus magnetometer-off replay pair."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_COLUMNS = (
    "seq", "ts_us", "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z",
    "mag_valid", "mag_update", "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z",
    "magnetic_declination_rad",
)
MAG_FLAG_COLUMNS = ("mag_valid", "mag_update")
TRUTH_USAGE = (
    "The same reference attitude is used once to initialize both arms via "
    "--reference-attitude-init. After initialization the reference is used only for offline "
    "scoring and magnetic-field audit, never as ongoing filter aiding. This is tracking/fusion "
    "robustness evidence, not cold-start absolute-heading evidence."
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_line_count(path: Path) -> int:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return sum(1 for _ in stream)


def _read_rows(path: Path) -> tuple[list[str], list[list[str]]]:
    if not path.is_file():
        raise ValueError(f"missing CSV: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        try:
            header = next(reader)
        except StopIteration as error:
            raise ValueError(f"empty CSV: {path}") from error
        rows = list(reader)
    if not header or len(set(header)) != len(header):
        raise ValueError(f"invalid or duplicate CSV header: {path}")
    missing = [name for name in REQUIRED_COLUMNS if name not in header]
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    if not rows:
        raise ValueError(f"CSV has no data rows: {path}")
    if any(len(row) != len(header) for row in rows):
        raise ValueError(f"inconsistent CSV row width: {path}")
    return header, rows


def create_paired_inputs(replay_path: Path, on_path: Path, off_path: Path) -> dict[str, Any]:
    if len({replay_path.resolve(), on_path.resolve(), off_path.resolve()}) != 3:
        raise ValueError("source, magnetometer-on, and magnetometer-off paths must be distinct")
    header, rows = _read_rows(replay_path)
    on_path.parent.mkdir(parents=True, exist_ok=True)
    off_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(replay_path, on_path)
    valid_index = header.index("mag_valid")
    update_index = header.index("mag_update")
    with off_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for source_row in rows:
            row = source_row.copy()
            row[valid_index] = "0"
            row[update_index] = "0"
            writer.writerow(row)
    audit = audit_paired_inputs(on_path, off_path)
    if sha256_file(replay_path) != sha256_file(on_path):
        raise RuntimeError("magnetometer-on arm is not a byte-identical replay copy")
    return audit


def audit_paired_inputs(on_path: Path, off_path: Path) -> dict[str, Any]:
    on_header, on_rows = _read_rows(on_path)
    off_header, off_rows = _read_rows(off_path)
    if on_header != off_header:
        raise ValueError("paired replay headers differ")
    if len(on_rows) != len(off_rows):
        raise ValueError("paired replay row counts differ")
    flag_indices = {on_header.index(name) for name in MAG_FLAG_COLUMNS}
    physical_updates = 0
    for row_index, (on_row, off_row) in enumerate(zip(on_rows, off_rows), start=2):
        for column_index, (on_value, off_value) in enumerate(zip(on_row, off_row)):
            if column_index in flag_indices:
                continue
            if on_value != off_value:
                raise ValueError(
                    f"paired invariant failed at row {row_index}, column "
                    f"{on_header[column_index]}"
                )
        if off_row[on_header.index("mag_valid")] != "0" \
                or off_row[on_header.index("mag_update")] != "0":
            raise ValueError(f"magnetometer-off flags are nonzero at row {row_index}")
        try:
            valid = int(float(on_row[on_header.index("mag_valid")])) != 0
            update = int(float(on_row[on_header.index("mag_update")])) != 0
        except ValueError as error:
            raise ValueError(f"invalid magnetometer flag at row {row_index}") from error
        physical_updates += int(valid and update)
    if physical_updates == 0:
        raise ValueError("magnetometer-on replay contains no physical valid updates")
    return {
        "data_rows": len(on_rows),
        "file_lines": len(on_rows) + 1,
        "physical_magnetometer_updates": physical_updates,
        "differing_columns": list(MAG_FLAG_COLUMNS),
        "all_other_fields_text_identical": True,
        "magnetometer_off_flags_all_zero": True,
    }


def _summary(values: np.ndarray) -> dict[str, float]:
    if len(values) == 0 or np.any(~np.isfinite(values)):
        raise ValueError("cannot summarize empty or non-finite values")
    return {
        "initial": float(values[0]),
        "median": float(np.median(values)),
        "p05": float(np.percentile(values, 5.0)),
        "p95": float(np.percentile(values, 95.0)),
        "minimum": float(np.min(values)),
        "maximum": float(np.max(values)),
        "final": float(values[-1]),
    }


def _rotate_body_to_reference(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    quaternion = quaternion / np.linalg.norm(quaternion, axis=1)[:, None]
    w, x, y, z = quaternion.T
    vx, vy, vz = vector.T
    return np.column_stack((
        (1.0 - 2.0 * (y * y + z * z)) * vx + 2.0 * (x * y - w * z) * vy
        + 2.0 * (x * z + w * y) * vz,
        2.0 * (x * y + w * z) * vx + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - w * x) * vz,
        2.0 * (x * z - w * y) * vx + 2.0 * (y * z + w * x) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    ))


def magnetic_field_audit(replay_path: Path) -> dict[str, Any]:
    header, rows = _read_rows(replay_path)
    indices = {name: header.index(name) for name in REQUIRED_COLUMNS}
    numeric = np.asarray([
        [float(row[indices[name]]) for name in REQUIRED_COLUMNS] for row in rows
    ], dtype=np.float64)
    if np.any(~np.isfinite(numeric)):
        raise ValueError("replay contains non-finite magnetic/reference values")
    by_name = {name: numeric[:, index] for index, name in enumerate(REQUIRED_COLUMNS)}
    timestamps = by_name["ts_us"]
    if len(timestamps) < 2 or np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("replay timestamps must be strictly increasing")
    mask = (by_name["mag_valid"] != 0.0) & (by_name["mag_update"] != 0.0)
    if not np.any(mask):
        raise ValueError("no physical magnetometer updates are available for audit")
    quaternion = np.column_stack([
        by_name[f"ref_q_{axis}"][mask] for axis in ("w", "x", "y", "z")
    ])
    norms = np.linalg.norm(quaternion, axis=1)
    if np.any(~np.isfinite(norms) | (norms <= 1.0e-9)) \
            or not np.allclose(norms, 1.0, atol=1.0e-3):
        raise ValueError("invalid reference quaternion at a physical magnetometer update")
    magnetic_body_ut = 0.01 * np.column_stack([
        by_name[f"raw_mag_cuT_{axis}"][mask] for axis in ("x", "y", "z")
    ])
    magnetic_reference_ut = _rotate_body_to_reference(quaternion, magnetic_body_ut)
    field_norm_ut = np.linalg.norm(magnetic_reference_ut, axis=1)
    horizontal_norm_ut = np.linalg.norm(magnetic_reference_ut[:, :2], axis=1)
    if np.any(~np.isfinite(field_norm_ut) | (field_norm_ut <= 1.0e-9)
              | (horizontal_norm_ut <= 1.0e-9)):
        raise ValueError("degenerate magnetic field at a physical update")
    heading_deg = np.degrees(np.arctan2(
        magnetic_reference_ut[:, 1], magnetic_reference_ut[:, 0]
    ))
    declared_datum_deg = np.degrees(by_name["magnetic_declination_rad"][mask])
    datum_change_deg = (
        declared_datum_deg - declared_datum_deg[0] + 180.0
    ) % 360.0 - 180.0
    if np.max(np.abs(datum_change_deg)) > 1.0e-6:
        raise ValueError("declared magnetic datum changes within the replay")
    datum_residual_deg = (
        heading_deg - declared_datum_deg + 180.0
    ) % 360.0 - 180.0
    drift_deg = (heading_deg - heading_deg[0] + 180.0) % 360.0 - 180.0
    inclination_deg = np.degrees(np.arctan2(
        magnetic_reference_ut[:, 2], horizontal_norm_ut
    ))
    drift_summary = _summary(drift_deg)
    drift_summary["p95_absolute"] = float(np.percentile(np.abs(drift_deg), 95.0))
    datum_summary = _summary(datum_residual_deg)
    datum_summary["p95_absolute"] = float(
        np.percentile(np.abs(datum_residual_deg), 95.0)
    )
    return {
        "physical_magnetometer_updates": int(np.count_nonzero(mask)),
        "reference_frame": "replay reference NED/local-NED frame from ref_q body-to-reference",
        "heading_reference": "first physical valid magnetometer update",
        "horizontal_magnetic_heading_reference_deg": float(heading_deg[0]),
        "declared_magnetic_datum_deg": float(declared_datum_deg[0]),
        "declared_datum_residual_deg": datum_summary,
        "horizontal_magnetic_heading_relative_drift_deg": drift_summary,
        "field_norm_ut": _summary(field_norm_ut),
        "inclination_deg_positive_down": _summary(inclination_deg),
    }


def _run(command: list[str], log_path: Path) -> None:
    completed = subprocess.run(
        command, cwd=ROOT, check=False, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed with status {completed.returncode}: {' '.join(command)}\n"
            f"{completed.stdout[-2000:]}"
        )


def strict_json_value(value: object) -> object:
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    if isinstance(value, dict):
        return {str(key): strict_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [strict_json_value(item) for item in value]
    return value


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant in analyzer metrics: {value}")


def run_ab(
    replay_path: Path,
    runner_path: Path,
    out_dir: Path,
    *,
    analyzer_path: Path | None = None,
    scenario: str = "insane-magnetometer-ab",
) -> dict[str, Any]:
    replay_path = replay_path.resolve()
    runner_path = runner_path.resolve()
    analyzer_path = (analyzer_path or ROOT / "validation/analyze_results.py").resolve()
    out_dir = out_dir.resolve()
    if not replay_path.is_file():
        raise ValueError(f"missing replay: {replay_path}")
    if not runner_path.is_file() or not os.access(runner_path, os.X_OK):
        raise ValueError(f"native runner is missing or not executable: {runner_path}")
    if not analyzer_path.is_file():
        raise ValueError(f"missing analyzer: {analyzer_path}")
    out_dir.mkdir(parents=True, exist_ok=True)
    on_dir = out_dir / "magnetometer-on"
    off_dir = out_dir / "magnetometer-off"
    on_dir.mkdir(parents=True, exist_ok=True)
    off_dir.mkdir(parents=True, exist_ok=True)
    on_replay = on_dir / "replay.csv"
    off_replay = off_dir / "replay.csv"
    if replay_path in (on_replay, off_replay):
        raise ValueError("output directory would overwrite the supplied replay")
    pair_audit = create_paired_inputs(replay_path, on_replay, off_replay)
    field_audit = magnetic_field_audit(on_replay)
    if field_audit["physical_magnetometer_updates"] != pair_audit["physical_magnetometer_updates"]:
        raise RuntimeError("paired-input and field-audit magnetometer update counts differ")

    arms: dict[str, dict[str, Any]] = {}
    commands: list[list[str]] = []
    for arm_name, arm_dir, arm_replay in (
        ("magnetometer_on", on_dir, on_replay),
        ("magnetometer_off", off_dir, off_replay),
    ):
        results_path = arm_dir / "results.csv"
        report_dir = arm_dir / "analysis"
        metrics_path = report_dir / "metrics.json"
        for stale_path in (results_path, metrics_path, arm_dir / "runner.log", arm_dir / "analyzer.log"):
            if stale_path.exists():
                stale_path.unlink()
        runner_command = [
            str(runner_path), "--reference-attitude-init", str(arm_replay), str(results_path)
        ]
        analyzer_command = [
            sys.executable, str(analyzer_path), str(results_path),
            "--out-dir", str(report_dir), "--scenario", f"{scenario}-{arm_name}",
            "--reference-kind", "independent_truth", "--no-plots",
        ]
        commands.extend((runner_command, analyzer_command))
        _run(runner_command, arm_dir / "runner.log")
        if not results_path.is_file() or file_line_count(results_path) != pair_audit["file_lines"]:
            raise RuntimeError(f"{arm_name} results row count does not match replay")
        _run(analyzer_command, arm_dir / "analyzer.log")
        if not metrics_path.is_file():
            raise RuntimeError(f"analyzer did not produce metrics for {arm_name}")
        metrics = json.loads(
            metrics_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant
        )
        if not isinstance(metrics, dict) or metrics.get("reference_kind") != "independent_truth":
            raise RuntimeError(f"invalid analyzer metrics for {arm_name}")
        if int(metrics.get("samples", -1)) != pair_audit["data_rows"]:
            raise RuntimeError(f"analyzer sample count mismatch for {arm_name}")
        arms[arm_name] = {
            "replay_sha256": sha256_file(arm_replay),
            "replay_file_lines": file_line_count(arm_replay),
            "results_sha256": sha256_file(results_path),
            "results_file_lines": file_line_count(results_path),
            "metrics_sha256": sha256_file(metrics_path),
            "metrics": metrics,
            "runner_command": runner_command,
            "analyzer_command": analyzer_command,
        }

    output: dict[str, Any] = {
        "schema_version": "aerakia.insane_magnetometer_ab.v1",
        "status": "completed",
        "scenario": scenario,
        "scope": "paired physical INSANE magnetometer tracking/fusion robustness diagnostic",
        "truth_usage": TRUTH_USAGE,
        "truth_never_enters_initialization": False,
        "cold_start_absolute_heading_evidence": False,
        "pair_audit": pair_audit,
        "magnetic_field_audit": field_audit,
        "arms": arms,
        "provenance": {
            "source_replay": str(replay_path),
            "source_replay_sha256": sha256_file(replay_path),
            "source_replay_file_lines": file_line_count(replay_path),
            "native_runner": str(runner_path),
            "native_runner_sha256": sha256_file(runner_path),
            "analyzer": str(analyzer_path),
            "analyzer_sha256": sha256_file(analyzer_path),
            "ab_script_sha256": sha256_file(Path(__file__).resolve()),
            "commands": commands,
        },
    }
    serialized = json.dumps(
        strict_json_value(output), indent=2, sort_keys=True, allow_nan=False
    ) + "\n"
    (out_dir / "summary.json").write_text(serialized, encoding="utf-8")
    drift = field_audit["horizontal_magnetic_heading_relative_drift_deg"]
    datum = field_audit["declared_datum_residual_deg"]
    norm = field_audit["field_norm_ut"]
    inclination = field_audit["inclination_deg_positive_down"]
    report_lines = [
        "# INSANE magnetometer on/off A/B", "",
        f"Physical valid magnetometer updates: **{pair_audit['physical_magnetometer_updates']}**.", "",
        f"Initial physical-heading minus declared-datum residual: **{datum['initial']:.6f} deg**.", "",
        TRUTH_USAGE, "",
        "The no-magnetometer replay is mechanically derived; only `mag_valid` and `mag_update` "
        "differ from the byte-identical magnetometer-on replay source.", "",
        "| Audit | Median | P95 / P95 abs | Minimum | Maximum | Final |", 
        "| --- | ---: | ---: | ---: | ---: | ---: |",
        f"| Horizontal magnetic-heading relative drift (deg) | {drift['median']:.6f} | "
        f"{drift['p95_absolute']:.6f} | {drift['minimum']:.6f} | {drift['maximum']:.6f} | "
        f"{drift['final']:.6f} |",
        f"| Physical heading minus declared datum (deg) | {datum['median']:.6f} | "
        f"{datum['p95_absolute']:.6f} | {datum['minimum']:.6f} | "
        f"{datum['maximum']:.6f} | {datum['final']:.6f} |",
        f"| Field norm (uT) | {norm['median']:.6f} | {norm['p95']:.6f} | "
        f"{norm['minimum']:.6f} | {norm['maximum']:.6f} | {norm['final']:.6f} |",
        f"| Inclination, positive down (deg) | {inclination['median']:.6f} | "
        f"{inclination['p95']:.6f} | {inclination['minimum']:.6f} | "
        f"{inclination['maximum']:.6f} | {inclination['final']:.6f} |", "",
    ]
    if all("algorithms" in arm["metrics"] for arm in arms.values()):
        report_lines.extend([
            "| Arm / algorithm | Geodesic RMSE (deg) | Tilt RMSE (deg) | Yaw RMSE (deg) |", 
            "| --- | ---: | ---: | ---: |",
        ])
        for arm_name in ("magnetometer_off", "magnetometer_on"):
            for algorithm in ("eskf", "mahony_robust"):
                metrics = arms[arm_name]["metrics"]["algorithms"][algorithm]
                report_lines.append(
                    f"| {arm_name} / {algorithm} | "
                    f"{metrics['overall_attitude_rmse_deg']:.6f} | "
                    f"{metrics['tilt_rmse_deg']:.6f} | "
                    f"{metrics['axes']['yaw']['rmse_deg']:.6f} |"
                )
        report_lines.extend([
            "",
            "The pair isolates the effect of accepted physical magnetometer updates after a shared "
            "one-time reference-attitude initialization. It does not measure causal cold start or "
            "unaided navigation capability.",
            "",
        ])
    report_lines.extend([
        "Complete commands, hashes, row counts, paired invariants, and analyzer metrics are in "
        "`summary.json`.",
    ])
    (out_dir / "report.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay_csv", type=Path)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--analyzer", type=Path, default=ROOT / "validation/analyze_results.py")
    parser.add_argument("--scenario", default="insane-magnetometer-ab")
    args = parser.parse_args()
    run_ab(
        args.replay_csv, args.runner, args.out_dir,
        analyzer_path=args.analyzer, scenario=args.scenario,
    )
    print(f"INSANE magnetometer A/B report: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
