#!/usr/bin/env python3
"""Run the M0 same-input gate on one deterministic no-delay synthetic event stream.

This is a transport, frame, delayed-horizon, and provenance sanity gate.  It
does not establish PX4-class accuracy, flight readiness, or a general
non-inferiority claim.  The only accepted comparison is between outputs made
from the same immutable generated CSV at the exact delayed-horizon timestamps
emitted by the official PX4 ecl_EKF runner.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
M0_PATH = ROOT / "validation" / "run_px4_bias_ab_m0.py"
BUILD_PX4_PATH = ROOT / "validation" / "build_px4_bias_ab_host.py"
M0_SPEC = importlib.util.spec_from_file_location("run_px4_bias_ab_m0", M0_PATH)
assert M0_SPEC is not None and M0_SPEC.loader is not None
M0 = importlib.util.module_from_spec(M0_SPEC)
M0_SPEC.loader.exec_module(M0)


class BlockedError(RuntimeError):
    """A required clean source or host dependency is unavailable."""


CANONICAL_FIELDS = (
    "timestamp_us",
    "delta_angle_dt_s", "delta_angle_x_rad", "delta_angle_y_rad", "delta_angle_z_rad",
    "delta_velocity_dt_s", "delta_velocity_x_m_s", "delta_velocity_y_m_s", "delta_velocity_z_m_s",
    "accel_clipping_x", "accel_clipping_y", "accel_clipping_z",
    "at_rest", "in_air", "in_transition",
    "gnss_position_update", "gnss_position_n_m", "gnss_position_e_m", "gnss_position_d_m",
    "gnss_position_variance_m2",
    "gnss_velocity_update", "gnss_velocity_n_m_s", "gnss_velocity_e_m_s", "gnss_velocity_d_m_s",
    "gnss_velocity_variance_m2_s2",
    "barometer_update", "barometer_height_up_m", "barometer_variance_m2",
    "magnetometer_update", "magnetometer_x_ut", "magnetometer_y_ut", "magnetometer_z_ut",
    "truth_accel_bias_x_m_s2", "truth_accel_bias_y_m_s2", "truth_accel_bias_z_m_s2",
    "truth_q_w", "truth_q_x", "truth_q_y", "truth_q_z",
    "truth_velocity_n_m_s", "truth_velocity_e_m_s", "truth_velocity_d_m_s",
    "truth_position_n_m", "truth_position_e_m", "truth_position_d_m",
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-source", type=Path, required=True)
    parser.add_argument(
        "--aerakia-source", type=Path, default=ROOT,
        help="clean Git worktree containing the Aerakia core sources",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("build/px4-bias-ab-m0-synthetic"))
    parser.add_argument("--px4-build-dir", type=Path)
    parser.add_argument(
        "--px4-python", type=Path,
        help="Python interpreter containing PX4's uORB-generator dependencies",
    )
    parser.add_argument("--duration-s", type=float, default=65.0)
    parser.add_argument("--rate-hz", type=int, default=100)
    return parser.parse_args(argv)


def sha256(path: Path) -> str:
    return M0.sha256_file(path)


def clean_git_commit(source: Path, label: str) -> str:
    try:
        commit = subprocess.run(
            ["git", "-C", str(source), "rev-parse", "HEAD"],
            check=False, text=True, capture_output=True, timeout=30,
        )
        status = subprocess.run(
            ["git", "-C", str(source), "status", "--porcelain", "--untracked-files=no"],
            check=False, text=True, capture_output=True, timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BlockedError(f"cannot inspect {label} Git tree: {error}") from error
    revision = commit.stdout.strip()
    if commit.returncode != 0 or len(revision) != 40:
        raise BlockedError(f"{label} is not an inspectable Git worktree")
    if status.returncode != 0 or status.stdout.strip():
        raise BlockedError(f"{label} must be clean for a completed M0 result")
    return revision


def run(command: list[str], *, cwd: Path) -> None:
    try:
        result = subprocess.run(command, cwd=cwd, check=False, text=True, capture_output=True, timeout=900)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BlockedError(f"cannot execute {' '.join(command)}: {error}") from error
    if result.returncode != 0:
        detail = (result.stdout + "\n" + result.stderr).strip()
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}\n{detail}")


def write_input(path: Path, *, duration_s: float, rate_hz: int) -> int:
    """Generate a physically consistent hover-first trajectory with a fixed accel bias."""
    if not math.isfinite(duration_s) or duration_s < 60.0 or rate_hz != 100:
        raise ValueError("M0 synthetic gate requires at least 60 s at 100 Hz")
    dt = 1.0 / rate_hz
    steps = int(round(duration_s * rate_hz)) + 1
    velocity = [0.0, 0.0, 0.0]
    position = [0.0, 0.0, 0.0]
    bias = (0.10, -0.07, 0.03)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CANONICAL_FIELDS)
        writer.writeheader()
        for index in range(steps):
            time_s = index * dt
            if time_s < 10.0:
                acceleration = (0.0, 0.0, 0.0)
            else:
                phase = time_s - 10.0
                acceleration = (
                    0.75 * math.sin(0.51 * phase) + 0.25 * math.sin(1.37 * phase),
                    0.65 * math.cos(0.43 * phase) - 0.20 * math.sin(1.11 * phase),
                    0.18 * math.sin(0.31 * phase),
                )
            if index > 0:
                for axis in range(3):
                    velocity[axis] += acceleration[axis] * dt
                    position[axis] += velocity[axis] * dt
            gnss_update = int(index % 10 == 0)
            row = {name: 0 for name in CANONICAL_FIELDS}
            row.update({
                "timestamp_us": index * int(round(dt * 1.0e6)),
                "delta_angle_dt_s": dt,
                "delta_velocity_dt_s": dt,
                "delta_velocity_x_m_s": (acceleration[0] + bias[0]) * dt,
                "delta_velocity_y_m_s": (acceleration[1] + bias[1]) * dt,
                # Static FRD specific force is -g in the PX4 and Aerakia contracts.
                "delta_velocity_z_m_s": (acceleration[2] - 9.80665 + bias[2]) * dt,
                "at_rest": int(time_s < 10.0),
                "in_air": int(time_s >= 10.0),
                "gnss_position_update": gnss_update,
                "gnss_position_n_m": position[0],
                "gnss_position_e_m": position[1],
                "gnss_position_d_m": position[2],
                "gnss_position_variance_m2": 0.25,
                "gnss_velocity_update": gnss_update,
                "gnss_velocity_n_m_s": velocity[0],
                "gnss_velocity_e_m_s": velocity[1],
                "gnss_velocity_d_m_s": velocity[2],
                "gnss_velocity_variance_m2_s2": 0.04,
                "barometer_update": gnss_update,
                "barometer_height_up_m": -position[2],
                "barometer_variance_m2": 0.25,
                "truth_accel_bias_x_m_s2": bias[0],
                "truth_accel_bias_y_m_s2": bias[1],
                "truth_accel_bias_z_m_s2": bias[2],
                "truth_q_w": 1.0,
                "truth_velocity_n_m_s": velocity[0],
                "truth_velocity_e_m_s": velocity[1],
                "truth_velocity_d_m_s": velocity[2],
                "truth_position_n_m": position[0],
                "truth_position_e_m": position[1],
                "truth_position_d_m": position[2],
            })
            writer.writerow(row)
    return steps


def build_aerakia_runner(source: Path, destination: Path) -> Path:
    compiler = shutil.which("cc")
    if compiler is None:
        raise BlockedError("a C99 compiler is required for the Aerakia M0 producer")
    runner = ROOT / "validation" / "aerakia_px4_bias_ab_host_runner.c"
    sources = [
        source / "src" / name for name in (
            "barometer_supervisor.c", "eskf.c", "eskf_adapter.c", "eskf_math.c",
            "eskf_models.c", "mag_gate.c", "mahony.c",
        )
    ]
    if not runner.is_file() or any(not item.is_file() for item in sources):
        raise BlockedError("Aerakia M0 producer or core source files are missing")
    destination.parent.mkdir(parents=True, exist_ok=True)
    run([
        compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-I", str(source / "include"), "-I", str(source / "src"), str(runner),
        *map(str, sources), "-lm", "-o", str(destination),
    ], cwd=ROOT)
    return destination


def count_rows(path: Path) -> int:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return sum(1 for _ in csv.DictReader(stream))


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def metadata(
    *, estimator: str, source_commit: str, runner: Path, input_path: Path,
    output_path: Path, rows: int, fingerprint: str, manifest: dict[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "estimator_id": estimator,
        "source_commit": source_commit,
        "source_dirty": False,
        "runner_sha256": sha256(runner),
        "input_sha256": sha256(input_path),
        "output_sha256": sha256(output_path),
        "protocol_fingerprint": fingerprint,
        "profile": "stock",
        "config_sha256": M0.canonical_sha256(manifest["profiles"]["stock"][estimator]),
        "output_time_semantics": manifest["output_time_semantics"],
        "row_count": rows,
    }


def write_report(path: Path, record: dict[str, Any]) -> None:
    comparison = record.get("comparison", {})
    lines = [
        "# PX4/Aerakia M0 Synthetic Same-Input Gate", "",
        f"Status: **{record['status']}**.", "",
        "This is a deterministic no-delay synthetic transport/frame/horizon sanity gate. It is not a PX4-class, flight-readiness, or general non-inferiority claim.",
        "",
    ]
    if comparison:
        lines.extend([
            "| Metric | Aerakia | PX4 |", "| --- | ---: | ---: |",
            f"| Bias terminal error (m/s2) | {comparison['aerakia']['bias_error_terminal_m_s2']:.6f} | {comparison['px4']['bias_error_terminal_m_s2']:.6f} |",
            f"| Bias RMSE (m/s2) | {comparison['aerakia']['bias_error_rmse_m_s2']:.6f} | {comparison['px4']['bias_error_rmse_m_s2']:.6f} |",
            f"| Settling time (s) | {comparison['aerakia']['settling_time_s']} | {comparison['px4']['settling_time_s']} |",
            "",
        ])
    lines.append("The event CSV, both canonical outputs, sidecars, and scorer run manifest retain SHA-256 provenance in this directory.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    px4_build_dir = args.px4_build_dir or (out_dir / "px4-build")
    px4_build_dir = px4_build_dir if px4_build_dir.is_absolute() else ROOT / px4_build_dir
    record: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "claim_status": "no_px4_parity_claim",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "scenario": "m0_synthetic_hover_first_no_delay_v1",
    }
    try:
        aerakia_commit = clean_git_commit(args.aerakia_source, "Aerakia source")
        manifest_path = ROOT / "validation" / "px4_bias_ab_m0_manifest.json"
        manifest = M0.load_json(manifest_path)
        schema_path = ROOT / manifest["schema_path"]
        M0.validate_manifest(manifest, M0.load_json(schema_path))
        px4_source = M0.validate_px4_source(args.px4_source, manifest)
        out_dir.mkdir(parents=True, exist_ok=True)
        input_csv = out_dir / "input.csv"
        input_rows = write_input(input_csv, duration_s=args.duration_s, rate_hz=args.rate_hz)
        input_metadata = {
            "schema_version": 1,
            "input_sha256": sha256(input_csv),
            "row_count": input_rows,
            "trajectory_id": "m0_synthetic_hover_first_no_delay_v1",
            "bias_case_id": "fixed_[+0.10,-0.07,+0.03]_m_s2",
            "generator_commit": aerakia_commit,
            "generator_sha256": sha256(Path(__file__).resolve()),
        }
        input_metadata_path = out_dir / "input.metadata.json"
        write_json(input_metadata_path, input_metadata)

        build_command = [
            sys.executable, str(BUILD_PX4_PATH), "--px4-source", str(args.px4_source),
            "--build-dir", str(px4_build_dir),
        ]
        if args.px4_python is not None:
            build_command.extend(["--python", str(args.px4_python)])
        run(build_command, cwd=ROOT)
        px4_runner = px4_build_dir / ("aerakia_px4_bias_ab_runner.exe" if sys.platform == "win32" else "aerakia_px4_bias_ab_runner")
        if not px4_runner.is_file():
            raise RuntimeError("verified PX4 build did not produce the host replay executable")
        px4_output = out_dir / "px4.csv"
        run([str(px4_runner), str(input_csv), str(px4_output)], cwd=ROOT)

        aerakia_runner = build_aerakia_runner(args.aerakia_source, out_dir / "aerakia_runner")
        aerakia_output = out_dir / "aerakia.csv"
        run([str(aerakia_runner), str(input_csv), str(px4_output), str(aerakia_output)], cwd=ROOT)

        fingerprint = M0.protocol_fingerprint(manifest_path, schema_path, M0_PATH)
        px4_metadata_path = out_dir / "px4.metadata.json"
        aerakia_metadata_path = out_dir / "aerakia.metadata.json"
        write_json(px4_metadata_path, metadata(
            estimator="px4", source_commit=manifest["px4_commit"], runner=px4_runner,
            input_path=input_csv, output_path=px4_output, rows=count_rows(px4_output),
            fingerprint=fingerprint, manifest=manifest,
        ))
        write_json(aerakia_metadata_path, metadata(
            estimator="aerakia", source_commit=aerakia_commit, runner=aerakia_runner,
            input_path=input_csv, output_path=aerakia_output, rows=count_rows(aerakia_output),
            fingerprint=fingerprint, manifest=manifest,
        ))
        scorer_dir = out_dir / "scorer"
        scorer_status = M0.main([
            "--px4-source", str(args.px4_source), "--input-csv", str(input_csv),
            "--input-metadata", str(input_metadata_path), "--aerakia-output", str(aerakia_output),
            "--aerakia-metadata", str(aerakia_metadata_path), "--px4-output", str(px4_output),
            "--px4-metadata", str(px4_metadata_path), "--out-dir", str(scorer_dir),
        ])
        scorer_record = M0.load_json(scorer_dir / "run-manifest.json")
        if scorer_status != 0 or scorer_record["status"] != "completed":
            raise RuntimeError(f"M0 scorer did not complete: {scorer_record.get('reason')}")
        record.update({
            "status": "completed", "px4_source": px4_source,
            "aerakia_source_commit": aerakia_commit,
            "input_sha256": sha256(input_csv), "comparison": scorer_record["comparison"],
            "scorer_manifest": str((scorer_dir / "run-manifest.json").resolve()),
        })
        return_code = 0
    except (BlockedError, M0.BlockedError) as error:
        record.update({"status": "blocked", "reason": str(error)})
        return_code = 2
    except (RuntimeError, M0.ProtocolError, OSError, ValueError, KeyError) as error:
        record.update({"status": "failed", "reason": str(error)})
        return_code = 1
    finally:
        record["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        out_dir.mkdir(parents=True, exist_ok=True)
        write_json(out_dir / "run-manifest.json", record)
        write_report(out_dir / "report.md", record)
    print(json.dumps({"status": record["status"], "reason": record.get("reason")}, sort_keys=True))
    return return_code


if __name__ == "__main__":
    sys.exit(main())
