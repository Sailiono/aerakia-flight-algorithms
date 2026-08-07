#!/usr/bin/env python3
"""Run paired IMU-only versus IMU+barometer vertical-outage trials."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path
from statistics import mean

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "tools"))
import generate_synthetic_imu as synthetic  # noqa: E402


FAULTS = ("nominal", "constant_bias", "random_walk", "weather_step", "freeze", "delay")


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def capture(command: list[str]) -> str | None:
    completed = subprocess.run(
        command, cwd=ROOT, check=False, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def vertical_truth(time_s: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Bounded periodic climb/descent after a two-second stationary prefix."""
    tau = np.maximum(time_s - 2.0, 0.0)
    active = time_s >= 2.0
    amplitude_m = 2.0
    omega_rad_s = 2.0 * math.pi / 30.0
    position = np.zeros((len(time_s), 3), dtype=np.float64)
    velocity = np.zeros_like(position)
    acceleration = np.zeros_like(position)
    position[active, 2] = -amplitude_m * (1.0 - np.cos(omega_rad_s * tau[active]))
    velocity[active, 2] = -amplitude_m * omega_rad_s * np.sin(omega_rad_s * tau[active])
    acceleration[active, 2] = (
        -amplitude_m * omega_rad_s * omega_rad_s * np.cos(omega_rad_s * tau[active])
    )
    return position, velocity, acceleration


def barometer_stream(
    time_s: np.ndarray,
    true_height_up_m: np.ndarray,
    *,
    fault: str,
    outage_start_s: float,
    outage_duration_s: float,
    rate_hz: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, object]]:
    if fault not in FAULTS:
        raise ValueError(f"unsupported barometer fault: {fault}")
    update_period = max(1, int(round(rate_hz / 20.0)))
    updates = (np.arange(len(time_s)) % update_period == 0)
    noise_std_m = 0.40
    error = rng.normal(0.0, noise_std_m, len(time_s))
    if fault == "constant_bias":
        error += 1.0
    elif fault == "random_walk":
        density_m_sqrt_s = 0.03
        error += np.cumsum(
            rng.normal(0.0, density_m_sqrt_s / math.sqrt(rate_hz), len(time_s))
        )
    elif fault == "weather_step":
        error[time_s >= outage_start_s + 0.5 * outage_duration_s] += 2.0

    measured = true_height_up_m + error
    if fault == "freeze":
        start_s = outage_start_s + 0.25 * outage_duration_s
        stop_s = outage_start_s + 0.75 * outage_duration_s
        start = int(np.searchsorted(time_s, start_s))
        stop = int(np.searchsorted(time_s, stop_s))
        if start > 0 and stop > start:
            measured[start:stop] = measured[start - 1]
    sample_timestamp_us = np.rint(time_s * 1.0e6).astype(np.int64)
    if fault == "delay":
        delay_s = 0.60
        delay_samples = max(1, int(round(delay_s * rate_hz)))
        measured[delay_samples:] = measured[:-delay_samples]
        measured[:delay_samples] = measured[0]
        sample_timestamp_us[delay_samples:] = sample_timestamp_us[:-delay_samples]
        updates[:delay_samples] = False

    return (
        updates.astype(np.int64),
        sample_timestamp_us,
        measured,
        np.full(len(time_s), noise_std_m * noise_std_m, dtype=np.float64),
        {
            "fault": fault,
            "noise_std_m": noise_std_m,
            "residual_constant_bias_m": 1.0 if fault == "constant_bias" else 0.0,
            "random_walk_density_m_sqrt_s": 0.03 if fault == "random_walk" else 0.0,
            "weather_step_m": 2.0 if fault == "weather_step" else 0.0,
            "delay_s": 0.60 if fault == "delay" else 0.0,
            "fault_onset_s": (
                outage_start_s + 0.5 * outage_duration_s
                if fault == "weather_step"
                else outage_start_s + 0.25 * outage_duration_s
                if fault == "freeze"
                else 0.0
                if fault in ("constant_bias", "delay")
                else None
            ),
        },
    )


def write_base_input(
    path: Path,
    *,
    outage_duration_s: float,
    rate_hz: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    outage_start_s = 8.0
    duration_s = outage_start_s + outage_duration_s + 10.0
    sample_count = int(round(duration_s * rate_hz))
    time_s = np.arange(sample_count, dtype=np.float64) / rate_hz
    position, velocity, acceleration = vertical_truth(time_s)
    zeros = np.zeros(sample_count, dtype=np.float64)
    rng = np.random.default_rng(seed)
    accel, gyro, magnetic, ideal_accel = synthetic.synthesize_measurements(
        time_s, zeros, zeros, zeros, rng,
        accel_noise_m_s2=0.02,
        gyro_noise_deg_s=0.05,
        mag_noise_ut=0.2,
        acceleration_ned_m_s2=acceleration,
    )
    gps_rng = np.random.default_rng(seed ^ 0x6A5A1D)
    gps_position = position.copy()
    gps_velocity = velocity.copy()
    gps_period = max(1, int(round(rate_hz / 10.0)))
    available = (time_s < outage_start_s) | (time_s >= outage_start_s + outage_duration_s)
    gps_updates = ((np.arange(sample_count) % gps_period == 0) & available).astype(np.int64)
    indices = np.flatnonzero(gps_updates)
    gps_position[indices] += gps_rng.normal(0.0, 0.5, (len(indices), 3))
    gps_velocity[indices] += gps_rng.normal(0.0, 0.1, (len(indices), 3))
    mag_period = max(1, int(round(rate_hz / 20.0)))
    synthetic.write_golden_csv(
        path,
        time_s,
        zeros,
        zeros,
        zeros,
        accel,
        gyro,
        magnetic,
        ideal_accel,
        np.zeros(sample_count, dtype=np.int64),
        rate_hz,
        (time_s < 1.5).astype(np.int64),
        position,
        velocity,
        gps_position,
        gps_velocity,
        gps_updates,
        np.ones(sample_count, dtype=np.int64),
        np.rint(time_s * 1.0e6).astype(np.int64),
        np.ones(sample_count, dtype=np.int64),
        (np.arange(sample_count) % mag_period == 0).astype(np.int64),
        np.zeros(sample_count, dtype=np.int64),
        np.zeros(sample_count, dtype=np.int64),
        np.zeros(sample_count, dtype=np.float64),
        np.ones(sample_count, dtype=np.float64),
        np.zeros(sample_count, dtype=np.int64),
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
    )
    return time_s, position, velocity, outage_start_s


def write_barometer_pair(
    base_path: Path,
    active_path: Path,
    shadow_path: Path,
    updates: np.ndarray,
    timestamp_us: np.ndarray,
    height_up_m: np.ndarray,
    variance_m2: np.ndarray,
) -> None:
    with base_path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        fieldnames = list(reader.fieldnames or []) + [
            "baro_update", "baro_timestamp_us", "baro_height_up_m", "baro_variance_m2"
        ]
        with active_path.open("w", encoding="utf-8", newline="") as active_stream, \
                shadow_path.open("w", encoding="utf-8", newline="") as shadow_stream:
            active_writer = csv.DictWriter(active_stream, fieldnames=fieldnames)
            shadow_writer = csv.DictWriter(shadow_stream, fieldnames=fieldnames)
            active_writer.writeheader()
            shadow_writer.writeheader()
            row_count = 0
            for index, row in enumerate(reader):
                row["baro_timestamp_us"] = str(int(timestamp_us[index]))
                row["baro_height_up_m"] = f"{height_up_m[index]:.9f}"
                row["baro_variance_m2"] = f"{variance_m2[index]:.9f}"
                row["baro_update"] = str(int(updates[index]))
                active_writer.writerow(row)
                row["baro_update"] = "0"
                shadow_writer.writerow(row)
                row_count += 1
    if row_count != len(updates):
        raise RuntimeError("barometer stream length does not match base replay")


def run_filter(
    runner: Path,
    input_path: Path,
    results_path: Path,
    log_path: Path,
    *,
    supervise_barometer: bool,
) -> list[str]:
    command = [str(runner), "--cold-start", "--compact-output"]
    if supervise_barometer:
        command.append("--supervise-barometer")
    command.extend([str(input_path), str(results_path)])
    completed = subprocess.run(
        command,
        cwd=ROOT, check=False, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        raise RuntimeError(f"runner returned {completed.returncode}: {completed.stdout[-1000:]}")
    return command


def audit_shadow_inputs(active_path: Path, shadow_path: Path) -> dict[str, object]:
    """Prove that the shadow input differs only by the barometer enable flag."""

    allowed_differences = {"baro_update"}
    with active_path.open("r", encoding="utf-8", newline="") as active_stream, \
            shadow_path.open("r", encoding="utf-8", newline="") as shadow_stream:
        active_reader = csv.DictReader(active_stream)
        shadow_reader = csv.DictReader(shadow_stream)
        if active_reader.fieldnames != shadow_reader.fieldnames:
            raise RuntimeError("active and shadow input headers differ")
        fieldnames = list(active_reader.fieldnames or [])
        differing_columns: set[str] = set()
        row_count = 0
        for row_count, (active_row, shadow_row) in enumerate(
            zip(active_reader, shadow_reader, strict=True), start=1
        ):
            for name in fieldnames:
                if active_row[name] != shadow_row[name]:
                    differing_columns.add(name)
                    if name not in allowed_differences:
                        raise RuntimeError(
                            f"active/shadow input differs at row {row_count}, column {name}"
                        )
    return {
        "verified": True,
        "data_rows": row_count,
        "allowed_differing_columns": sorted(allowed_differences),
        "observed_differing_columns": sorted(differing_columns),
        "all_other_fields_text_identical": True,
        "active_sha256": file_sha256(active_path),
        "shadow_sha256": file_sha256(shadow_path),
    }


def audit_shadow_commands(active: list[str], shadow: list[str]) -> dict[str, object]:
    """Check common runner/mode apart from declared supervisor and file paths."""

    def normalized(command: list[str]) -> list[str]:
        core = [argument for argument in command if argument != "--supervise-barometer"]
        if len(core) < 3:
            raise RuntimeError("unexpected validation runner command")
        return [*core[:-2], "<input>", "<results>"]

    active_normalized = normalized(active)
    shadow_normalized = normalized(shadow)
    if active_normalized != shadow_normalized:
        raise RuntimeError("active and shadow core runner commands are not equivalent")
    return {
        "verified": True,
        "active_command": active,
        "shadow_command": shadow,
        "normalized_common_command": active_normalized,
        "allowed_active_only_option": "--supervise-barometer",
    }


def score_vertical(
    results_path: Path,
    *,
    outage_start_s: float,
    outage_duration_s: float,
) -> dict[str, float]:
    columns: dict[str, list[float]] = {}
    with results_path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            for name in (
                "ts_us", "ref_position_d_m", "ref_velocity_d_m_s",
                "eskf_position_d_m", "eskf_velocity_d_m_s", "eskf_healthy",
                "input_baro_update", "eskf_baro_accepted", "eskf_baro_nis",
                "eskf_baro_test_ratio", "input_baro_status", "input_baro_age_s",
                "baro_supervisor_enabled", "baro_supervisor_accepted",
                "baro_supervisor_fault_flags", "baro_supervisor_latched",
            ):
                columns.setdefault(name, []).append(float(row[name]))
    values = {name: np.asarray(items, dtype=np.float64) for name, items in columns.items()}
    time_s = (values["ts_us"] - values["ts_us"][0]) * 1.0e-6
    outage = (time_s >= outage_start_s) & (time_s < outage_start_s + outage_duration_s)
    recovery = (time_s >= outage_start_s + outage_duration_s) & (
        time_s < outage_start_s + outage_duration_s + 5.0
    )
    position_error = values["eskf_position_d_m"] - values["ref_position_d_m"]
    velocity_error = values["eskf_velocity_d_m_s"] - values["ref_velocity_d_m_s"]
    outage_baro_updates = outage & (values["input_baro_update"] > 0.5)
    outage_baro_update_count = int(np.count_nonzero(outage_baro_updates))
    outage_baro_accepted_count = int(np.count_nonzero(
        outage_baro_updates & (values["eskf_baro_accepted"] > 0.5)
    ))
    finite_baro_nis = values["eskf_baro_nis"][outage_baro_updates]
    finite_baro_nis = finite_baro_nis[np.isfinite(finite_baro_nis)]
    outage_baro_stale_count = int(np.count_nonzero(
        outage_baro_updates & (values["input_baro_status"] == -5.0)
    ))
    outage_supervisor_rejected_count = int(np.count_nonzero(
        outage_baro_updates & (values["baro_supervisor_enabled"] > 0.5)
        & (values["baro_supervisor_accepted"] < 0.5)
    ))
    outage_supervisor_latched_count = int(np.count_nonzero(
        outage_baro_updates & (values["baro_supervisor_latched"] > 0.5)
    ))
    return {
        "outage_vertical_position_rmse_m": float(np.sqrt(np.mean(position_error[outage] ** 2))),
        "outage_vertical_position_peak_m": float(np.max(np.abs(position_error[outage]))),
        "outage_vertical_velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_error[outage] ** 2))),
        "outage_vertical_velocity_peak_m_s": float(np.max(np.abs(velocity_error[outage]))),
        "recovery_vertical_position_rmse_m": float(np.sqrt(np.mean(position_error[recovery] ** 2))),
        "healthy_ratio": float(np.mean(values["eskf_healthy"] > 0.5)),
        "outage_baro_update_count": outage_baro_update_count,
        "outage_baro_accepted_count": outage_baro_accepted_count,
        "outage_baro_stale_count": outage_baro_stale_count,
        "outage_supervisor_rejected_count": outage_supervisor_rejected_count,
        "outage_supervisor_latched_ratio": (
            float(outage_supervisor_latched_count / outage_baro_update_count)
            if outage_baro_update_count > 0 else 0.0
        ),
        "outage_baro_acceptance_ratio": (
            float(outage_baro_accepted_count / outage_baro_update_count)
            if outage_baro_update_count > 0 else 0.0
        ),
        "outage_baro_nis_p95": (
            float(np.percentile(finite_baro_nis, 95.0)) if len(finite_baro_nis) else math.nan
        ),
        "outage_baro_age_p95_s": (
            float(np.percentile(values["input_baro_age_s"][outage_baro_updates], 95.0))
            if outage_baro_update_count > 0 else math.nan
        ),
    }


def score_shadow_failover(
    supervised_results_path: Path,
    shadow_results_path: Path,
    *,
    outage_start_s: float,
    outage_duration_s: float,
    input_provenance_equivalent: bool = False,
    configuration_equivalent: bool = False,
) -> dict[str, object]:
    names = (
        "ts_us", "ref_position_d_m", "ref_velocity_d_m_s",
        "eskf_position_d_m", "eskf_velocity_d_m_s", "eskf_healthy",
        "eskf_static_aligned", "baro_supervisor_latched",
    )

    def load(path: Path) -> dict[str, np.ndarray]:
        columns = {name: [] for name in names}
        with path.open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                for name in names:
                    columns[name].append(float(row[name]))
        return {name: np.asarray(items, dtype=np.float64) for name, items in columns.items()}

    supervised = load(supervised_results_path)
    shadow = load(shadow_results_path)
    if not np.array_equal(supervised["ts_us"], shadow["ts_us"]):
        raise RuntimeError("supervised and shadow result timestamps differ")
    if not np.array_equal(supervised["ref_position_d_m"], shadow["ref_position_d_m"]):
        raise RuntimeError("supervised and shadow position references differ")
    if not np.array_equal(supervised["ref_velocity_d_m_s"], shadow["ref_velocity_d_m_s"]):
        raise RuntimeError("supervised and shadow velocity references differ")
    time_s = (supervised["ts_us"] - supervised["ts_us"][0]) * 1.0e-6
    latch_indices = np.flatnonzero(supervised["baro_supervisor_latched"] > 0.5)
    candidate_index = int(latch_indices[0]) if len(latch_indices) else None
    blocked_reasons: list[str] = []
    if candidate_index is None:
        blocked_reasons.append("no_supervisor_latch")
    else:
        if not input_provenance_equivalent:
            blocked_reasons.append("input_provenance_unverified")
        if not configuration_equivalent:
            blocked_reasons.append("configuration_equivalence_unverified")
        if shadow["eskf_healthy"][candidate_index] <= 0.5:
            blocked_reasons.append("shadow_unhealthy")
        elif not np.all(shadow["eskf_healthy"][candidate_index:] > 0.5):
            blocked_reasons.append("shadow_health_not_sustained")
        if shadow["eskf_static_aligned"][candidate_index] <= 0.5:
            blocked_reasons.append("shadow_alignment_incomplete")
        elif not np.all(shadow["eskf_static_aligned"][candidate_index:] > 0.5):
            blocked_reasons.append("shadow_alignment_not_sustained")
        if supervised["eskf_static_aligned"][candidate_index] <= 0.5:
            blocked_reasons.append("active_alignment_incomplete")
        if not all(np.all(np.isfinite(shadow[name][candidate_index:])) for name in (
            "eskf_position_d_m", "eskf_velocity_d_m_s",
        )):
            blocked_reasons.append("shadow_vertical_output_nonfinite")
    switch_index = candidate_index if not blocked_reasons else None
    use_shadow = np.zeros(len(time_s), dtype=bool)
    if switch_index is not None:
        use_shadow[switch_index:] = True
    position = np.where(
        use_shadow, shadow["eskf_position_d_m"], supervised["eskf_position_d_m"]
    )
    velocity = np.where(
        use_shadow, shadow["eskf_velocity_d_m_s"], supervised["eskf_velocity_d_m_s"]
    )
    healthy = np.where(use_shadow, shadow["eskf_healthy"], supervised["eskf_healthy"])
    outage = (time_s >= outage_start_s) & (time_s < outage_start_s + outage_duration_s)
    recovery = (time_s >= outage_start_s + outage_duration_s) & (
        time_s < outage_start_s + outage_duration_s + 5.0
    )
    position_error = position - supervised["ref_position_d_m"]
    velocity_error = velocity - supervised["ref_velocity_d_m_s"]
    result = {
        "outage_vertical_position_rmse_m": float(np.sqrt(np.mean(position_error[outage] ** 2))),
        "outage_vertical_position_peak_m": float(np.max(np.abs(position_error[outage]))),
        "outage_vertical_velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_error[outage] ** 2))),
        "outage_vertical_velocity_peak_m_s": float(np.max(np.abs(velocity_error[outage]))),
        "recovery_vertical_position_rmse_m": float(np.sqrt(np.mean(position_error[recovery] ** 2))),
        "healthy_ratio": float(np.mean(healthy > 0.5)),
        "shadow_failover_count": 1 if switch_index is not None else 0,
        "shadow_failover_time_s": (
            float(time_s[switch_index]) if switch_index is not None else math.nan
        ),
        "shadow_failover_position_reset_m": (
            float(shadow["eskf_position_d_m"][switch_index]
                - supervised["eskf_position_d_m"][switch_index])
            if switch_index is not None else 0.0
        ),
        "shadow_failover_velocity_reset_m_s": (
            float(shadow["eskf_velocity_d_m_s"][switch_index]
                - supervised["eskf_velocity_d_m_s"][switch_index])
            if switch_index is not None else 0.0
        ),
        "shadow_failover_candidate_count": 1 if candidate_index is not None else 0,
        "shadow_switch_preconditions_met": not blocked_reasons,
        "shadow_switch_blocked_reasons": blocked_reasons,
        "shadow_failover_upper_bound": True,
        "shadow_full_state_verified": False,
        "shadow_reset_semantics_verified": False,
        "shadow_state_components_switched": ["position_d", "velocity_d", "healthy"],
    }
    return result


def strict_json_value(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: strict_json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [strict_json_value(item) for item in value]
    if isinstance(value, tuple):
        return [strict_json_value(item) for item in value]
    return value


def aggregate(trials: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, float], list[dict[str, object]]] = {}
    for trial in trials:
        grouped.setdefault((str(trial["fault"]), float(trial["outage_duration_s"])), []).append(trial)
    rows: list[dict[str, object]] = []
    for (fault, duration), records in sorted(grouped.items()):
        row: dict[str, object] = {"fault": fault, "outage_duration_s": duration, "trials": len(records)}
        for metric in (
            "outage_vertical_position_rmse_m",
            "outage_vertical_position_peak_m",
            "outage_vertical_velocity_rmse_m_s",
            "outage_vertical_velocity_peak_m_s",
            "recovery_vertical_position_rmse_m",
            "healthy_ratio",
            "outage_baro_acceptance_ratio",
            "outage_baro_nis_p95",
            "outage_baro_age_p95_s",
            "outage_supervisor_latched_ratio",
        ):
            off = [float(record["imu_only"][metric]) for record in records]
            raw = [float(record["imu_baro_raw"][metric]) for record in records]
            supervised = [float(record["imu_baro_supervised"][metric]) for record in records]
            failover = [float(record["imu_baro_shadow_failover"][metric]) for record in records]
            row[metric] = {
                "imu_only_mean": mean(off),
                "imu_baro_raw_mean": mean(raw),
                "imu_baro_supervised_mean": mean(supervised),
                "imu_baro_shadow_failover_mean": mean(failover),
                "raw_paired_mean_delta": mean(
                    on_value - off_value for off_value, on_value in zip(off, raw)
                ),
                "supervised_paired_mean_delta": mean(
                    on_value - off_value for off_value, on_value in zip(off, supervised)
                ),
                "shadow_failover_paired_mean_delta": mean(
                    on_value - off_value for off_value, on_value in zip(off, failover)
                ),
            }
        failover_records = [record["imu_baro_shadow_failover"] for record in records]
        switched = [record for record in failover_records if int(record["shadow_failover_count"])]
        row["shadow_failover"] = {
            "switch_count": len(switched),
            "trial_count": len(failover_records),
            "position_reset_abs_p95_m": (
                float(np.percentile(
                    [abs(float(record["shadow_failover_position_reset_m"])) for record in switched],
                    95.0,
                )) if switched else None
            ),
            "position_reset_abs_max_m": (
                max(abs(float(record["shadow_failover_position_reset_m"])) for record in switched)
                if switched else None
            ),
            "velocity_reset_abs_p95_m_s": (
                float(np.percentile(
                    [abs(float(record["shadow_failover_velocity_reset_m_s"])) for record in switched],
                    95.0,
                )) if switched else None
            ),
            "velocity_reset_abs_max_m_s": (
                max(abs(float(record["shadow_failover_velocity_reset_m_s"])) for record in switched)
                if switched else None
            ),
            "detection_delay_p95_s": (
                float(np.percentile(
                    [float(record["fault_detection_delay_s"]) for record in switched
                     if math.isfinite(float(record["fault_detection_delay_s"]))],
                    95.0,
                )) if any(
                    math.isfinite(float(record["fault_detection_delay_s"])) for record in switched
                ) else None
            ),
        }
        rows.append(row)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/baro-outage-ab"))
    parser.add_argument("--outages", default="5,10,30,60,120")
    parser.add_argument("--faults", default=",".join(FAULTS))
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument(
        "--seed-start", type=int, default=0,
        help="first deterministic seed in this resumable campaign shard",
    )
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    if args.seeds <= 0 or args.seed_start < 0 or args.rate <= 0.0:
        parser.error("--seeds and --rate must be positive; --seed-start must be non-negative")
    outages = [float(value) for value in args.outages.split(",") if value]
    faults = [value for value in args.faults.split(",") if value]
    if any(value <= 0.0 for value in outages):
        parser.error("outage durations must be positive")
    if any(value not in FAULTS for value in faults):
        parser.error(f"faults must be selected from {', '.join(FAULTS)}")

    runner = args.runner.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    trials: list[dict[str, object]] = []
    started = time.monotonic()
    for outage_duration_s in outages:
        for fault in faults:
            for seed in range(args.seed_start, args.seed_start + args.seeds):
                trial_dir = out_dir / fault / f"outage-{outage_duration_s:g}s" / f"seed-{seed:04d}"
                trial_dir.mkdir(parents=True, exist_ok=True)
                base_path = trial_dir / "base.csv"
                time_s, position, _, outage_start_s = write_base_input(
                    base_path, outage_duration_s=outage_duration_s, rate_hz=args.rate, seed=seed
                )
                updates, baro_timestamp_us, height, variance, fault_model = barometer_stream(
                    time_s, -position[:, 2], fault=fault,
                    outage_start_s=outage_start_s, outage_duration_s=outage_duration_s,
                    rate_hz=args.rate, rng=np.random.default_rng(seed ^ 0xBA20),
                )
                arm_metrics: dict[str, object] = {}
                input_hashes: dict[str, str] = {}
                generated_paths: set[Path] = set()
                results_paths: dict[str, Path] = {}
                input_paths: dict[str, Path] = {}
                runner_commands: dict[str, list[str]] = {}
                active_path = trial_dir / "barometer-active.csv"
                shadow_path = trial_dir / "imu-only.csv"
                write_barometer_pair(
                    base_path, active_path, shadow_path, updates, baro_timestamp_us, height, variance
                )
                generated_paths.update((active_path, shadow_path))
                for arm, input_path, supervised in (
                    ("imu_only", shadow_path, False),
                    ("imu_baro_raw", active_path, False),
                    ("imu_baro_supervised", active_path, True),
                ):
                    results_path = trial_dir / f"{arm}-results.csv"
                    generated_paths.add(results_path)
                    results_paths[arm] = results_path
                    input_paths[arm] = input_path
                    input_hashes[arm] = file_sha256(input_path)
                    runner_commands[arm] = run_filter(
                        runner, input_path, results_path, trial_dir / f"{arm}.log",
                        supervise_barometer=supervised,
                    )
                    arm_metrics[arm] = score_vertical(
                        results_path,
                        outage_start_s=outage_start_s,
                        outage_duration_s=outage_duration_s,
                    )
                failover_metrics = dict(arm_metrics["imu_baro_supervised"])
                shadow_input_audit = audit_shadow_inputs(
                    input_paths["imu_baro_supervised"], input_paths["imu_only"]
                )
                shadow_command_audit = audit_shadow_commands(
                    runner_commands["imu_baro_supervised"], runner_commands["imu_only"]
                )
                failover_metrics.update(score_shadow_failover(
                    results_paths["imu_baro_supervised"],
                    results_paths["imu_only"],
                    outage_start_s=outage_start_s,
                    outage_duration_s=outage_duration_s,
                    input_provenance_equivalent=bool(shadow_input_audit["verified"]),
                    configuration_equivalent=bool(shadow_command_audit["verified"]),
                ))
                failover_metrics["shadow_input_audit"] = shadow_input_audit
                failover_metrics["shadow_command_audit"] = shadow_command_audit
                fault_onset_s = fault_model["fault_onset_s"]
                failover_metrics["fault_detection_delay_s"] = (
                    float(failover_metrics["shadow_failover_time_s"]) - float(fault_onset_s)
                    if failover_metrics["shadow_failover_count"] and fault_onset_s is not None
                    else math.nan
                )
                arm_metrics["imu_baro_shadow_failover"] = failover_metrics
                record: dict[str, object] = {
                    "fault": fault,
                    "fault_model": fault_model,
                    "outage_duration_s": outage_duration_s,
                    "seed": seed,
                    "base_sha256": file_sha256(base_path),
                    "input_sha256": input_hashes,
                    **arm_metrics,
                }
                trials.append(record)
                if args.compact:
                    for generated_path in sorted(generated_paths):
                        generated_path.unlink()
                    base_path.unlink()

    output = {
        "schema_version": 5,
        "status": "completed",
        "scope": "four-arm synthetic barometer aiding, source-supervision, and shadow-failover study; no promotion gates",
        "outages_s": outages,
        "faults": faults,
        "seeds": args.seeds,
        "seed_start": args.seed_start,
        "rate_hz": args.rate,
        "trial_count": len(trials),
        "runtime_s": time.monotonic() - started,
        "provenance": {
            "git_commit": capture(["git", "rev-parse", "HEAD"]),
            "git_status_short": (capture(["git", "status", "--short"]) or "").splitlines(),
            "runner_sha256": file_sha256(runner),
            "script_sha256": file_sha256(Path(__file__).resolve()),
            "generator_sha256": file_sha256(ROOT / "simulation/tools/generate_synthetic_imu.py"),
            "barometer_supervisor_sha256": file_sha256(ROOT / "src/barometer_supervisor.c"),
        },
        "aggregate": aggregate(trials),
        "trials": trials,
        "limitations": [
            "The barometer is already converted to relative height; pressure and datum management are not tested.",
            "The current 15-state ESKF has no barometer-bias state.",
            "The source supervisor and thresholds are experimental PC-validation candidates, not FCOne release gates.",
            "The shadow-failover arm is an offline timestamp-aligned output mux, not a real-time FCOne lane implementation.",
            "The four-arm study does not exercise source-generation reset or authorized jump recovery end to end.",
            "Constant datum bias cannot be distinguished from true relative height without an independent vertical reference.",
            "Synthetic vertical motion and fault models are not physical FCOne qualification.",
        ],
    }
    (out_dir / "summary.json").write_text(
        json.dumps(strict_json_value(output), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Four-arm barometer outage study", "",
        f"Completed **{len(trials)}** fault/outage trials across four matched arms in "
        f"**{output['runtime_s']:.1f} s**.", "",
        "| Fault | Outage | IMU-only RMSE | Raw baro RMSE | Supervised RMSE | Shadow failover RMSE | Failover delta | Raw accepted | Supervised accepted | Supervisor latched |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in output["aggregate"]:
        metric = row["outage_vertical_position_rmse_m"]
        lines.append(
            f"| {row['fault']} | {row['outage_duration_s']:g} s | "
            f"{metric['imu_only_mean']:.4f} m | {metric['imu_baro_raw_mean']:.4f} m | "
            f"{metric['imu_baro_supervised_mean']:.4f} m | "
            f"{metric['imu_baro_shadow_failover_mean']:.4f} m | "
            f"{metric['shadow_failover_paired_mean_delta']:+.4f} m | "
            f"{row['outage_baro_acceptance_ratio']['imu_baro_raw_mean']:.1%} | "
            f"{row['outage_baro_acceptance_ratio']['imu_baro_supervised_mean']:.1%} | "
            f"{row['outage_supervisor_latched_ratio']['imu_baro_supervised_mean']:.1%} |"
        )
    lines.extend([
        "", "Negative deltas favor barometer aiding. This is a diagnostic study without promotion gates;",
        "pressure conversion, datum management, source supervision, and physical qualification remain open.",
    ])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Barometer A/B report: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
