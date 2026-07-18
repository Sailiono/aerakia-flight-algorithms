#!/usr/bin/env python3
"""Inventory a PX4 ULog corpus without treating PX4 estimates as truth."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Callable

import numpy as np


TOPICS = (
    "sensor_combined",
    "vehicle_attitude",
    "vehicle_gps_position",
    "vehicle_imu_status",
    "vehicle_local_position",
    "vehicle_magnetometer",
    "yaw_estimator_status",
)


def _load_ulog(path: Path) -> Any:
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError("pyulog is required; install the repository requirements") from error
    return ULog(str(path), message_name_filter_list=list(TOPICS))


def _topic(ulog: Any, name: str, multi_id: int = 0) -> dict[str, np.ndarray] | None:
    for dataset in getattr(ulog, "data_list", []):
        if dataset.name == name and int(getattr(dataset, "multi_id", 0)) == multi_id:
            return dataset.data
    return None


def _field(data: dict[str, np.ndarray] | None, *names: str) -> np.ndarray:
    if data is not None:
        for name in names:
            if name in data:
                return np.asarray(data[name])
    return np.asarray([], dtype=np.float64)


def _vector(data: dict[str, np.ndarray] | None, *bases: str) -> np.ndarray:
    if data is not None:
        for base in bases:
            names = [f"{base}[{axis}]" for axis in range(3)]
            if all(name in data for name in names):
                return np.column_stack([data[name] for name in names]).astype(np.float64)
    return np.empty((0, 3), dtype=np.float64)


def _finite_percentile(values: np.ndarray, percentile: float) -> float | None:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    return None if finite.size == 0 else float(np.percentile(finite, percentile))


def _maximum_gap_s(timestamps_us: np.ndarray) -> float | None:
    timestamps = np.asarray(timestamps_us, dtype=np.float64)
    timestamps = np.unique(timestamps[np.isfinite(timestamps) & (timestamps > 0.0)])
    return None if timestamps.size < 2 else float(np.max(np.diff(timestamps)) * 1.0e-6)


def _timestamps(data: dict[str, np.ndarray] | None) -> np.ndarray:
    if data is not None:
        for name in ("timestamp_sample", "timestamp"):
            candidate = _field(data, name).astype(np.float64)
            finite = candidate[np.isfinite(candidate) & (candidate >= 0.0)]
            if finite.size == 1 or (
                finite.size >= 2 and float(np.max(finite)) > float(np.min(finite))
            ):
                return candidate
    return np.asarray([], dtype=np.float64)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit_ulog(path: Path, ulog_factory: Callable[[Path], Any] = _load_ulog) -> dict[str, Any]:
    path = Path(path)
    ulog = ulog_factory(path)
    imu = _topic(ulog, "sensor_combined")
    gps = _topic(ulog, "vehicle_gps_position")
    attitude = _topic(ulog, "vehicle_attitude")
    mag = _topic(ulog, "vehicle_magnetometer")
    local_position = _topic(ulog, "vehicle_local_position")
    imu_status = _topic(ulog, "vehicle_imu_status")

    imu_t = _timestamps(imu)
    gyro = _vector(imu, "gyro_rad", "gyroscope_rad_s")
    accel = _vector(imu, "accelerometer_m_s2")
    gps_t = _timestamps(gps)
    gps_heading = _field(gps, "heading")
    gps_velocity = _vector(gps, "vel_ned_m_s")
    if gps_velocity.size == 0 and gps is not None:
        names = ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s")
        if all(name in gps for name in names):
            gps_velocity = np.column_stack([gps[name] for name in names]).astype(np.float64)
    gps_speed = np.linalg.norm(gps_velocity, axis=1) if gps_velocity.size else np.asarray([])
    gyro_norm = np.linalg.norm(gyro, axis=1) if gyro.size else np.asarray([])
    accel_norm = np.linalg.norm(accel, axis=1) if accel.size else np.asarray([])

    reset_counter = _field(attitude, "quat_reset_counter")
    reset_events = 0
    if reset_counter.size > 1:
        reset_events = int(np.count_nonzero(np.diff(reset_counter.astype(np.int64))))

    clipping_max = 0
    if imu_status is not None:
        clipping_fields = [
            name for name in imu_status
            if name.startswith("accel_clipping[") or name.startswith("gyro_clipping[")
        ]
        if clipping_fields:
            clipping = np.column_stack([imu_status[name] for name in clipping_fields])
            clipping_max = int(np.nanmax(np.sum(clipping, axis=1)))

    duration_s = float((ulog.last_timestamp - ulog.start_timestamp) * 1.0e-6)
    heading_valid = int(np.count_nonzero(np.isfinite(gps_heading)))
    return {
        "file": path.name,
        "sha256": _sha256(path),
        "duration_s": duration_s,
        "imu_samples": int(imu_t.size),
        "imu_rate_hz": float(imu_t.size / duration_s) if duration_s > 0.0 else None,
        "gyro_norm_p95_rad_s": _finite_percentile(gyro_norm, 95.0),
        "gyro_norm_max_rad_s": _finite_percentile(gyro_norm, 100.0),
        "accel_norm_p95_m_s2": _finite_percentile(accel_norm, 95.0),
        "accel_norm_max_m_s2": _finite_percentile(accel_norm, 100.0),
        "gps_samples": int(gps_t.size),
        "gps_rate_hz": float(gps_t.size / duration_s) if duration_s > 0.0 else None,
        "gps_maximum_gap_s": _maximum_gap_s(gps_t),
        "gps_speed_p95_m_s": _finite_percentile(gps_speed, 95.0),
        "gps_speed_max_m_s": _finite_percentile(gps_speed, 100.0),
        "direct_gnss_heading_samples": heading_valid,
        "attitude_samples": int(_field(attitude, "timestamp").size),
        "attitude_reset_events": reset_events,
        "magnetometer_samples": int(_field(mag, "timestamp").size),
        "local_position_samples": int(_field(local_position, "timestamp").size),
        "maximum_reported_clipping_count": clipping_max,
        "reference_kind": "px4_estimate",
    }


def summarize(files: list[dict[str, Any]], errors: list[dict[str, str]]) -> dict[str, Any]:
    def total(name: str) -> float:
        return float(sum(float(item.get(name) or 0.0) for item in files))

    maximum_gap = max((item.get("gps_maximum_gap_s") or 0.0 for item in files), default=0.0)
    return {
        "schema_version": 1,
        "evidence_boundary": (
            "Physical PX4 sensor/log compatibility and coverage only; onboard PX4 estimates are "
            "not independent navigation truth."
        ),
        "corpus": {
            "files": len(files),
            "parse_errors": len(errors),
            "total_duration_s": total("duration_s"),
            "total_imu_samples": int(total("imu_samples")),
            "total_gps_samples": int(total("gps_samples")),
            "files_with_direct_gnss_heading": sum(
                int(item["direct_gnss_heading_samples"] > 0) for item in files
            ),
            "files_with_attitude_resets": sum(
                int(item["attitude_reset_events"] > 0) for item in files
            ),
            "files_with_reported_clipping": sum(
                int(item["maximum_reported_clipping_count"] > 0) for item in files
            ),
            "maximum_gps_gap_s": float(maximum_gap),
            "maximum_gyro_norm_rad_s": max(
                (item.get("gyro_norm_max_rad_s") or 0.0 for item in files), default=0.0
            ),
            "maximum_accel_norm_m_s2": max(
                (item.get("accel_norm_max_m_s2") or 0.0 for item in files), default=0.0
            ),
            "maximum_gps_speed_m_s": max(
                (item.get("gps_speed_max_m_s") or 0.0 for item in files), default=0.0
            ),
        },
        "files": files,
        "errors": errors,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--allow-errors", action="store_true")
    args = parser.parse_args()
    paths = sorted(args.root.rglob("*.ulg"))
    if not paths:
        parser.error(f"no ULog files found under {args.root}")
    files: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    for index, path in enumerate(paths, start=1):
        print(f"[{index}/{len(paths)}] {path.name}", flush=True)
        try:
            files.append(audit_ulog(path))
        except Exception as error:  # fail closed after retaining corpus diagnostics
            errors.append({"file": path.name, "error": f"{type(error).__name__}: {error}"})
    result = summarize(files, errors)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result["corpus"], indent=2, sort_keys=True))
    if errors and not args.allow_errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
