#!/usr/bin/env python3
"""Compute repeatable accuracy, integrity, reset, and navigation replay metrics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ALGORITHMS = ("mahony_standard", "mahony_robust", "eskf")
AXES = ("roll", "pitch", "yaw")
CHI_SQUARE_95 = {
    3: (0.215795, 9.348404),
    6: (1.237344, 14.449375),
}


def wrapped_error_deg(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    return (estimate - truth + 180.0) % 360.0 - 180.0


def circular_mean_deg(angle: np.ndarray) -> float:
    radians = np.radians(angle)
    return float(np.degrees(np.arctan2(np.mean(np.sin(radians)), np.mean(np.cos(radians)))))


def load_columns(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            for name, value in row.items():
                if name is not None and value not in (None, ""):
                    columns.setdefault(name, []).append(float(value))
    if not columns:
        raise ValueError(f"empty results file: {path}")
    lengths = {len(values) for values in columns.values()}
    if len(lengths) != 1:
        raise ValueError(f"inconsistent CSV columns: {path}")
    return {name: np.asarray(values, dtype=np.float64) for name, values in columns.items()}


def _initial_alignment_offset(
    time_s: np.ndarray, estimate: np.ndarray, truth: np.ndarray, maximum_s: float = 2.0
) -> float:
    mask = time_s <= min(maximum_s, max(0.5, float(time_s[-1]) * 0.05))
    if not np.any(mask):
        mask = np.arange(len(time_s)) < min(10, len(time_s))
    return circular_mean_deg(wrapped_error_deg(truth[mask], estimate[mask]))


def _error_summary(error: np.ndarray) -> dict[str, float]:
    absolute = np.abs(error)
    return {
        "rmse_deg": float(np.sqrt(np.mean(error * error))),
        "p95_abs_deg": float(np.percentile(absolute, 95)),
        "max_abs_deg": float(np.max(absolute)),
        "final_error_deg": float(error[-1]),
    }


def _normalized_quaternion_columns(
    columns: dict[str, np.ndarray], prefix: str
) -> np.ndarray | None:
    names = [f"{prefix}_q_{axis}" for axis in ("w", "x", "y", "z")]
    if not all(name in columns for name in names):
        return None
    quaternion = np.column_stack([columns[name] for name in names])
    norms = np.linalg.norm(quaternion, axis=1)
    if np.any(~np.isfinite(norms) | (norms <= 1.0e-12)):
        raise ValueError(f"invalid quaternion output for {prefix}")
    return quaternion / norms[:, None]


def _apply_navigation_yaw_offset(quaternion: np.ndarray, yaw_offset_deg: float) -> np.ndarray:
    if yaw_offset_deg == 0.0:
        return quaternion
    half = np.radians(yaw_offset_deg) * 0.5
    yaw = np.asarray([np.cos(half), 0.0, 0.0, np.sin(half)])
    w1, x1, y1, z1 = yaw
    w2, x2, y2, z2 = quaternion.T
    return np.column_stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    )


def quaternion_attitude_errors_deg(
    estimate: np.ndarray, truth: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    estimate = estimate / np.linalg.norm(estimate, axis=1)[:, None]
    truth = truth / np.linalg.norm(truth, axis=1)[:, None]
    dot = np.sum(estimate * truth, axis=1)
    geodesic = np.degrees(2.0 * np.arccos(np.clip(np.abs(dot), 0.0, 1.0)))

    def down_body(quaternion: np.ndarray) -> np.ndarray:
        w, x, y, z = quaternion.T
        return np.column_stack(
            (
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            )
        )

    down_dot = np.sum(down_body(estimate) * down_body(truth), axis=1)
    tilt = np.degrees(np.arccos(np.clip(down_dot, -1.0, 1.0)))
    return geodesic, tilt


def metrics_for(
    columns: dict[str, np.ndarray], algorithm: str, reference_kind: str
) -> dict[str, object]:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    axes: dict[str, dict[str, float]] = {}
    squared_errors: list[np.ndarray] = []
    roll_pitch_squared: list[np.ndarray] = []
    yaw_offset = 0.0
    if reference_kind == "px4_estimate":
        yaw_offset = _initial_alignment_offset(
            time_s, columns[f"{algorithm}_yaw_deg"], columns["truth_yaw_deg"]
        )
    for axis in AXES:
        truth = columns[f"truth_{axis}_deg"]
        estimate = columns[f"{algorithm}_{axis}_deg"].copy()
        if axis == "yaw":
            estimate += yaw_offset
        error = wrapped_error_deg(estimate, truth)
        squared_errors.append(error * error)
        if axis != "yaw":
            roll_pitch_squared.append(error * error)
        axes[axis] = _error_summary(error)

    euler_component_rmse = float(np.sqrt(np.mean(np.column_stack(squared_errors))))
    euler_roll_pitch_rmse = float(np.sqrt(np.mean(np.column_stack(roll_pitch_squared))))
    result: dict[str, object] = {
        "overall_attitude_rmse_deg": euler_component_rmse,
        "tilt_rmse_deg": euler_roll_pitch_rmse,
        "euler_component_rmse_deg": euler_component_rmse,
        "euler_roll_pitch_component_rmse_deg": euler_roll_pitch_rmse,
        "initial_yaw_alignment_offset_deg": yaw_offset,
        "axes": axes,
    }
    truth_q = _normalized_quaternion_columns(columns, "truth")
    estimate_q = _normalized_quaternion_columns(columns, algorithm)
    if truth_q is not None and estimate_q is not None:
        estimate_q = _apply_navigation_yaw_offset(estimate_q, yaw_offset)
        geodesic_error, tilt_error = quaternion_attitude_errors_deg(estimate_q, truth_q)
        result["overall_attitude_rmse_deg"] = float(
            np.sqrt(np.mean(geodesic_error * geodesic_error))
        )
        result["tilt_rmse_deg"] = float(np.sqrt(np.mean(tilt_error * tilt_error)))
        result["quaternion_geodesic_error_deg"] = _error_summary(geodesic_error)
        result["gravity_direction_error_deg"] = _error_summary(tilt_error)
    if algorithm == "mahony_robust":
        result["minimum_accelerometer_weight"] = float(np.min(columns["mahony_robust_acc_weight"]))
        result["mean_magnetometer_weight"] = float(np.mean(columns["mahony_robust_mag_weight"]))
    return result


def fallback_envelope_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    """Bound robust-Mahony truth error after a continuity-qualified fallback entry.

    This is an offline truth diagnostic, not a signal available to the flight supervisor. It helps
    select conservative entry gates and degraded-mode time budgets from recorded motion.
    """
    truth = _normalized_quaternion_columns(columns, "truth")
    fallback = _normalized_quaternion_columns(columns, "mahony_robust")
    if truth is None or fallback is None:
        return None
    error_deg, _ = quaternion_attitude_errors_deg(fallback, truth)
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    result: dict[str, object] = {
        "interpretation": (
            "Offline independent-truth envelope; the flight supervisor cannot observe truth error."
        ),
        "first_exceedance_s": {},
        "entry_envelopes": {},
    }
    for gate_deg in (5.0, 10.0, 15.0):
        crossing = np.flatnonzero(error_deg > gate_deg)
        result["first_exceedance_s"][f"{gate_deg:g}_deg"] = (
            float(time_s[crossing[0]]) if len(crossing) else None
        )
        eligible = np.flatnonzero(error_deg <= gate_deg)
        gate_result: dict[str, object] = {"eligible_start_samples": int(len(eligible))}
        for horizon_s in (0.5, 1.0, 2.0, 5.0):
            if len(eligible) == 0:
                gate_result[f"{horizon_s:g}_s"] = {
                    "p95_peak_error_deg": None,
                    "maximum_peak_error_deg": None,
                }
                continue
            peaks = np.asarray(
                [
                    np.max(error_deg[index : np.searchsorted(
                        time_s, time_s[index] + horizon_s, side="right"
                    )])
                    for index in eligible
                ],
                dtype=np.float64,
            )
            gate_result[f"{horizon_s:g}_s"] = {
                "p95_peak_error_deg": float(np.percentile(peaks, 95.0)),
                "maximum_peak_error_deg": float(np.max(peaks)),
            }
        result["entry_envelopes"][f"{gate_deg:g}_deg"] = gate_result
    return result


def cold_start_alignment_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    if "eskf_static_tilt_aligned" not in columns:
        return None
    tilt_aligned = columns["eskf_static_tilt_aligned"] > 0.5
    heading_aligned = columns.get(
        "eskf_static_heading_aligned", np.zeros(len(tilt_aligned))
    ) > 0.5
    if not np.any(tilt_aligned):
        return None
    first_tilt = int(np.flatnonzero(tilt_aligned)[0])
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    truth_q = _normalized_quaternion_columns(columns, "truth")
    estimate_q = _normalized_quaternion_columns(columns, "eskf")
    if truth_q is not None and estimate_q is not None:
        _, tilt_error = quaternion_attitude_errors_deg(
            estimate_q[first_tilt:], truth_q[first_tilt:]
        )
        post_tilt_rmse = float(np.sqrt(np.mean(tilt_error * tilt_error)))
    else:
        roll_error = wrapped_error_deg(
            columns["eskf_roll_deg"][first_tilt:], columns["truth_roll_deg"][first_tilt:]
        )
        pitch_error = wrapped_error_deg(
            columns["eskf_pitch_deg"][first_tilt:], columns["truth_pitch_deg"][first_tilt:]
        )
        post_tilt_rmse = float(
            np.sqrt(np.mean(np.column_stack((roll_error * roll_error, pitch_error * pitch_error))))
        )
    result: dict[str, object] = {
        "tilt_alignment_time_s": float(time_s[first_tilt]),
        "post_tilt_alignment_samples": int(len(time_s) - first_tilt),
        "post_tilt_alignment_tilt_rmse_deg": post_tilt_rmse,
        "heading_alignment_completed": bool(np.any(heading_aligned)),
    }
    complete = tilt_aligned & heading_aligned
    if not np.any(complete):
        return result
    first = int(np.flatnonzero(complete)[0])
    squared_errors: list[np.ndarray] = []
    axes: dict[str, dict[str, float]] = {}
    for axis in AXES:
        error = wrapped_error_deg(
            columns[f"eskf_{axis}_deg"][first:], columns[f"truth_{axis}_deg"][first:]
        )
        squared_errors.append(error * error)
        axes[axis] = _error_summary(error)
    post_alignment_rmse = float(np.sqrt(np.mean(np.column_stack(squared_errors))))
    if truth_q is not None and estimate_q is not None:
        geodesic_error, _ = quaternion_attitude_errors_deg(
            estimate_q[first:], truth_q[first:]
        )
        post_alignment_rmse = float(np.sqrt(np.mean(geodesic_error * geodesic_error)))
    result.update({
        "alignment_time_s": float(time_s[first]),
        "post_alignment_samples": int(len(time_s) - first),
        "post_alignment_attitude_rmse_deg": post_alignment_rmse,
        "axes": axes,
    })
    return result


def segment_aligned_yaw_metrics(columns: dict[str, np.ndarray]) -> dict[str, object]:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    counters = columns.get("ref_attitude_reset_counter", np.zeros(len(time_s)))
    estimate = columns["eskf_yaw_deg"]
    truth = columns["truth_yaw_deg"]
    errors: list[np.ndarray] = []
    offsets: list[dict[str, float]] = []
    start = 0
    for index in range(1, len(counters) + 1):
        if index < len(counters) and counters[index] == counters[start]:
            continue
        segment_time = time_s[start:index] - time_s[start]
        segment_duration = float(segment_time[-1]) if len(segment_time) else 0.0
        alignment_s = min(5.0, max(0.5, 0.20 * segment_duration))
        alignment = segment_time <= alignment_s
        offset = circular_mean_deg(
            wrapped_error_deg(truth[start:index][alignment], estimate[start:index][alignment])
        )
        errors.append(wrapped_error_deg(estimate[start:index] + offset, truth[start:index]))
        offsets.append({"counter": float(counters[start]), "offset_deg": offset})
        start = index
    combined = np.concatenate(errors)
    summary: dict[str, object] = _error_summary(combined)
    summary["segments"] = len(offsets)
    summary["segment_offsets"] = offsets
    return summary


def reset_yaw_delta_deg(columns: dict[str, np.ndarray]) -> np.ndarray:
    result = np.zeros(len(columns["ts_us"]), dtype=np.float64)
    if "ref_delta_q_reset_w" not in columns:
        return result
    q = np.column_stack(
        [columns[f"ref_delta_q_reset_{axis}"] for axis in ("w", "x", "y", "z")]
    )
    q /= np.maximum(np.linalg.norm(q, axis=1)[:, None], 1.0e-12)
    return np.degrees(
        np.arctan2(
            2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
            1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
        )
    )


def reset_compensated_reference_yaw(columns: dict[str, np.ndarray]) -> np.ndarray:
    events = columns.get("ref_attitude_reset_event", np.zeros(len(columns["ts_us"]))) > 0.5
    cumulative_reset = np.cumsum(np.where(events, reset_yaw_delta_deg(columns), 0.0))
    return wrapped_error_deg(columns["truth_yaw_deg"], cumulative_reset)


def reset_compensated_yaw_metrics(columns: dict[str, np.ndarray]) -> dict[str, float]:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    reference = reset_compensated_reference_yaw(columns)
    estimate = columns["eskf_yaw_deg"].copy()
    offset = _initial_alignment_offset(time_s, estimate, reference)
    result = _error_summary(wrapped_error_deg(estimate + offset, reference))
    result["initial_yaw_alignment_offset_deg"] = offset
    return result


def reset_summary(columns: dict[str, np.ndarray]) -> dict[str, object]:
    events = columns.get("ref_attitude_reset_event", np.zeros(len(columns["ts_us"]))) > 0.5
    result: dict[str, object] = {
        "events": float(np.count_nonzero(events)),
        "maximum_rotation_deg": 0.0,
        "maximum_yaw_delta_deg": 0.0,
        "event_details": [],
    }
    if not np.any(events) or "ref_delta_q_reset_w" not in columns:
        return result
    q = np.column_stack(
        [columns[f"ref_delta_q_reset_{axis}"] for axis in ("w", "x", "y", "z")]
    )[events]
    q /= np.maximum(np.linalg.norm(q, axis=1)[:, None], 1.0e-12)
    result["maximum_rotation_deg"] = float(
        np.max(np.degrees(2.0 * np.arccos(np.clip(np.abs(q[:, 0]), 0.0, 1.0))))
    )
    yaw = reset_yaw_delta_deg(columns)[events]
    result["maximum_yaw_delta_deg"] = float(np.max(np.abs(yaw)))
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    details: list[dict[str, float]] = []
    for event_index in np.flatnonzero(events):
        before = max(0, int(event_index) - 1)
        after = int(event_index)
        raw_jump = wrapped_error_deg(
            np.asarray([columns["truth_yaw_deg"][after]]),
            np.asarray([columns["truth_yaw_deg"][before]]),
        )[0]
        error_before = wrapped_error_deg(
            np.asarray([columns["eskf_yaw_deg"][before]]),
            np.asarray([columns["truth_yaw_deg"][before]]),
        )[0]
        error_after = wrapped_error_deg(
            np.asarray([columns["eskf_yaw_deg"][after]]),
            np.asarray([columns["truth_yaw_deg"][after]]),
        )[0]
        details.append(
            {
                "time_s": float(time_s[after]),
                "delta_quaternion_yaw_deg": float(reset_yaw_delta_deg(columns)[after]),
                "observed_px4_yaw_jump_deg": float(raw_jump),
                "eskf_minus_px4_before_deg": float(error_before),
                "eskf_minus_px4_after_deg": float(error_after),
                "agreement_error_step_deg": float(wrapped_error_deg(
                    np.asarray([error_after]), np.asarray([error_before])
                )[0]),
            }
        )
    result["event_details"] = details
    return result


def yaw_source_diagnostics(columns: dict[str, np.ndarray]) -> dict[str, object]:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    result: dict[str, object] = {
        "direct_gnss_heading_updates": int(np.count_nonzero(
            columns.get("input_heading_update", np.zeros(len(time_s))) > 0.5
        )),
        "gnss_course_diagnostic_updates": int(np.count_nonzero(
            columns.get("gnss_course_update", np.zeros(len(time_s))) > 0.5
        )),
        "px4_gsf_updates": int(np.count_nonzero(
            columns.get("px4_gsf_yaw_update", np.zeros(len(time_s))) > 0.5
        )),
    }
    direct_valid = (
        (columns.get("gnss_heading_valid", np.zeros(len(time_s))) > 0.5)
        & (columns.get("input_heading_update", np.zeros(len(time_s))) > 0.5)
    )
    gsf_valid = (
        (columns.get("px4_gsf_yaw_valid", np.zeros(len(time_s))) > 0.5)
        & (columns.get("px4_gsf_yaw_update", np.zeros(len(time_s))) > 0.5)
        & (columns.get("px4_gsf_yaw_variance_rad2", np.ones(len(time_s)))
           <= np.radians(15.0) ** 2)
        & (columns.get("gnss_ground_speed_m_s", np.zeros(len(time_s))) >= 1.5)
    )
    result["px4_gsf_confident_updates"] = int(np.count_nonzero(gsf_valid))
    sources = (
        ("direct_gnss_heading", direct_valid, "gnss_heading_rad"),
        ("px4_gsf_yaw", gsf_valid, "px4_gsf_yaw_rad"),
    )
    for label, valid, value_name in sources:
        if value_name not in columns:
            continue
        if not np.any(valid):
            continue
        reference = np.degrees(columns[value_name])
        estimate = columns["eskf_yaw_deg"].copy()
        offset = _initial_alignment_offset(time_s[valid], estimate[valid], reference[valid])
        summary = _error_summary(wrapped_error_deg(estimate[valid] + offset, reference[valid]))
        summary["samples"] = int(np.count_nonzero(valid))
        summary["initial_yaw_alignment_offset_deg"] = offset
        result[label] = summary
    return result


def trusted_heading_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    attempted = columns.get("input_heading_update", np.zeros(len(columns["ts_us"]))) > 0.5
    if not np.any(attempted):
        return None
    accepted = columns.get("eskf_heading_accepted", np.zeros(len(attempted))) > 0.5
    fault = columns.get("input_heading_fault", np.zeros(len(attempted))) > 0.5
    normal = attempted & ~fault
    fault_attempted = attempted & fault
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    yaw_error = wrapped_error_deg(columns["eskf_yaw_deg"], columns["truth_yaw_deg"])
    # A scalar body-forward heading is ill-defined when that axis is nearly vertical. Keep
    # geometry-driven invalidity separate from an actual source outage, and never turn Euler
    # wrap behavior near the singularity into a claimed heading error.
    horizontal_projection = np.abs(np.cos(np.radians(columns["truth_pitch_deg"])))
    heading_observable = horizontal_projection >= 0.25
    normal_accepts = np.flatnonzero(normal & accepted)

    valid = columns.get("gnss_heading_valid", np.zeros(len(attempted))) > 0.5
    valid_indices = np.flatnonzero(valid)
    longest_start = longest_stop = None
    dropout = ~valid & heading_observable
    if len(valid_indices):
        dropout[:int(valid_indices[0])] = False
        dropout[int(valid_indices[-1]) + 1:] = False
    cursor = 0
    while cursor < len(dropout):
        if not dropout[cursor]:
            cursor += 1
            continue
        start = cursor
        while cursor < len(dropout) and dropout[cursor]:
            cursor += 1
        if longest_start is None or cursor - start > longest_stop - longest_start:
            longest_start, longest_stop = start, cursor
    post_first_yaw = np.asarray([], dtype=np.float64)
    if len(normal_accepts):
        first_accept = int(normal_accepts[0])
        post_first_yaw = yaw_error[first_accept:][heading_observable[first_accept:]]

    result: dict[str, object] = {
        "attempted_updates": int(np.count_nonzero(attempted)),
        "normal_attempts": int(np.count_nonzero(normal)),
        "fault_attempts": int(np.count_nonzero(fault_attempted)),
        "normal_acceptance_ratio": float(np.mean(accepted[normal])) if np.any(normal) else None,
        "fault_rejection_ratio": (
            float(np.mean(~accepted[fault_attempted])) if np.any(fault_attempted) else None
        ),
        "minimum_horizontal_projection": 0.25,
        "geometry_unobservable_samples": int(np.count_nonzero(~heading_observable)),
        "overall_post_first_accept_yaw_rmse_deg": (
            float(np.sqrt(np.mean(post_first_yaw * post_first_yaw)))
            if len(post_first_yaw) else None
        ),
    }
    if np.any(fault_attempted):
        innovation = np.degrees(np.abs(columns["eskf_heading_innovation_rad"][fault_attempted]))
        result["fault_innovation_min_abs_deg"] = float(np.min(innovation))
    if longest_start is not None and longest_stop is not None:
        recovery_candidates = np.flatnonzero(
            (np.arange(len(attempted)) >= longest_stop) & normal & accepted
        )
        result["dropout_duration_s"] = float(
            time_s[longest_stop - 1] - time_s[longest_start]
        )
        result["dropout_max_abs_yaw_error_deg"] = float(
            np.max(np.abs(yaw_error[longest_start:longest_stop]))
        )
        if len(recovery_candidates):
            recovery = int(recovery_candidates[0])
            result["recovery_time_s"] = float(time_s[recovery] - time_s[longest_stop])
            post_recovery = yaw_error[recovery:][heading_observable[recovery:]]
            result["post_recovery_yaw_rmse_deg"] = (
                float(np.sqrt(np.mean(post_recovery * post_recovery)))
                if len(post_recovery) else None
            )
    return result


def bias_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    truth_accel_names = [f"truth_accel_bias_{axis}_m_s2" for axis in ("x", "y", "z")]
    truth_gyro_names = [f"truth_gyro_bias_{axis}_rad_s" for axis in ("x", "y", "z")]
    estimate_accel_names = [f"eskf_accel_bias_{axis}_m_s2" for axis in ("x", "y", "z")]
    estimate_gyro_names = [f"eskf_gyro_bias_{axis}_rad_s" for axis in ("x", "y", "z")]
    names = truth_accel_names + truth_gyro_names + estimate_accel_names + estimate_gyro_names
    if not all(name in columns for name in names):
        return None
    truth_accel = np.column_stack([columns[name] for name in truth_accel_names])
    truth_gyro = np.column_stack([columns[name] for name in truth_gyro_names])
    if not np.any(np.isfinite(truth_accel)) or not np.any(np.isfinite(truth_gyro)):
        return None
    estimate_accel = np.column_stack([columns[name] for name in estimate_accel_names])
    estimate_gyro = np.column_stack([columns[name] for name in estimate_gyro_names])
    aligned = columns.get("eskf_static_aligned", np.ones(len(truth_accel))) > 0.5
    valid = (
        aligned
        & np.all(np.isfinite(truth_accel), axis=1)
        & np.all(np.isfinite(truth_gyro), axis=1)
        & np.all(np.isfinite(estimate_accel), axis=1)
        & np.all(np.isfinite(estimate_gyro), axis=1)
    )
    if not np.any(valid):
        return None
    accel_error_vector = estimate_accel - truth_accel
    gyro_error_vector = estimate_gyro - truth_gyro
    accel_error = np.linalg.norm(accel_error_vector, axis=1)
    gyro_error = np.linalg.norm(gyro_error_vector, axis=1)
    first = int(np.flatnonzero(valid)[0])
    indices = np.flatnonzero(valid)
    last = int(indices[-1])
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6

    def settling_time(error: np.ndarray, threshold: float) -> float | None:
        post_alignment = indices[indices >= first]
        outside = post_alignment[error[post_alignment] > threshold]
        if len(outside) == 0:
            return 0.0
        candidate = int(outside[-1]) + 1
        remaining = post_alignment[post_alignment >= candidate]
        if len(remaining) == 0:
            return None
        return float(time_s[int(remaining[0])] - time_s[first])

    def continuous_convergence(
        error: np.ndarray, threshold: float, hold_s: float
    ) -> dict[str, object]:
        run_start: int | None = None
        achieved_time: float | None = None
        previous_time: float | None = None
        for index in range(first, last + 1):
            sample_time = float(time_s[index])
            monotonic = previous_time is None or sample_time > previous_time
            if not valid[index] or not monotonic or error[index] > threshold:
                run_start = None
            elif run_start is None:
                run_start = index
            if run_start is not None and sample_time - float(time_s[run_start]) >= hold_s:
                achieved_time = float(time_s[run_start] - time_s[first])
                break
            previous_time = sample_time
        observation_s = float(time_s[last] - time_s[first])
        return {
            "threshold": threshold,
            "required_continuous_duration_s": hold_s,
            "achieved": achieved_time is not None,
            "time_after_alignment_s": achieved_time,
            "right_censored": achieved_time is None,
            "censoring_time_s": observation_s if achieved_time is None else None,
            "observation_duration_after_alignment_s": observation_s,
        }

    terminal_mask = valid & (time_s >= time_s[last] - 5.0)
    terminal_indices = np.flatnonzero(terminal_mask)

    def terminal_summary(
        error_vector: np.ndarray, error_norm: np.ndarray, unit: str
    ) -> dict[str, object]:
        selected = error_vector[terminal_mask]
        norms = error_norm[terminal_mask]
        return {
            "requested_duration_s": 5.0,
            "actual_duration_s": float(
                time_s[int(terminal_indices[-1])] - time_s[int(terminal_indices[0])]
            ),
            "samples": int(len(terminal_indices)),
            f"mean_error_vector_{unit}": np.mean(selected, axis=0).tolist(),
            f"rmse_vector_norm_{unit}": float(np.sqrt(np.mean(norms * norms))),
            f"mean_vector_norm_{unit}": float(np.mean(norms)),
            f"p95_vector_norm_{unit}": float(np.percentile(norms, 95)),
            f"max_vector_norm_{unit}": float(np.max(norms)),
        }

    def per_axis_summary(error_vector: np.ndarray, unit: str) -> dict[str, object]:
        result: dict[str, object] = {}
        for axis_index, axis in enumerate(("x", "y", "z")):
            terminal = error_vector[terminal_mask, axis_index]
            result[axis] = {
                f"error_at_alignment_{unit}": float(error_vector[first, axis_index]),
                f"final_error_{unit}": float(error_vector[last, axis_index]),
                f"terminal_5s_mean_error_{unit}": float(np.mean(terminal)),
                f"terminal_5s_rmse_{unit}": float(np.sqrt(np.mean(terminal * terminal))),
                f"terminal_5s_p95_abs_error_{unit}": float(
                    np.percentile(np.abs(terminal), 95)
                ),
                f"terminal_5s_max_abs_error_{unit}": float(np.max(np.abs(terminal))),
            }
        return result

    def covariance_columns(prefix: str) -> list[str]:
        return [
            f"{prefix}_cov_xx", f"{prefix}_cov_xy", f"{prefix}_cov_xz",
            f"{prefix}_cov_yy", f"{prefix}_cov_yz", f"{prefix}_cov_zz",
        ]

    def covariance_series(prefix: str) -> np.ndarray | None:
        suffix = "m2_s4" if "accel" in prefix else "rad2_s2"
        names = [f"{name}_{suffix}" for name in covariance_columns(prefix)]
        if not all(name in columns for name in names):
            return None
        values = np.column_stack([columns[name] for name in names])
        covariance = np.empty((len(values), 3, 3), dtype=np.float64)
        covariance[:, 0, 0] = values[:, 0]
        covariance[:, 0, 1] = covariance[:, 1, 0] = values[:, 1]
        covariance[:, 0, 2] = covariance[:, 2, 0] = values[:, 2]
        covariance[:, 1, 1] = values[:, 3]
        covariance[:, 1, 2] = covariance[:, 2, 1] = values[:, 4]
        covariance[:, 2, 2] = values[:, 5]
        return covariance

    def bias_nees(
        error_vector: np.ndarray, covariance: np.ndarray
    ) -> tuple[dict[str, object], np.ndarray]:
        nees = np.full(len(error_vector), np.nan, dtype=np.float64)
        covariance_valid = np.zeros(len(error_vector), dtype=bool)
        for index in indices:
            matrix = 0.5 * (covariance[index] + covariance[index].T)
            if not np.all(np.isfinite(matrix)):
                continue
            eigenvalues = np.linalg.eigvalsh(matrix)
            if eigenvalues[-1] <= 0.0 or eigenvalues[0] <= eigenvalues[-1] * 1.0e-12:
                continue
            nees[index] = float(
                error_vector[index] @ np.linalg.solve(matrix, error_vector[index])
            )
            covariance_valid[index] = True
        summary: dict[str, object] = _consistency_summary(nees[valid], 3)
        finite = nees[valid & np.isfinite(nees)]
        terminal = nees[terminal_mask & np.isfinite(nees)]
        summary.update(
            {
                "median": float(np.median(finite)) if len(finite) else float("nan"),
                "terminal_5s_mean": (
                    float(np.mean(terminal)) if len(terminal) else float("nan")
                ),
                "invalid_covariance_samples": int(np.count_nonzero(valid & ~covariance_valid)),
            }
        )
        return summary, covariance_valid

    result: dict[str, object] = {
        "alignment_time_s": float(time_s[first]),
        "truth_accel_bias_m_s2": truth_accel[first].tolist(),
        "truth_gyro_bias_rad_s": truth_gyro[first].tolist(),
        "accel_error_at_alignment_m_s2": float(accel_error[first]),
        "gyro_error_at_alignment_rad_s": float(gyro_error[first]),
        "accel_final_error_m_s2": float(accel_error[last]),
        "gyro_final_error_rad_s": float(gyro_error[last]),
        "accel_error_rmse_m_s2": float(np.sqrt(np.mean(accel_error[valid] ** 2))),
        "gyro_error_rmse_rad_s": float(np.sqrt(np.mean(gyro_error[valid] ** 2))),
        "accel_error_reduction_ratio": float(
            1.0 - accel_error[last] / max(accel_error[first], 1.0e-12)
        ),
        "gyro_error_reduction_ratio": float(
            1.0 - gyro_error[last] / max(gyro_error[first], 1.0e-12)
        ),
        "accel_settling_time_below_0_05_m_s2_s": settling_time(accel_error, 0.05),
        "gyro_settling_time_below_0_001_rad_s_s": settling_time(gyro_error, 0.001),
        "accel_continuous_5s_convergence": continuous_convergence(accel_error, 0.05, 5.0),
        "gyro_continuous_5s_convergence": continuous_convergence(gyro_error, 0.001, 5.0),
        "accel_terminal_5s": terminal_summary(accel_error_vector, accel_error, "m_s2"),
        "gyro_terminal_5s": terminal_summary(gyro_error_vector, gyro_error, "rad_s"),
        "accel_axes": per_axis_summary(accel_error_vector, "m_s2"),
        "gyro_axes": per_axis_summary(gyro_error_vector, "rad_s"),
        "bias_consistency_available": False,
        "tilt_accel_bias_joint_nees": None,
        "tilt_accel_bias_joint_nees_status": (
            "not_reported: the runner does not export the complete attitude-bias 6x6 "
            "covariance in the ESKF right-error tangent frame"
        ),
    }
    accel_covariance = covariance_series("eskf_accel_bias")
    gyro_covariance = covariance_series("eskf_gyro_bias")
    if accel_covariance is not None and gyro_covariance is not None:
        accel_nees, accel_covariance_valid = bias_nees(accel_error_vector, accel_covariance)
        gyro_nees, gyro_covariance_valid = bias_nees(gyro_error_vector, gyro_covariance)
        accel_terminal_covariance = terminal_mask & accel_covariance_valid
        gyro_terminal_covariance = terminal_mask & gyro_covariance_valid
        result.update(
            {
                "bias_consistency_available": True,
                "accel_bias_nees": accel_nees,
                "gyro_bias_nees": gyro_nees,
                "accel_bias_final_covariance_m2_s4": accel_covariance[last].tolist(),
                "gyro_bias_final_covariance_rad2_s2": gyro_covariance[last].tolist(),
                "accel_bias_terminal_5s_mean_covariance_m2_s4": (
                    np.mean(accel_covariance[accel_terminal_covariance], axis=0).tolist()
                    if np.any(accel_terminal_covariance) else None
                ),
                "gyro_bias_terminal_5s_mean_covariance_rad2_s2": (
                    np.mean(gyro_covariance[gyro_terminal_covariance], axis=0).tolist()
                    if np.any(gyro_terminal_covariance) else None
                ),
            }
        )
    return result


def eskf_integrity_metrics(columns: dict[str, np.ndarray]) -> dict[str, object]:
    input_mag = columns.get("input_mag_update", np.ones(len(columns["ts_us"]))) > 0.5
    accepted = columns["eskf_mag_accepted"] > 0.5
    innovation = columns.get("eskf_mag_innovation_rad", np.zeros(len(accepted)))
    test_ratio = columns.get("eskf_mag_test_ratio", np.zeros(len(accepted)))
    attempted = input_mag & (accepted | (np.abs(innovation) > 1.0e-12) | (test_ratio > 1.0e-12))
    rejected = attempted & ~accepted
    rejected_degrees = np.degrees(np.abs(innovation[rejected]))
    input_count = int(np.count_nonzero(input_mag))
    attempted_count = int(np.count_nonzero(attempted))
    navigation_valid = columns.get(
        "eskf_horizontal_navigation_valid", np.zeros(len(accepted))
    ) > 0.5
    aiding_age = columns.get(
        "eskf_horizontal_aiding_age_s", np.full(len(accepted), np.inf)
    )
    finite_aiding_age = aiding_age[np.isfinite(aiding_age)]
    position_aiding_age = columns.get(
        "eskf_horizontal_position_aiding_age_s", np.full(len(accepted), np.inf)
    )
    velocity_aiding_age = columns.get(
        "eskf_horizontal_velocity_aiding_age_s", np.full(len(accepted), np.inf)
    )
    finite_position_aiding_age = position_aiding_age[np.isfinite(position_aiding_age)]
    finite_velocity_aiding_age = velocity_aiding_age[np.isfinite(velocity_aiding_age)]
    result: dict[str, object] = {
        "magnetometer_source_updates": input_count,
        "magnetometer_outer_gate_pass_ratio": (
            attempted_count / input_count if input_count else None
        ),
        "magnetometer_innovation_acceptance_ratio": (
            float(np.count_nonzero(accepted & attempted)) / attempted_count if attempted_count else None
        ),
        "rejected_heading_innovation_p95_deg": (
            float(np.percentile(rejected_degrees, 95)) if len(rejected_degrees) else None
        ),
        "healthy_ratio": float(np.mean(columns.get("eskf_healthy", np.ones(len(accepted))))),
        "navigation_recoveries": int(np.max(columns.get("eskf_navigation_recovery_count", [0]))),
        "static_alignment_completed": bool(np.max(columns.get("eskf_static_aligned", [0])) > 0.5),
        "static_tilt_alignment_completed": bool(
            np.max(columns.get("eskf_static_tilt_aligned", [0])) > 0.5
        ),
        "static_heading_alignment_completed": bool(
            np.max(columns.get("eskf_static_heading_aligned", [0])) > 0.5
        ),
        "zero_velocity_updates": int(np.max(columns.get("eskf_zupt_count", [0]))),
        "horizontal_navigation_valid_ratio": float(np.mean(navigation_valid)),
        "horizontal_navigation_valid_samples": int(np.count_nonzero(navigation_valid)),
        "maximum_finite_horizontal_aiding_age_s": (
            float(np.max(finite_aiding_age)) if len(finite_aiding_age) else None
        ),
        "maximum_finite_horizontal_position_aiding_age_s": (
            float(np.max(finite_position_aiding_age))
            if len(finite_position_aiding_age) else None
        ),
        "maximum_finite_horizontal_velocity_aiding_age_s": (
            float(np.max(finite_velocity_aiding_age))
            if len(finite_velocity_aiding_age) else None
        ),
    }
    return result


def navigation_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    if "position_ref_valid" not in columns:
        return None
    valid = columns["position_ref_valid"] > 0.5
    if not np.any(valid):
        return None
    estimate_p = np.column_stack(
        [columns[f"eskf_position_{axis}_m"] for axis in ("n", "e", "d")]
    )
    reference_p = np.column_stack(
        [columns[f"ref_position_{axis}_m"] for axis in ("n", "e", "d")]
    )
    estimate_v = np.column_stack(
        [columns[f"eskf_velocity_{axis}_m_s"] for axis in ("n", "e", "d")]
    )
    reference_v = np.column_stack(
        [columns[f"ref_velocity_{axis}_m_s"] for axis in ("n", "e", "d")]
    )
    position_error = estimate_p[valid] - reference_p[valid]
    velocity_error = estimate_v[valid] - reference_v[valid]
    position_updates = columns.get("input_position_update", np.zeros(len(valid))) > 0.5
    velocity_updates = columns.get("input_velocity_update", position_updates) > 0.5
    return {
        "reference_samples": int(np.count_nonzero(valid)),
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(position_error * position_error, axis=1)))),
        "position_p95_m": float(np.percentile(np.linalg.norm(position_error, axis=1), 95)),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum(velocity_error * velocity_error, axis=1)))),
        "gps_updates": int(np.count_nonzero(position_updates | velocity_updates)),
        "position_updates": int(np.count_nonzero(position_updates)),
        "velocity_updates": int(np.count_nonzero(velocity_updates)),
        "position_acceptance_ratio": (
            float(np.count_nonzero(
                (columns.get("eskf_position_accepted", 0) > 0.5) & position_updates
            )) / np.count_nonzero(position_updates)
            if np.any(position_updates) else None
        ),
        "velocity_acceptance_ratio": (
            float(np.count_nonzero(
                (columns.get("eskf_velocity_accepted", 0) > 0.5) & velocity_updates
            )) / np.count_nonzero(velocity_updates)
            if np.any(velocity_updates) else None
        ),
    }


def navigation_aiding_gap_metrics(
    columns: dict[str, np.ndarray], minimum_gap_s: float = 5.0,
    recovery_threshold_m: float = 10.0, recovery_hold_s: float = 5.0,
) -> dict[str, object] | None:
    """Measure drift and reacquisition separately from ordinary aided accuracy.

    Overall trajectory RMSE can hide whether error comes from normal GNSS operation or one long
    recorded outage. This diagnostic uses only the recorded update-validity timeline and the
    independent reference; it does not synthesize an outage or assume that a rejected fix existed.
    """
    required = [
        "position_ref_valid", "input_position_update",
        *[f"eskf_position_{axis}_m" for axis in ("n", "e", "d")],
        *[f"ref_position_{axis}_m" for axis in ("n", "e", "d")],
        *[f"eskf_velocity_{axis}_m_s" for axis in ("n", "e", "d")],
        *[f"ref_velocity_{axis}_m_s" for axis in ("n", "e", "d")],
    ]
    if not all(name in columns for name in required):
        return None
    valid = columns["position_ref_valid"] > 0.5
    updates = (columns["input_position_update"] > 0.5) & valid
    update_indices = np.flatnonzero(updates)
    if len(update_indices) < 2:
        return None
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    position_error = np.linalg.norm(
        np.column_stack([columns[f"eskf_position_{axis}_m"] for axis in ("n", "e", "d")])
        - np.column_stack([columns[f"ref_position_{axis}_m"] for axis in ("n", "e", "d")]),
        axis=1,
    )
    velocity_error = np.linalg.norm(
        np.column_stack([columns[f"eskf_velocity_{axis}_m_s"] for axis in ("n", "e", "d")])
        - np.column_stack([columns[f"ref_velocity_{axis}_m_s"] for axis in ("n", "e", "d")]),
        axis=1,
    )
    nominal_interval_s = float(np.median(np.diff(time_s[update_indices])))
    last_update_time = np.full(len(time_s), -np.inf, dtype=np.float64)
    last_update_time[update_indices] = time_s[update_indices]
    last_update_time = np.maximum.accumulate(last_update_time)
    aiding_available = valid & ((time_s - last_update_time) <= 1.5 * nominal_interval_s)

    details: list[dict[str, object]] = []
    for start, resume in zip(update_indices[:-1], update_indices[1:]):
        gap_s = float(time_s[resume] - time_s[start])
        if gap_s < minimum_gap_s:
            continue
        segment = slice(int(start), int(resume) + 1)
        peak_offset = int(np.argmax(position_error[segment]))
        peak_index = int(start) + peak_offset
        recovery_time_s: float | None = None
        for candidate in np.flatnonzero(
            (np.arange(len(time_s)) >= resume) & valid
            & (position_error <= recovery_threshold_m)
        ):
            hold_end = int(np.searchsorted(
                time_s, time_s[candidate] + recovery_hold_s, side="left"
            ))
            if hold_end >= len(time_s):
                break
            if np.all(position_error[candidate : hold_end + 1] <= recovery_threshold_m):
                recovery_time_s = float(time_s[candidate] - time_s[resume])
                break
        details.append(
            {
                "last_update_time_s": float(time_s[start]),
                "resume_update_time_s": float(time_s[resume]),
                "update_interval_s": gap_s,
                "missing_nominal_epochs": max(0, int(round(gap_s / nominal_interval_s)) - 1),
                "position_error_before_gap_m": float(position_error[start]),
                "peak_position_error_m": float(position_error[peak_index]),
                "peak_position_error_time_s": float(time_s[peak_index]),
                "peak_velocity_error_m_s": float(np.max(velocity_error[segment])),
                "position_error_after_first_resume_update_m": float(position_error[resume]),
                "resume_position_nis": (
                    float(columns["eskf_position_nis"][resume])
                    if "eskf_position_nis" in columns else None
                ),
                "sustained_recovery_threshold_m": recovery_threshold_m,
                "sustained_recovery_hold_s": recovery_hold_s,
                "sustained_recovery_time_s": recovery_time_s,
            }
        )
    if not details:
        return None
    details.sort(key=lambda item: float(item["update_interval_s"]), reverse=True)
    result: dict[str, object] = {
        "interpretation": (
            "Recorded position-update gaps are scored separately from nominal aided operation; "
            "the first resumed row contains the posterior after that update."
        ),
        "nominal_update_interval_s": nominal_interval_s,
        "minimum_reported_gap_s": minimum_gap_s,
        "reported_gaps": len(details),
        "aiding_available_ratio": float(np.mean(aiding_available[valid])),
        "aided_position_rmse_m": float(np.sqrt(np.mean(position_error[aiding_available] ** 2))),
        "unaided_position_rmse_m": float(np.sqrt(np.mean(position_error[valid & ~aiding_available] ** 2))),
        "longest_gap": details[0],
        "gaps": details,
    }
    return result


def _consistency_summary(values: np.ndarray, degrees_of_freedom: int) -> dict[str, float | int]:
    finite = values[np.isfinite(values)]
    lower, upper = CHI_SQUARE_95[degrees_of_freedom]
    if len(finite) == 0:
        return {
            "samples": 0,
            "degrees_of_freedom": degrees_of_freedom,
            "expected_mean": float(degrees_of_freedom),
            "mean": float("nan"),
            "p95": float("nan"),
            "single_sample_95_coverage": float("nan"),
        }
    return {
        "samples": int(len(finite)),
        "degrees_of_freedom": degrees_of_freedom,
        "expected_mean": float(degrees_of_freedom),
        "mean": float(np.mean(finite)),
        "p95": float(np.percentile(finite, 95)),
        "single_sample_95_coverage": float(np.mean((finite >= lower) & (finite <= upper))),
    }


def consistency_metrics(
    columns: dict[str, np.ndarray], reference_kind: str
) -> dict[str, object] | None:
    if "eskf_position_nis" not in columns or "eskf_velocity_nis" not in columns:
        return None
    position_updates = columns.get(
        "input_position_update", np.zeros(len(columns["ts_us"]))
    ) > 0.5
    velocity_updates = columns.get("input_velocity_update", position_updates) > 0.5
    if not np.any(position_updates | velocity_updates):
        return None
    result: dict[str, object] = {
        "interpretation": (
            "NIS uses each pre-update GNSS innovation. Single-sample chi-square coverage is "
            "diagnostic because consecutive samples are time-correlated."
        ),
        "position_nis": _consistency_summary(
            columns["eskf_position_nis"][position_updates], 3
        ),
        "velocity_nis": _consistency_summary(
            columns["eskf_velocity_nis"][velocity_updates], 3
        ),
    }
    if reference_kind in ("synthetic", "independent_truth") and "eskf_navigation_nees" in columns:
        result["navigation_nees"] = _consistency_summary(
            columns["eskf_navigation_nees"][position_updates | velocity_updates], 6
        )
        result["navigation_nees_scope"] = (
            "Posterior [velocity, position] 6-state error against the declared truth source."
        )
    return result


def create_plot(columns: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    figure, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    colors = {"mahony_standard": "#8b8b8b", "mahony_robust": "#0072b2", "eskf": "#d55e00"}
    reset_events = columns.get("ref_attitude_reset_event", np.zeros(len(time_s))) > 0.5

    for row, axis_name in enumerate(AXES):
        truth = columns[f"truth_{axis_name}_deg"]
        axes[row, 0].plot(time_s, truth, color="black", linewidth=1.5, label="reference")
        for algorithm in ALGORITHMS:
            estimate = columns[f"{algorithm}_{axis_name}_deg"]
            axes[row, 0].plot(time_s, estimate, color=colors[algorithm], linewidth=0.9, label=algorithm)
            axes[row, 1].plot(
                time_s, wrapped_error_deg(estimate, truth), color=colors[algorithm], linewidth=0.9,
                label=algorithm,
            )
        for event_time in time_s[reset_events]:
            axes[row, 0].axvline(event_time, color="#8e44ad", alpha=0.35, linewidth=0.8)
            axes[row, 1].axvline(event_time, color="#8e44ad", alpha=0.35, linewidth=0.8)
        axes[row, 0].set_ylabel(f"{axis_name} (deg)")
        axes[row, 1].set_ylabel(f"{axis_name} error (deg)")
        axes[row, 0].grid(alpha=0.25); axes[row, 1].grid(alpha=0.25)
    axes[0, 0].legend(ncol=2, fontsize=8); axes[0, 1].legend(ncol=2, fontsize=8)
    axes[2, 0].set_xlabel("time (s)"); axes[2, 1].set_xlabel("time (s)")
    figure.suptitle(title); figure.tight_layout(); figure.savefig(output_path, dpi=150)
    plt.close(figure)


def create_yaw_diagnostic_plot(columns: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    raw_reference = columns["truth_yaw_deg"]
    compensated_reference = reset_compensated_reference_yaw(columns)
    estimate = columns["eskf_yaw_deg"]
    reset_events = columns.get("ref_attitude_reset_event", np.zeros(len(time_s))) > 0.5
    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].plot(time_s, raw_reference, color="black", linewidth=1.2, label="PX4 raw yaw")
    axes[0].plot(
        time_s, compensated_reference, color="#8e44ad", linewidth=1.1,
        label="PX4 reset-compensated yaw",
    )
    axes[0].plot(time_s, estimate, color="#d55e00", linewidth=0.9, label="Aerakia ESKF")
    gsf_valid = (
        (columns.get("px4_gsf_yaw_valid", np.zeros(len(time_s))) > 0.5)
        & (columns.get("px4_gsf_yaw_update", np.zeros(len(time_s))) > 0.5)
        & (columns.get("px4_gsf_yaw_variance_rad2", np.ones(len(time_s)))
           <= np.radians(15.0) ** 2)
        & (columns.get("gnss_ground_speed_m_s", np.zeros(len(time_s))) >= 1.5)
    )
    if np.any(gsf_valid):
        axes[0].plot(
            time_s[gsf_valid], np.degrees(columns["px4_gsf_yaw_rad"][gsf_valid]),
            color="#009e73", linewidth=0.0, marker=".", markersize=2.5,
            alpha=0.8, label="PX4 GSF confident updates",
        )
    offset = _initial_alignment_offset(time_s, estimate, compensated_reference)
    axes[1].plot(
        time_s, wrapped_error_deg(estimate + offset, compensated_reference),
        color="#d55e00", linewidth=0.9, label="ESKF minus reset-compensated PX4",
    )
    axes[1].plot(
        time_s, wrapped_error_deg(estimate, raw_reference),
        color="#777777", linewidth=0.7, alpha=0.75, label="ESKF minus raw PX4",
    )
    for event_time in time_s[reset_events]:
        for axis in axes:
            axis.axvline(event_time, color="#8e44ad", alpha=0.4, linewidth=0.9)
    axes[0].set_ylabel("yaw (deg)")
    axes[1].set_ylabel("wrapped error (deg)")
    axes[1].set_xlabel("time (s)")
    axes[0].legend(ncol=2, fontsize=8)
    axes[1].legend(ncol=2, fontsize=8)
    axes[0].grid(alpha=0.25); axes[1].grid(alpha=0.25)
    figure.suptitle(title); figure.tight_layout(); figure.savefig(output_path, dpi=150)
    plt.close(figure)


def create_navigation_diagnostic_plot(
    columns: dict[str, np.ndarray], output_path: Path, title: str
) -> None:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    position_error = np.linalg.norm(
        np.column_stack([columns[f"eskf_position_{axis}_m"] for axis in ("n", "e", "d")])
        - np.column_stack([columns[f"ref_position_{axis}_m"] for axis in ("n", "e", "d")]),
        axis=1,
    )
    velocity_error = np.linalg.norm(
        np.column_stack([columns[f"eskf_velocity_{axis}_m_s"] for axis in ("n", "e", "d")])
        - np.column_stack([columns[f"ref_velocity_{axis}_m_s"] for axis in ("n", "e", "d")]),
        axis=1,
    )
    updates = columns.get("input_position_update", np.zeros(len(time_s))) > 0.5
    update_indices = np.flatnonzero(updates)
    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].semilogy(time_s, np.maximum(position_error, 1.0e-3), color="#d55e00")
    axes[1].semilogy(time_s, np.maximum(velocity_error, 1.0e-4), color="#0072b2")
    if len(update_indices) >= 2:
        intervals = np.diff(time_s[update_indices])
        for gap_index in np.flatnonzero(intervals >= 5.0):
            start = time_s[update_indices[gap_index]]
            stop = time_s[update_indices[gap_index + 1]]
            for axis in axes:
                axis.axvspan(start, stop, color="#cc79a7", alpha=0.18, label="recorded aid gap")
    axes[0].set_ylabel("3D position error (m)")
    axes[1].set_ylabel("3D velocity error (m/s)")
    axes[1].set_xlabel("time (s)")
    for axis in axes:
        axis.grid(alpha=0.25, which="both")
    if any(patch.get_label() == "recorded aid gap" for patch in axes[0].patches):
        axes[0].legend(fontsize=8)
    figure.suptitle(title); figure.tight_layout(); figure.savefig(output_path, dpi=150)
    plt.close(figure)


def write_markdown(
    path: Path, scenario: str, metrics: dict[str, object], plot_name: str | None,
    yaw_plot_name: str | None = None, navigation_plot_name: str | None = None,
) -> None:
    algorithms = metrics["algorithms"]
    assert isinstance(algorithms, dict)
    lines = [
        f"# Validation report: {scenario}", "",
        "| Algorithm | Full attitude RMSE | Tilt RMSE | Yaw RMSE |",
        "| --- | ---: | ---: | ---: |",
    ]
    for algorithm in ALGORITHMS:
        values = algorithms[algorithm]
        lines.append(
            f"| `{algorithm}` | {values['overall_attitude_rmse_deg']:.4f}° "
            f"| {values['tilt_rmse_deg']:.4f}° | {values['axes']['yaw']['rmse_deg']:.4f}° |"
        )
    lines.extend(
        [
            "",
            "Full attitude uses quaternion geodesic angle and tilt uses gravity-direction angle "
            "when quaternion outputs are available. Per-axis Euler errors remain diagnostics and "
            "must not be combined near pitch singularities.",
        ]
    )
    integrity = metrics["eskf_integrity"]
    navigation = metrics.get("navigation")
    lines.extend(["", "## ESKF integrity", ""])
    lines.append(
        f"- Navigation recoveries: {integrity['navigation_recoveries']}; "
        f"ZUPT updates: {integrity['zero_velocity_updates']}."
    )
    lines.append(
        f"- Static alignment completed: {integrity['static_alignment_completed']}; "
        f"healthy ratio: {integrity['healthy_ratio']:.4f}."
    )
    lines.append(
        f"- Horizontal navigation valid ratio: "
        f"{integrity['horizontal_navigation_valid_ratio']:.4f}; valid samples: "
        f"{integrity['horizontal_navigation_valid_samples']}."
    )
    cold_start = metrics.get("cold_start_alignment")
    if cold_start:
        lines.append(
            f"- Cold-start tilt alignment completed at "
            f"{cold_start['tilt_alignment_time_s']:.3f} s; post-alignment tilt RMSE: "
            f"{cold_start['post_tilt_alignment_tilt_rmse_deg']:.4f}°."
        )
        if cold_start["heading_alignment_completed"]:
            lines.append(
                f"- Heading alignment completed at "
                f"{cold_start['alignment_time_s']:.3f} s; post-alignment attitude RMSE: "
                f"{cold_start['post_alignment_attitude_rmse_deg']:.4f}°."
            )
        else:
            lines.append("- Heading alignment did not complete because no accepted heading source was present.")
    if navigation:
        lines.append(
            f"- Position RMSE against the declared navigation reference: "
            f"{navigation['position_rmse_m']:.3f} m."
        )
    aiding_gaps = metrics.get("navigation_aiding_gaps")
    if aiding_gaps:
        longest = aiding_gaps["longest_gap"]
        recovery = longest.get("sustained_recovery_time_s")
        recovery_text = "not reached" if recovery is None else f"{recovery:.3f} s"
        lines.extend(
            [
                "", "## Recorded navigation-aiding gaps", "",
                f"- Longest recorded position-update interval: "
                f"{longest['update_interval_s']:.3f} s; estimated missing nominal epochs: "
                f"{longest['missing_nominal_epochs']}.",
                f"- Position error before gap / peak / after first resumed update: "
                f"{longest['position_error_before_gap_m']:.3f} / "
                f"{longest['peak_position_error_m']:.3f} / "
                f"{longest['position_error_after_first_resume_update_m']:.3f} m.",
                f"- Sustained recovery below "
                f"{longest['sustained_recovery_threshold_m']:.1f} m for "
                f"{longest['sustained_recovery_hold_s']:.1f} s: {recovery_text}.",
                f"- Aided / unaided position RMSE: "
                f"{aiding_gaps['aided_position_rmse_m']:.3f} / "
                f"{aiding_gaps['unaided_position_rmse_m']:.3f} m.",
            ]
        )
    trusted_heading = metrics.get("trusted_heading")
    if trusted_heading:
        normal_acceptance = trusted_heading.get("normal_acceptance_ratio")
        fault_rejection = trusted_heading.get("fault_rejection_ratio")
        normal_text = "n/a" if normal_acceptance is None else f"{normal_acceptance:.3%}"
        fault_text = "n/a (no declared fault samples)" if fault_rejection is None \
            else f"{fault_rejection:.3%}"
        lines.extend(
            [
                "", "## Trusted-heading behavior", "",
                f"- Normal accepted updates: {normal_text}; fault rejection: {fault_text}.",
                f"- Longest dropout: {trusted_heading.get('dropout_duration_s', 0.0):.3f} s; "
                f"maximum yaw error during dropout: "
                f"{trusted_heading.get('dropout_max_abs_yaw_error_deg', 0.0):.3f}°.",
                f"- First accepted recovery delay: "
                f"{trusted_heading.get('recovery_time_s', 0.0):.3f} s; post-recovery yaw RMSE: "
                f"{trusted_heading.get('post_recovery_yaw_rmse_deg', 0.0):.3f}°.",
            ]
        )
    bias = metrics.get("bias_estimation")
    if bias:
        accel_convergence = bias["accel_continuous_5s_convergence"]
        gyro_convergence = bias["gyro_continuous_5s_convergence"]
        accel_convergence_text = (
            f"{accel_convergence['time_after_alignment_s']:.3f} s"
            if accel_convergence["achieved"] else
            f"right-censored at {accel_convergence['censoring_time_s']:.3f} s"
        )
        gyro_convergence_text = (
            f"{gyro_convergence['time_after_alignment_s']:.3f} s"
            if gyro_convergence["achieved"] else
            f"right-censored at {gyro_convergence['censoring_time_s']:.3f} s"
        )
        lines.extend(
            [
                "", "## IMU bias estimation", "",
                f"- Accelerometer bias vector error: {bias['accel_error_at_alignment_m_s2']:.5f} "
                f"m/s² at alignment; {bias['accel_final_error_m_s2']:.5f} m/s² final.",
                f"- Gyroscope bias vector error: {bias['gyro_error_at_alignment_rad_s']:.6f} "
                f"rad/s at alignment; {bias['gyro_final_error_rad_s']:.6f} rad/s final.",
                f"- First continuous 5 s below the accelerometer/gyroscope thresholds: "
                f"{accel_convergence_text} / {gyro_convergence_text} after alignment.",
                "- A single stationary pose cannot independently identify horizontal accelerometer "
                "bias and tilt; this metric reports the resulting error rather than hiding it.",
            ]
        )
        if bias.get("bias_consistency_available"):
            accel_nees = bias["accel_bias_nees"]
            gyro_nees = bias["gyro_bias_nees"]
            lines.extend(
                [
                    f"- Accelerometer-bias 3D NEES mean: {accel_nees['mean']:.3f} "
                    f"(expected 3, n={accel_nees['samples']}, invalid covariance="
                    f"{accel_nees['invalid_covariance_samples']}).",
                    f"- Gyroscope-bias 3D NEES mean: {gyro_nees['mean']:.3f} "
                    f"(expected 3, n={gyro_nees['samples']}, invalid covariance="
                    f"{gyro_nees['invalid_covariance_samples']}).",
                    "- Tilt-plus-accelerometer-bias joint NEES is not reported because the full "
                    "right-error 6x6 cross-covariance is not exported.",
                ]
            )
    consistency = metrics.get("eskf_consistency")
    if consistency:
        position_nis = consistency["position_nis"]
        velocity_nis = consistency["velocity_nis"]
        lines.extend(
            [
                "", "## ESKF consistency diagnostics", "",
                f"- Position NIS mean: {position_nis['mean']:.3f} "
                f"(expected {position_nis['expected_mean']:.0f}, "
                f"n={position_nis['samples']}).",
                f"- Velocity NIS mean: {velocity_nis['mean']:.3f} "
                f"(expected {velocity_nis['expected_mean']:.0f}, "
                f"n={velocity_nis['samples']}).",
            ]
        )
        if "navigation_nees" in consistency:
            nees = consistency["navigation_nees"]
            lines.append(
                f"- Posterior 6-state navigation NEES mean: {nees['mean']:.3f} "
                f"(expected {nees['expected_mean']:.0f}, n={nees['samples']})."
            )
    reset = metrics["reference_resets"]
    reset_compensated = metrics.get("eskf_reset_compensated_yaw")
    segment_drift = metrics.get("eskf_segment_aligned_yaw")
    yaw_sources = metrics.get("yaw_sources", {})
    reset_source = "PX4 reference" if metrics["reference_kind"] == "px4_estimate" else "Reference"
    lines.extend(
        [
            "", "## Reference resets", "",
            f"{reset_source} reset events: {int(reset['events'])}; maximum reset rotation: "
            f"{reset['maximum_rotation_deg']:.3f}°.",
        ]
    )
    if reset_compensated and segment_drift:
        lines.extend(
            [
                f"- ESKF yaw RMSE against raw PX4 yaw: "
                f"{algorithms['eskf']['axes']['yaw']['rmse_deg']:.3f}°.",
                f"- ESKF yaw RMSE after removing logged PX4 reset deltas: "
                f"{reset_compensated['rmse_deg']:.3f}°.",
                f"- Per-segment aligned yaw drift RMSE: {segment_drift['rmse_deg']:.3f}° "
                "(diagnostic only; every PX4 reset segment gets a new offset).",
            ]
        )
    lines.extend(
        [
            "", "## Yaw-source audit", "",
            f"- Direct dual-GNSS heading updates fused by Aerakia: "
            f"{yaw_sources.get('direct_gnss_heading_updates', 0)}.",
            f"- GNSS course-over-ground diagnostic updates (never fused as body yaw): "
            f"{yaw_sources.get('gnss_course_diagnostic_updates', 0)}.",
            f"- PX4 GSF diagnostic updates (never fused into Aerakia during comparison): "
            f"{yaw_sources.get('px4_gsf_updates', 0)}; confident moving updates: "
            f"{yaw_sources.get('px4_gsf_confident_updates', 0)}.",
        ]
    )
    if plot_name is not None:
        lines.extend(["", f"![Attitude estimates and wrapped errors]({plot_name})", ""])
    if yaw_plot_name is not None:
        lines.extend([f"![Yaw reset and source diagnostics]({yaw_plot_name})", ""])
    if navigation_plot_name is not None:
        lines.extend([f"![Navigation error and recorded aiding gaps]({navigation_plot_name})", ""])
    reference_kind = metrics["reference_kind"]
    if reference_kind == "px4_estimate":
        lines.extend(
            [
                "> PX4 estimates are an engineering reference, not ground truth or an airworthiness claim.",
                "> Ordinary single-antenna GNSS course is direction of travel, not guaranteed vehicle heading.",
            ]
        )
    elif reference_kind == "independent_truth":
        lines.append(
            "> External truth is subject to the source dataset's calibration, synchronization, and observability limits."
        )
    elif reference_kind == "shared_sensor_reference":
        lines.append(
            "> The estimator input and scoring reference share physical measurements; results validate integration behavior, not independent absolute accuracy."
        )
    elif reference_kind == "external_reference":
        lines.append(
            "> At least one scored channel uses a separately recorded external reference; channel-level independence and shared-source limitations remain defined by the dataset manifest."
        )
    else:
        lines.append("> Synthetic truth validates implementation behavior, not physical flight reliability.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_csv", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="validation")
    parser.add_argument(
        "--no-plots", action="store_true",
        help="write metrics and Markdown without generating PNGs",
    )
    parser.add_argument(
        "--reference-kind",
        choices=(
            "synthetic",
            "independent_truth",
            "external_reference",
            "shared_sensor_reference",
            "px4_estimate",
        ),
        default="synthetic",
    )
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    columns = load_columns(args.results_csv)
    metrics: dict[str, object] = {
        "scenario": args.scenario,
        "reference_kind": args.reference_kind,
        "samples": int(len(columns["seq"])),
        "duration_s": float((columns["ts_us"][-1] - columns["ts_us"][0]) * 1.0e-6),
        "algorithms": {
            algorithm: metrics_for(columns, algorithm, args.reference_kind) for algorithm in ALGORITHMS
        },
        "eskf_integrity": eskf_integrity_metrics(columns),
        "navigation": navigation_metrics(columns),
        "navigation_aiding_gaps": navigation_aiding_gap_metrics(columns),
        "eskf_consistency": consistency_metrics(columns, args.reference_kind),
        "reference_resets": reset_summary(columns),
        "trusted_heading": trusted_heading_metrics(columns),
        "bias_estimation": bias_metrics(columns),
    }
    if args.reference_kind == "independent_truth":
        metrics["mahony_fallback_envelope"] = fallback_envelope_metrics(columns)
    cold_start = cold_start_alignment_metrics(columns)
    if cold_start is not None:
        metrics["cold_start_alignment"] = cold_start
    if args.reference_kind == "px4_estimate":
        metrics["eskf_reset_compensated_yaw"] = reset_compensated_yaw_metrics(columns)
        metrics["eskf_segment_aligned_yaw"] = segment_aligned_yaw_metrics(columns)
        metrics["yaw_sources"] = yaw_source_diagnostics(columns)
    plot_path: Path | None = None
    if not args.no_plots:
        plot_path = args.out_dir / "attitude_comparison.png"
        create_plot(columns, plot_path, f"Aerakia validation — {args.scenario}")
    yaw_plot_path: Path | None = None
    if args.reference_kind == "px4_estimate" and not args.no_plots:
        yaw_plot_path = args.out_dir / "yaw_diagnostics.png"
        create_yaw_diagnostic_plot(columns, yaw_plot_path, f"Yaw diagnostics — {args.scenario}")
    navigation_plot_path: Path | None = None
    if metrics["navigation"] is not None and not args.no_plots:
        navigation_plot_path = args.out_dir / "navigation_diagnostics.png"
        create_navigation_diagnostic_plot(
            columns, navigation_plot_path, f"Navigation diagnostics — {args.scenario}"
        )
    (args.out_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_markdown(
        args.out_dir / "report.md", args.scenario, metrics,
        plot_path.name if plot_path is not None else None,
        yaw_plot_path.name if yaw_plot_path is not None else None,
        navigation_plot_path.name if navigation_plot_path is not None else None,
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
