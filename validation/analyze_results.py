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


def wrapped_error_deg(estimate: np.ndarray, truth: np.ndarray) -> np.ndarray:
    return (estimate - truth + 180.0) % 360.0 - 180.0


def circular_mean_deg(angle: np.ndarray) -> float:
    radians = np.radians(angle)
    return float(np.degrees(np.arctan2(np.mean(np.sin(radians)), np.mean(np.cos(radians)))))


def load_columns(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError(f"missing CSV header: {path}")
        columns = {name: [] for name in reader.fieldnames}
        for row_number, row in enumerate(reader, start=2):
            if None in row:
                raise ValueError(f"extra CSV value at row {row_number}: {path}")
            for name in reader.fieldnames:
                value = row.get(name)
                if value in (None, ""):
                    raise ValueError(
                        f"missing CSV value for {name!r} at row {row_number}: {path}"
                    )
                try:
                    columns[name].append(float(value))
                except ValueError as exc:
                    raise ValueError(
                        f"invalid CSV value for {name!r} at row {row_number}: {value!r}"
                    ) from exc
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

    result: dict[str, object] = {
        "overall_attitude_rmse_deg": float(np.sqrt(np.mean(np.column_stack(squared_errors)))),
        "tilt_rmse_deg": float(np.sqrt(np.mean(np.column_stack(roll_pitch_squared)))),
        "initial_yaw_alignment_offset_deg": yaw_offset,
        "axes": axes,
    }
    if algorithm == "mahony_robust":
        result["minimum_accelerometer_weight"] = float(np.min(columns["mahony_robust_acc_weight"]))
        result["mean_magnetometer_weight"] = float(np.mean(columns["mahony_robust_mag_weight"]))
    return result


def reset_aware_yaw_metrics(columns: dict[str, np.ndarray]) -> dict[str, object]:
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


def _quaternion_yaw_deg(columns: dict[str, np.ndarray], index: int) -> float | None:
    names = [f"ref_delta_q_reset_{axis}" for axis in ("w", "x", "y", "z")]
    if not all(name in columns for name in names):
        return None
    q = np.asarray([columns[name][index] for name in names], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return None
    w, x, y, z = q / norm
    return float(
        np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    )


def continuous_yaw_reference(
    columns: dict[str, np.ndarray],
) -> tuple[np.ndarray, list[dict[str, float | int | None]]]:
    """Remove observed PX4 reset jumps while preserving inter-reset motion.

    The sample-to-sample PX4 yaw jump is used instead of treating delta_q_reset
    yaw as an exact Euler-angle delta. The quaternion-derived value is retained
    as an independent consistency check in the event diagnostics.
    """

    truth = columns["truth_yaw_deg"]
    estimate = columns["eskf_yaw_deg"]
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    events = columns.get("ref_attitude_reset_event")
    if events is None:
        counters = columns.get("ref_attitude_reset_counter", np.zeros(len(truth)))
        event_mask = np.r_[False, counters[1:] != counters[:-1]]
    else:
        event_mask = events > 0.5

    correction = np.zeros(len(truth), dtype=np.float64)
    cumulative_jump = 0.0
    diagnostics: list[dict[str, float | int | None]] = []
    counters = columns.get("ref_attitude_reset_counter", np.zeros(len(truth)))
    for index in np.flatnonzero(event_mask):
        observed_jump: float | None = None
        estimate_jump: float | None = None
        if index > 0:
            observed_jump = float(wrapped_error_deg(truth[index:index + 1], truth[index - 1:index])[0])
            estimate_jump = float(
                wrapped_error_deg(estimate[index:index + 1], estimate[index - 1:index])[0]
            )
            cumulative_jump += observed_jump
        correction[index:] = cumulative_jump
        diagnostics.append(
            {
                "sample_index": int(index),
                "time_s": float(time_s[index]),
                "counter": int(counters[index]),
                "observed_reference_jump_deg": observed_jump,
                "quaternion_yaw_delta_deg": _quaternion_yaw_deg(columns, int(index)),
                "aerakia_sample_jump_deg": estimate_jump,
            }
        )
    return (truth - correction + 180.0) % 360.0 - 180.0, diagnostics


def continuous_reference_yaw_metrics(columns: dict[str, np.ndarray]) -> dict[str, object]:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    estimate = columns["eskf_yaw_deg"]
    raw_reference = columns["truth_yaw_deg"]
    continuous_reference, events = continuous_yaw_reference(columns)
    alignment_offset = _initial_alignment_offset(time_s, estimate, continuous_reference)
    aligned_estimate = estimate + alignment_offset
    error = wrapped_error_deg(aligned_estimate, continuous_reference)
    raw_error = wrapped_error_deg(aligned_estimate, raw_reference)

    for event in events:
        index = int(event["sample_index"])
        event["raw_error_before_deg"] = float(raw_error[index - 1]) if index > 0 else None
        event["raw_error_after_deg"] = float(raw_error[index])
        event["continuous_error_before_deg"] = float(error[index - 1]) if index > 0 else None
        event["continuous_error_after_deg"] = float(error[index])

    summary: dict[str, object] = _error_summary(error)
    summary.update(
        {
            "initial_yaw_alignment_offset_deg": alignment_offset,
            "events": events,
            "method": "subtract_observed_px4_yaw_jump_at_attitude_reset",
            "interpretation": (
                "comparative continuous-reference metric; PX4 remains an estimate, not truth"
            ),
        }
    )
    return summary


def _masked_yaw_comparison(
    time_s: np.ndarray,
    estimate: np.ndarray,
    reference: np.ndarray,
    valid: np.ndarray,
) -> dict[str, float]:
    valid_indices = np.flatnonzero(valid)
    first_time = float(time_s[valid_indices[0]])
    alignment = valid & (time_s <= first_time + 5.0)
    if np.count_nonzero(alignment) < 2:
        alignment = np.zeros(len(valid), dtype=bool)
        alignment[valid_indices[: min(10, len(valid_indices))]] = True
    offset = circular_mean_deg(wrapped_error_deg(reference[alignment], estimate[alignment]))
    summary = _error_summary(wrapped_error_deg(estimate[valid] + offset, reference[valid]))
    summary["initial_alignment_offset_deg"] = offset
    return summary


def gsf_yaw_comparison_metrics(columns: dict[str, np.ndarray]) -> dict[str, object] | None:
    if "ref_gsf_yaw_valid" not in columns:
        return None
    valid = columns["ref_gsf_yaw_valid"] > 0.5
    valid &= np.isfinite(columns["ref_gsf_yaw_deg"])
    if np.count_nonzero(valid) < 10:
        return None
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    gsf_yaw = columns["ref_gsf_yaw_deg"]
    continuous_px4, _ = continuous_yaw_reference(columns)
    variance = columns.get("ref_gsf_yaw_variance_rad2", np.zeros(len(valid)))
    return {
        "reference_samples": int(np.count_nonzero(valid)),
        "maximum_reported_sigma_deg": float(np.degrees(np.sqrt(np.max(variance[valid])))),
        "eskf": _masked_yaw_comparison(time_s, columns["eskf_yaw_deg"], gsf_yaw, valid),
        "px4_raw": _masked_yaw_comparison(time_s, columns["truth_yaw_deg"], gsf_yaw, valid),
        "px4_continuous": _masked_yaw_comparison(time_s, continuous_px4, gsf_yaw, valid),
        "interpretation": "GNSS-velocity-aided PX4 GSF comparison, not independent truth",
    }


def reset_summary(columns: dict[str, np.ndarray]) -> dict[str, float]:
    events = columns.get("ref_attitude_reset_event", np.zeros(len(columns["ts_us"]))) > 0.5
    result = {
        "events": float(np.count_nonzero(events)),
        "maximum_rotation_deg": 0.0,
        "maximum_yaw_delta_deg": 0.0,
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
    yaw = np.degrees(
        np.arctan2(
            2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
            1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2),
        )
    )
    result["maximum_yaw_delta_deg"] = float(np.max(np.abs(yaw)))
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
        "zero_velocity_updates": int(np.max(columns.get("eskf_zupt_count", [0]))),
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
    gps_updates = columns.get("input_position_update", np.zeros(len(valid))) > 0.5
    return {
        "reference_samples": int(np.count_nonzero(valid)),
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(position_error * position_error, axis=1)))),
        "position_p95_m": float(np.percentile(np.linalg.norm(position_error, axis=1), 95)),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum(velocity_error * velocity_error, axis=1)))),
        "gps_updates": int(np.count_nonzero(gps_updates)),
        "position_acceptance_ratio": (
            float(np.count_nonzero((columns.get("eskf_position_accepted", 0) > 0.5) & gps_updates))
            / np.count_nonzero(gps_updates)
            if np.any(gps_updates) else None
        ),
        "velocity_acceptance_ratio": (
            float(np.count_nonzero((columns.get("eskf_velocity_accepted", 0) > 0.5) & gps_updates))
            / np.count_nonzero(gps_updates)
            if np.any(gps_updates) else None
        ),
    }


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


def create_yaw_reset_plot(columns: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    time_s = (columns["ts_us"] - columns["ts_us"][0]) * 1.0e-6
    raw_reference = columns["truth_yaw_deg"]
    continuous_reference, _ = continuous_yaw_reference(columns)
    estimate = columns["eskf_yaw_deg"]
    offset = _initial_alignment_offset(time_s, estimate, continuous_reference)
    aligned_estimate = (estimate + offset + 180.0) % 360.0 - 180.0
    reset_events = columns.get("ref_attitude_reset_event", np.zeros(len(time_s))) > 0.5

    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].plot(time_s, raw_reference, color="#777777", linewidth=1.0, label="PX4 raw yaw")
    axes[0].plot(
        time_s, continuous_reference, color="black", linewidth=1.5,
        label="PX4 de-reset continuous yaw",
    )
    axes[0].plot(time_s, aligned_estimate, color="#d55e00", linewidth=0.9, label="Aerakia ESKF")
    axes[1].plot(
        time_s, wrapped_error_deg(aligned_estimate, raw_reference), color="#777777",
        linewidth=0.9, label="error vs raw PX4",
    )
    axes[1].plot(
        time_s, wrapped_error_deg(aligned_estimate, continuous_reference), color="#d55e00",
        linewidth=0.9, label="error vs continuous PX4",
    )
    for axis in axes:
        for event_time in time_s[reset_events]:
            axis.axvline(event_time, color="#8e44ad", alpha=0.45, linewidth=0.8)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    axes[0].set_ylabel("yaw (deg)")
    axes[1].set_ylabel("wrapped error (deg)")
    axes[1].set_xlabel("time (s)")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def write_markdown(
    path: Path, scenario: str, metrics: dict[str, object], plot_name: str,
    yaw_reset_plot_name: str | None = None,
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
    if navigation:
        lines.append(
            f"- Position RMSE against PX4 local-position reference: "
            f"{navigation['position_rmse_m']:.3f} m."
        )
    reset = metrics["reference_resets"]
    lines.extend(
        [
            "", "## Reference resets", "",
            f"PX4 reference reset events: {int(reset['events'])}; maximum reset rotation: "
            f"{reset['maximum_rotation_deg']:.3f}°. Reset-aware segment metrics are diagnostic "
            "and are not directly comparable to the globally aligned raw metric.",
        ]
    )
    continuous = metrics.get("eskf_continuous_reference_yaw")
    if continuous:
        lines.append(
            f"- ESKF yaw RMSE against the globally aligned, de-reset PX4 reference: "
            f"{continuous['rmse_deg']:.3f}°. This removes observed PX4 reset jumps but does not "
            "turn the PX4 estimate into independent heading truth."
        )
    gsf = metrics.get("gsf_yaw_comparison")
    if gsf:
        lines.append(
            f"- Against converged GNSS-velocity-aided GSF yaw, globally aligned RMSE is "
            f"{gsf['eskf']['rmse_deg']:.3f}° for Aerakia ESKF, "
            f"{gsf['px4_raw']['rmse_deg']:.3f}° for raw PX4 yaw, and "
            f"{gsf['px4_continuous']['rmse_deg']:.3f}° for de-reset PX4 yaw."
        )
    lines.extend(
        [
            "", f"![Attitude estimates and wrapped errors]({plot_name})", "",
        ]
    )
    if yaw_reset_plot_name:
        lines.extend([f"![Yaw reset diagnostics]({yaw_reset_plot_name})", ""])
    lines.extend(
        [
            "> PX4 estimates are an engineering reference, not ground truth or an airworthiness claim.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_csv", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="validation")
    parser.add_argument("--reference-kind", choices=("synthetic", "px4_estimate"), default="synthetic")
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
        "reference_resets": reset_summary(columns),
    }
    if args.reference_kind == "px4_estimate":
        metrics["eskf_reset_aware_yaw"] = reset_aware_yaw_metrics(columns)
        metrics["eskf_continuous_reference_yaw"] = continuous_reference_yaw_metrics(columns)
        metrics["gsf_yaw_comparison"] = gsf_yaw_comparison_metrics(columns)
    plot_path = args.out_dir / "attitude_comparison.png"
    create_plot(columns, plot_path, f"Aerakia validation — {args.scenario}")
    yaw_reset_plot_path: Path | None = None
    if args.reference_kind == "px4_estimate":
        yaw_reset_plot_path = args.out_dir / "yaw_reset_diagnostics.png"
        create_yaw_reset_plot(
            columns, yaw_reset_plot_path, f"Aerakia yaw reset diagnostics — {args.scenario}"
        )
    (args.out_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_markdown(
        args.out_dir / "report.md", args.scenario, metrics, plot_path.name,
        yaw_reset_plot_path.name if yaw_reset_plot_path else None,
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
