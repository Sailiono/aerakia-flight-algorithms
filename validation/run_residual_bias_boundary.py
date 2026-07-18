#!/usr/bin/env python3
"""Exercise every reviewed residual-IMU-bias boundary direction deterministically."""

from __future__ import annotations

import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np


DIMENSIONS = (
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
)


def build_boundary_cases() -> list[dict[str, object]]:
    """Return zero, isolated-axis, isolated-pair, and exhaustive six-axis corners."""
    cases: list[dict[str, object]] = []

    def add_case(case_id: str, family: str, sigma_vector: tuple[int, ...]) -> None:
        cases.append({
            "id": case_id,
            "family": family,
            "sigma_vector": list(sigma_vector),
        })

    zero = (0,) * len(DIMENSIONS)
    add_case("nominal_zero", "nominal", zero)
    for index, name in enumerate(DIMENSIONS):
        for sign, label in ((-1, "neg"), (1, "pos")):
            vector = list(zero)
            vector[index] = sign
            add_case(f"axis__{name}__{label}", "axis", tuple(vector))
    for first, second in itertools.combinations(range(len(DIMENSIONS)), 2):
        for first_sign, second_sign in itertools.product((-1, 1), repeat=2):
            vector = list(zero)
            vector[first] = first_sign
            vector[second] = second_sign
            first_label = "neg" if first_sign < 0 else "pos"
            second_label = "neg" if second_sign < 0 else "pos"
            add_case(
                f"pair__{DIMENSIONS[first]}_{first_label}__"
                f"{DIMENSIONS[second]}_{second_label}",
                "pairwise",
                tuple(vector),
            )
    for signs in itertools.product((-1, 1), repeat=len(DIMENSIONS)):
        label = "".join("n" if sign < 0 else "p" for sign in signs)
        add_case(f"corner__{label}", "full_corner", signs)
    return cases


def coverage_summary(cases: list[dict[str, object]]) -> dict[str, object]:
    family_counts: dict[str, int] = {}
    axis_coverage: set[tuple[int, int]] = set()
    pair_coverage: set[tuple[int, int, int, int]] = set()
    corners: set[tuple[int, ...]] = set()
    ids: set[str] = set()
    duplicates: list[str] = []
    for case in cases:
        case_id = str(case["id"])
        if case_id in ids:
            duplicates.append(case_id)
        ids.add(case_id)
        family = str(case["family"])
        family_counts[family] = family_counts.get(family, 0) + 1
        vector = tuple(int(value) for value in case["sigma_vector"])
        active = [index for index, value in enumerate(vector) if value]
        if family == "axis" and len(active) == 1:
            axis_coverage.add((active[0], vector[active[0]]))
        if family == "pairwise" and len(active) == 2:
            first, second = active
            pair_coverage.add((first, second, vector[first], vector[second]))
        if family == "full_corner" and len(active) == len(DIMENSIONS):
            corners.add(vector)

    expected_axis = {
        (index, sign) for index in range(len(DIMENSIONS)) for sign in (-1, 1)
    }
    expected_pairs = {
        (first, second, first_sign, second_sign)
        for first, second in itertools.combinations(range(len(DIMENSIONS)), 2)
        for first_sign, second_sign in itertools.product((-1, 1), repeat=2)
    }
    expected_corners = set(itertools.product((-1, 1), repeat=len(DIMENSIONS)))
    missing_axis = sorted(expected_axis - axis_coverage)
    missing_pairs = sorted(expected_pairs - pair_coverage)
    missing_corners = sorted(expected_corners - corners)
    complete = (
        len(cases) == 137
        and family_counts == {
            "nominal": 1, "axis": 12, "pairwise": 60, "full_corner": 64,
        }
        and not duplicates
        and not missing_axis
        and not missing_pairs
        and not missing_corners
    )
    return {
        "complete": complete,
        "total_cases": len(cases),
        "family_counts": family_counts,
        "axis_sign_boundaries_expected": len(expected_axis),
        "axis_sign_boundaries_covered": len(axis_coverage),
        "isolated_pair_sign_combinations_expected": len(expected_pairs),
        "isolated_pair_sign_combinations_covered": len(pair_coverage),
        "full_corners_expected": len(expected_corners),
        "full_corners_covered": len(corners),
        "duplicate_ids": duplicates,
        "missing_axis_sign_boundaries": missing_axis,
        "missing_pair_sign_combinations": missing_pairs,
        "missing_full_corners": missing_corners,
    }


def physical_bias_vectors(
    case: dict[str, object],
    accel_sigma_m_s2: float,
    gyro_sigma_deg_s: float,
    sigma_limit: float,
) -> tuple[list[float], list[float]]:
    sigma_vector = [int(value) for value in case["sigma_vector"]]
    accel = [value * sigma_limit * accel_sigma_m_s2 for value in sigma_vector[:3]]
    gyro = [value * sigma_limit * gyro_sigma_deg_s for value in sigma_vector[3:]]
    return accel, gyro


def suite_command(
    *,
    root: Path,
    runner: Path,
    case_dir: Path,
    accel_bias: list[float],
    gyro_bias: list[float],
    rate_hz: float,
    seed: int,
) -> list[str]:
    return [
        sys.executable,
        str(root / "validation" / "run_suite.py"),
        "--runner", str(runner),
        "--out-dir", str(case_dir),
        "--scenarios", "bias_convergence",
        "--rate", str(rate_hz),
        "--seed", str(seed),
        "--accel-bias-vector-m-s2", *(str(value) for value in accel_bias),
        "--gyro-bias-vector-deg-s", *(str(value) for value in gyro_bias),
        "--rate-invariant-streams",
        "--no-plots",
    ]


def extract_metrics(metrics: dict[str, object]) -> dict[str, float | int | None]:
    alignment = metrics["cold_start_alignment"]
    navigation = metrics["navigation"]
    consistency = metrics["eskf_consistency"]
    integrity = metrics["eskf_integrity"]
    bias = metrics["bias_estimation"]
    assert isinstance(alignment, dict)
    assert isinstance(navigation, dict)
    assert isinstance(consistency, dict)
    assert isinstance(integrity, dict)
    assert isinstance(bias, dict)
    navigation_nees = consistency["navigation_nees"]
    assert isinstance(navigation_nees, dict)
    return {
        "attitude_rmse_deg": float(alignment["post_alignment_attitude_rmse_deg"]),
        "position_rmse_m": float(navigation["position_rmse_m"]),
        "velocity_rmse_m_s": float(navigation["velocity_rmse_m_s"]),
        "navigation_nees_mean": float(navigation_nees["mean"]),
        "accel_error_at_alignment_m_s2": float(bias["accel_error_at_alignment_m_s2"]),
        "accel_final_error_m_s2": float(bias["accel_final_error_m_s2"]),
        "accel_error_reduction_ratio": float(bias["accel_error_reduction_ratio"]),
        "accel_settling_time_s": (
            None if bias["accel_settling_time_below_0_05_m_s2_s"] is None
            else float(bias["accel_settling_time_below_0_05_m_s2_s"])
        ),
        "gyro_error_at_alignment_rad_s": float(bias["gyro_error_at_alignment_rad_s"]),
        "gyro_final_error_rad_s": float(bias["gyro_final_error_rad_s"]),
        "gyro_settling_time_s": (
            None if bias["gyro_settling_time_below_0_001_rad_s_s"] is None
            else float(bias["gyro_settling_time_below_0_001_rad_s_s"])
        ),
        "healthy_ratio": float(integrity["healthy_ratio"]),
        "navigation_recoveries": int(integrity["navigation_recoveries"]),
    }


def reviewed_limits(threshold_path: Path) -> dict[str, dict[str, float]]:
    thresholds = json.loads(threshold_path.read_text(encoding="utf-8"))
    bias = thresholds["bias_convergence"]
    return {
        "attitude_rmse_deg": {"maximum": float(bias["eskf_post_alignment"])},
        "position_rmse_m": {"maximum": float(bias["position_rmse_m"])},
        "velocity_rmse_m_s": {"maximum": float(bias["velocity_rmse_m_s"])},
        "navigation_nees_mean": {
            "minimum": float(bias["navigation_nees_mean_min"]),
            "maximum": float(bias["navigation_nees_mean"]),
        },
        "accel_final_error_m_s2": {
            "maximum": float(bias["bias_accel_final_error_m_s2"]),
        },
        "accel_error_reduction_ratio": {
            "minimum": float(bias["bias_accel_error_reduction_ratio_min"]),
        },
        "accel_settling_time_s": {
            "maximum": float(bias["bias_accel_settling_time_below_0_05_m_s2_s"]),
        },
        "gyro_final_error_rad_s": {
            "maximum": float(bias["bias_gyro_final_error_rad_s"]),
        },
        "gyro_settling_time_s": {
            "maximum": float(bias["bias_gyro_settling_time_below_0_001_rad_s_s"]),
        },
        "healthy_ratio": {"minimum": 1.0},
        "navigation_recoveries": {"maximum": 0.0},
    }


def metric_failures(
    metrics: dict[str, float | int | None],
    limits: dict[str, dict[str, float]],
    *,
    accel_active: bool,
    gyro_active: bool,
) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    skipped: list[str] = []
    for name, bounds in limits.items():
        if name.startswith("accel_") and not accel_active:
            skipped.append(f"{name}: no injected accelerometer bias")
            continue
        if name.startswith("gyro_") and not gyro_active:
            skipped.append(f"{name}: no injected gyroscope bias")
            continue
        if (
            name == "accel_error_reduction_ratio"
            and float(metrics["accel_error_at_alignment_m_s2"])
            <= limits["accel_final_error_m_s2"]["maximum"]
        ):
            skipped.append(
                "accel_error_reduction_ratio: alignment error already met the absolute final-error limit"
            )
            continue
        value = metrics[name]
        if value is None:
            failures.append(f"{name}=null")
            continue
        numeric = float(value)
        if "minimum" in bounds and numeric < bounds["minimum"]:
            failures.append(f"{name}={numeric:.9g}<{bounds['minimum']:.9g}")
        if "maximum" in bounds and numeric > bounds["maximum"]:
            failures.append(f"{name}={numeric:.9g}>{bounds['maximum']:.9g}")
    return failures, skipped


def run_case(
    case: dict[str, object],
    *,
    root: Path,
    runner: Path,
    out_dir: Path,
    accel_sigma_m_s2: float,
    gyro_sigma_deg_s: float,
    sigma_limit: float,
    rate_hz: float,
    seed: int,
    limits: dict[str, dict[str, float]],
    timeout_s: float,
    compact: bool,
) -> dict[str, object]:
    case_id = str(case["id"])
    case_dir = out_dir / "cases" / case_id
    accel_bias, gyro_bias = physical_bias_vectors(
        case, accel_sigma_m_s2, gyro_sigma_deg_s, sigma_limit
    )
    command = suite_command(
        root=root,
        runner=runner,
        case_dir=case_dir,
        accel_bias=accel_bias,
        gyro_bias=gyro_bias,
        rate_hz=rate_hz,
        seed=seed,
    )
    environment = os.environ.copy()
    environment["MPLCONFIGDIR"] = str((case_dir / ".matplotlib").resolve())
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=root,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        timeout=timeout_s,
    )
    duration_s = time.monotonic() - started
    record: dict[str, object] = {
        **case,
        "acceleration_bias_m_s2": accel_bias,
        "gyroscope_bias_deg_s": gyro_bias,
        "command": command,
        "runtime_s": duration_s,
        "returncode": completed.returncode,
        "metrics": None,
        "failures": [],
    }
    failures: list[str] = []
    if completed.returncode != 0:
        failures.append(f"suite_returncode={completed.returncode}")
        record["output_tail"] = completed.stdout[-4000:]
    else:
        scenario_dir = case_dir / "bias_convergence"
        metrics = json.loads((scenario_dir / "metrics.json").read_text(encoding="utf-8"))
        generation = json.loads(
            (scenario_dir / "input-metadata.json").read_text(encoding="utf-8")
        )
        extracted = extract_metrics(metrics)
        record["metrics"] = extracted
        record["generation"] = generation
        actual_accel = np.asarray(generation["acceleration_bias_m_s2"], dtype=np.float64)
        actual_gyro = np.asarray(generation["gyroscope_bias_deg_s"], dtype=np.float64)
        if generation.get("acceleration_bias_source") != "explicit_vector":
            failures.append("accelerometer override source was not explicit_vector")
        if generation.get("gyroscope_bias_source") != "explicit_vector":
            failures.append("gyroscope override source was not explicit_vector")
        if not np.array_equal(actual_accel, np.asarray(accel_bias, dtype=np.float64)):
            failures.append("generated accelerometer bias differs from requested boundary")
        if not np.array_equal(actual_gyro, np.asarray(gyro_bias, dtype=np.float64)):
            failures.append("generated gyroscope bias differs from requested boundary")
        metric_gate_failures, skipped_gates = metric_failures(
            extracted,
            limits,
            accel_active=any(value != 0.0 for value in accel_bias),
            gyro_active=any(value != 0.0 for value in gyro_bias),
        )
        failures.extend(metric_gate_failures)
        record["skipped_gates"] = skipped_gates
    record["failures"] = failures
    record["status"] = "passed" if not failures else "failed"
    if compact and case_dir.exists():
        shutil.rmtree(case_dir)
    return record


def aggregate_metrics(records: list[dict[str, object]]) -> dict[str, object]:
    available = [record for record in records if isinstance(record.get("metrics"), dict)]
    if not available:
        return {}
    names = list(available[0]["metrics"])
    aggregates: dict[str, object] = {}
    for name in names:
        pairs = [
            (str(record["id"]), float(record["metrics"][name]))
            for record in available
            if record["metrics"][name] is not None
        ]
        if not pairs:
            aggregates[name] = {"finite_count": 0}
            continue
        values = np.asarray([value for _, value in pairs], dtype=np.float64)
        minimum_index = int(np.argmin(values))
        maximum_index = int(np.argmax(values))
        aggregates[name] = {
            "finite_count": len(pairs),
            "minimum": float(values[minimum_index]),
            "minimum_case": pairs[minimum_index][0],
            "mean": float(np.mean(values)),
            "maximum": float(values[maximum_index]),
            "maximum_case": pairs[maximum_index][0],
        }
    return aggregates


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("build/residual-bias-boundary"))
    parser.add_argument("--thresholds", type=Path)
    parser.add_argument("--accel-sigma-m-s2", type=float, default=0.05)
    parser.add_argument("--gyro-sigma-deg-s", type=float, default=0.20)
    parser.add_argument("--sigma-limit", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()

    if (
        args.accel_sigma_m_s2 <= 0.0
        or args.gyro_sigma_deg_s <= 0.0
        or args.sigma_limit <= 0.0
        or args.rate <= 0.0
        or args.jobs < 1
        or args.timeout_s <= 0.0
    ):
        parser.error("sigmas, sigma limit, rate, jobs, and timeout must be positive")

    root = Path(__file__).resolve().parents[1]
    out_dir = args.out_dir if args.out_dir.is_absolute() else (root / args.out_dir)
    threshold_path = args.thresholds or (root / "validation" / "thresholds.json")
    if not threshold_path.is_absolute():
        threshold_path = root / threshold_path
    runner = args.runner.resolve()
    cases = build_boundary_cases()
    coverage = coverage_summary(cases)
    if not coverage["complete"]:
        raise SystemExit(f"internal boundary coverage is incomplete: {coverage}")
    limits = reviewed_limits(threshold_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    started = time.monotonic()
    records: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = {
            executor.submit(
                run_case,
                case,
                root=root,
                runner=runner,
                out_dir=out_dir,
                accel_sigma_m_s2=args.accel_sigma_m_s2,
                gyro_sigma_deg_s=args.gyro_sigma_deg_s,
                sigma_limit=args.sigma_limit,
                rate_hz=args.rate,
                seed=args.seed,
                limits=limits,
                timeout_s=args.timeout_s,
                compact=args.compact,
            ): str(case["id"])
            for case in cases
        }
        completed_count = 0
        for future in as_completed(futures):
            case_id = futures[future]
            try:
                records.append(future.result())
            except Exception as error:
                records.append({
                    "id": case_id,
                    "status": "execution_error",
                    "metrics": None,
                    "failures": [f"{type(error).__name__}: {error}"],
                })
            completed_count += 1
            if completed_count == 1 or completed_count % 20 == 0 or completed_count == len(cases):
                print(f"completed {completed_count}/{len(cases)} boundary cases")
    order = {str(case["id"]): index for index, case in enumerate(cases)}
    records.sort(key=lambda record: order[str(record["id"])])

    failures = [record for record in records if record["failures"]]
    family_results: dict[str, dict[str, int]] = {}
    for record in records:
        family = str(record.get("family", "unknown"))
        counts = family_results.setdefault(family, {"total": 0, "passed": 0, "failed": 0})
        counts["total"] += 1
        counts["passed" if not record["failures"] else "failed"] += 1
    elapsed_s = time.monotonic() - started
    output = {
        "schema_version": 1,
        "status": "passed" if not failures else "failed",
        "scope": {
            "scenario": "bias_convergence",
            "deterministic_seed": args.seed,
            "rate_hz": args.rate,
            "accel_sigma_m_s2": args.accel_sigma_m_s2,
            "gyro_sigma_deg_s": args.gyro_sigma_deg_s,
            "sigma_limit": args.sigma_limit,
            "accel_boundary_m_s2": args.accel_sigma_m_s2 * args.sigma_limit,
            "gyro_boundary_deg_s": args.gyro_sigma_deg_s * args.sigma_limit,
            "same_noise_realization_for_every_case": True,
            "covered": [
                "zero-bias baseline",
                "each of six bias dimensions at positive and negative boundary",
                "all four sign combinations for every isolated dimension pair",
                "all 64 simultaneous six-dimensional boundary corners",
                "cold start, multi-axis excitation, GNSS aiding, bias convergence, and consistency",
            ],
            "not_covered": [
                "bias magnitude outside the declared residual calibration boundary",
                "temperature-dependent or time-varying bias",
                "scale-factor, non-orthogonality, vibration rectification, or physical sensor truth",
            ],
        },
        "coverage": coverage,
        "reviewed_limits": limits,
        "family_results": family_results,
        "failure_count": len(failures),
        "runtime_s": elapsed_s,
        "aggregate_metrics": aggregate_metrics(records),
        "cases": records,
    }
    (out_dir / "summary.json").write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    aggregate = output["aggregate_metrics"]
    lines = [
        "# Deterministic residual-bias boundary campaign",
        "",
        f"Status: **{output['status']}**; cases: **{len(records)}**; failures: "
        f"**{len(failures)}**; runtime: **{elapsed_s:.1f} s**.",
        "",
        f"Boundary: accelerometer `±{args.accel_sigma_m_s2 * args.sigma_limit:.6g} m/s²`; "
        f"gyroscope `±{args.gyro_sigma_deg_s * args.sigma_limit:.6g} deg/s` "
        f"(`±{args.sigma_limit:g}σ` per axis).",
        "",
        "| Family | Cases | Passed | Failed |",
        "| --- | ---: | ---: | ---: |",
    ]
    for family in ("nominal", "axis", "pairwise", "full_corner", "unknown"):
        if family in family_results:
            counts = family_results[family]
            lines.append(
                f"| {family} | {counts['total']} | {counts['passed']} | {counts['failed']} |"
            )
    lines.extend(["", "| Metric | Minimum | Mean | Maximum |", "| --- | ---: | ---: | ---: |"])
    for name, values in aggregate.items():
        if values.get("finite_count", 0):
            lines.append(
                f"| {name} | {values['minimum']:.6g} | {values['mean']:.6g} "
                f"| {values['maximum']:.6g} |"
            )
    if failures:
        lines.extend(["", "## Failures", ""])
        for record in failures:
            lines.append(f"- `{record['id']}`: {'; '.join(record['failures'])}")
    lines.extend([
        "",
        "Every case uses the same deterministic measurement-noise realization. This campaign "
        "validates the declared constant residual-bias boundary; it is not evidence for thermal "
        "drift, calibration errors outside that boundary, or physical-flight qualification.",
    ])
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Residual-bias boundary report: {out_dir / 'report.md'}")
    if failures:
        raise SystemExit(f"residual-bias boundary failed in {len(failures)}/{len(records)} cases")


if __name__ == "__main__":
    main()
