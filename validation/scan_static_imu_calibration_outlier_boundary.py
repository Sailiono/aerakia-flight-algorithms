#!/usr/bin/env python3
"""Scan the public six-pose calibrator's deterministic outlier boundary.

This is a diagnostic characterization, not threshold tuning and not a claim
that the calibrator detects arbitrary bad windows.  One pose is contaminated
along either a tangential or radial direction while the other five remain
exact six-axis means.  The report records acceptance, fitted-bias error,
gravity residual, and leave-one-out influence for every amplitude.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path


GRAVITY = 9.80665
TRUE_BIAS = (0.10, -0.05, 0.03)
POSES = ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
         (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
         (0.0, 0.0, 1.0), (0.0, 0.0, -1.0))


def run_case(calibrator: Path, mode: str, amplitude: float) -> dict[str, object]:
    rows = ["acc_x_m_s2,acc_y_m_s2,acc_z_m_s2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,sample_count"]
    for index, direction in enumerate(POSES):
        acceleration = [TRUE_BIAS[axis] + direction[axis] * GRAVITY for axis in range(3)]
        if index == 2:
            if mode == "tangential":
                acceleration[0] += amplitude
            else:
                acceleration[1] += amplitude
        rows.append(",".join(str(value) for value in (*acceleration, 0.0, 0.0, 0.0, 400)))
    with tempfile.TemporaryDirectory(prefix="aerakia-static-calibration-scan-") as temporary:
        root = Path(temporary)
        source = root / "poses.csv"
        result_path = root / "result.json"
        source.write_text("\n".join(rows) + "\n", encoding="utf-8")
        completed = subprocess.run(
            [str(calibrator), str(source), str(result_path)],
            capture_output=True, text=True, check=False,
        )
        result = json.loads(result_path.read_text(encoding="utf-8"))
    fitted = result["accelerometer_bias_m_s2"]
    error = math.sqrt(sum((float(fitted[i]) - TRUE_BIAS[i]) ** 2 for i in range(3)))
    return {
        "mode": mode,
        "amplitude_m_s2": amplitude,
        "returncode": completed.returncode,
        "status": result["status"],
        "accepted": bool(result["accepted"]),
        "bias_error_norm_m_s2": error,
        "gravity_residual_max_abs_m_s2": result["gravity_residual_max_abs_m_s2"],
        "leave_one_out_bias_delta_m_s2": result["maximum_leave_one_out_bias_delta_m_s2"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibrator", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("validation/public/static_imu_calibration_outlier_boundary_v1.json"))
    parser.add_argument("--amplitudes", default="0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.12,0.15,0.2,0.3,0.5,0.8,1.0")
    args = parser.parse_args()
    calibrator = args.calibrator.resolve()
    if not calibrator.is_file():
        parser.error("calibrator must be an existing executable")
    amplitudes = [float(item) for item in args.amplitudes.split(",")]
    if any(value < 0.0 or not math.isfinite(value) for value in amplitudes):
        parser.error("amplitudes must be finite and non-negative")
    cases = [run_case(calibrator, mode, amplitude)
             for mode in ("tangential", "radial") for amplitude in amplitudes]
    output = {
        "schema_version": 1,
        "study_id": "aerakia-static-imu-calibration-outlier-boundary-v1",
        "status": "diagnostic_only",
        "true_bias_m_s2": list(TRUE_BIAS),
        "contaminated_pose": "+Y",
        "interpretation": [
            "This scan characterizes the frozen default gates; it does not tune them.",
            "Accepted cases with non-zero bias error are false accepts for a stricter calibration-accuracy contract.",
            "The result does not establish physical IMU calibration accuracy or flight readiness.",
        ],
        "calibrator_sha256": __import__("hashlib").sha256(calibrator.read_bytes()).hexdigest(),
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "cases": cases,
    }
    output_path = args.out if args.out.is_absolute() else Path(__file__).resolve().parents[1] / args.out
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
