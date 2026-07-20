from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
import analyze_magnetic_source_ab as diagnostic  # noqa: E402


REPLAY_HEADER = [
    "ts_us", "raw_mag_cuT_x", "raw_mag_cuT_y", "raw_mag_cuT_z", "mag_valid", "mag_update",
    "magnetic_declination_rad", "ref_q_w", "ref_q_x", "ref_q_y", "ref_q_z", "raw_acc_mg_x",
    "raw_acc_mg_y", "raw_acc_mg_z", "raw_gyro_mdps_x", "raw_gyro_mdps_y", "raw_gyro_mdps_z",
]
RESULT_HEADER = [
    "ts_us", "input_mag_update", "eskf_mag_accepted", "eskf_mag_innovation_rad", "eskf_mag_test_ratio",
    "truth_roll_deg", "truth_pitch_deg", "truth_yaw_deg", "eskf_roll_deg", "eskf_pitch_deg", "eskf_yaw_deg",
    "truth_q_w", "truth_q_x", "truth_q_y", "truth_q_z", "eskf_q_w", "eskf_q_x", "eskf_q_y", "eskf_q_z",
]


def write_fixture(replay_path: Path, on_path: Path, off_path: Path) -> None:
    with replay_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=REPLAY_HEADER)
        writer.writeheader()
        for index, heading_deg in enumerate((0.0, 4.0, 10.0, 20.0)):
            heading_rad = math.radians(heading_deg)
            writer.writerow({
                "ts_us": index * 10000,
                "raw_mag_cuT_x": 5000.0 * math.cos(heading_rad),
                "raw_mag_cuT_y": 5000.0 * math.sin(heading_rad),
                "raw_mag_cuT_z": 2000.0,
                "mag_valid": 1,
                "mag_update": 1,
                "magnetic_declination_rad": 0.0,
                "ref_q_w": 1.0,
                "ref_q_x": 0.0,
                "ref_q_y": 0.0,
                "ref_q_z": 0.0,
                "raw_acc_mg_x": 0.0,
                "raw_acc_mg_y": 0.0,
                "raw_acc_mg_z": -1000.0,
                "raw_gyro_mdps_x": 0.0,
                "raw_gyro_mdps_y": 0.0,
                "raw_gyro_mdps_z": 0.0,
            })

    for path, enabled, yaw_errors, tilt_x in (
        (on_path, 1, (0.0, 3.0, 8.0, 15.0), (0.0, 0.03, 0.06, 0.10)),
        (off_path, 0, (0.0, 0.2, 0.2, 0.2), (0.0, 0.0, 0.0, 0.0)),
    ):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=RESULT_HEADER)
            writer.writeheader()
            for index, (yaw_error, qx) in enumerate(zip(yaw_errors, tilt_x)):
                writer.writerow({
                    "ts_us": index * 10000,
                    "input_mag_update": enabled,
                    "eskf_mag_accepted": enabled,
                    "eskf_mag_innovation_rad": math.radians(yaw_error),
                    "eskf_mag_test_ratio": 0.5 if enabled else 0.0,
                    "truth_roll_deg": 0.0,
                    "truth_pitch_deg": 0.0,
                    "truth_yaw_deg": 0.0,
                    "eskf_roll_deg": 0.0,
                    "eskf_pitch_deg": 0.0,
                    "eskf_yaw_deg": yaw_error,
                    "truth_q_w": 1.0,
                    "truth_q_x": 0.0,
                    "truth_q_y": 0.0,
                    "truth_q_z": 0.0,
                    "eskf_q_w": math.sqrt(1.0 - qx * qx),
                    "eskf_q_x": qx,
                    "eskf_q_y": 0.0,
                    "eskf_q_z": 0.0,
                })


class MagneticSourceAbTests(unittest.TestCase):
    def test_reports_physical_residual_and_paired_effect(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            replay = root / "replay.csv"
            on = root / "on.csv"
            off = root / "off.csv"
            write_fixture(replay, on, off)
            report = diagnostic.analyze_magnetic_source_ab(replay, on, off)
            diagnostic.write_report(report, root / "report")
            serialized = json.loads((root / "report" / "summary.json").read_text())

        self.assertEqual(report["status"], "completed_offline_truth_scored_diagnostic")
        self.assertEqual(report["physical_magnetic_source"]["updates"], 4)
        self.assertAlmostEqual(
            report["physical_magnetic_source"]["heading_minus_declared_datum_deg"]["maximum"],
            20.0,
            delta=0.01,
        )
        self.assertEqual(report["predeclared_residual_bins"]["under_5_deg"]["samples"], 2)
        self.assertEqual(report["predeclared_residual_bins"]["5_to_15_deg"]["samples"], 1)
        self.assertEqual(report["predeclared_residual_bins"]["at_least_15_deg"]["samples"], 1)
        self.assertGreater(
            report["paired_estimator_effect"]["eskf_on_yaw_error_deg"]["rmse"],
            report["paired_estimator_effect"]["eskf_off_yaw_error_deg"]["rmse"],
        )
        self.assertNotIn("NaN", json.dumps(serialized))

    def test_rejects_mismatched_off_update_flags(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            replay = root / "replay.csv"
            on = root / "on.csv"
            off = root / "off.csv"
            write_fixture(replay, on, off)
            with off.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            rows[0]["input_mag_update"] = "1"
            with off.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=RESULT_HEADER)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "magnetometer-off results"):
                diagnostic.analyze_magnetic_source_ab(replay, on, off)

    def test_rejects_off_arm_acceptance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            replay = root / "replay.csv"
            on = root / "on.csv"
            off = root / "off.csv"
            write_fixture(replay, on, off)
            with off.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            rows[0]["eskf_mag_accepted"] = "1"
            with off.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=RESULT_HEADER)
                writer.writeheader()
                writer.writerows(rows)
            with self.assertRaisesRegex(ValueError, "magnetometer-off results accept"):
                diagnostic.analyze_magnetic_source_ab(replay, on, off)


if __name__ == "__main__":
    unittest.main()
