from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_multipose_static_calibration_campaign as campaign  # noqa: E402


class MultiPoseStaticCalibrationCampaignTests(unittest.TestCase):
    def test_seed_parser_is_unique_and_bounded(self) -> None:
        self.assertEqual(campaign.parse_seed_list("1, 3, 5"), [1, 3, 5])
        with self.assertRaisesRegex(ValueError, "unique"):
            campaign.parse_seed_list("1,1")
        with self.assertRaisesRegex(ValueError, "unsigned"):
            campaign.parse_seed_list("-1")

    def test_pose_means_have_six_physical_directions_and_no_estimator_truth_field(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "poses.csv"
            metadata = campaign.write_synthetic_pose_means(
                path,
                acceleration_bias_m_s2=[0.12, -0.08, 0.05],
                gyroscope_bias_deg_s=[0.3, -0.2, 0.1],
                raw_acceleration_noise_m_s2=0.02,
                raw_gyroscope_noise_deg_s=0.05,
                samples_per_pose=400,
                seed=17,
            )
            with path.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            self.assertEqual(set(rows[0]), {
                "acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2",
                "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "sample_count",
            })
            self.assertEqual(metadata["pose_count"], 6)
            self.assertEqual(metadata["samples_per_pose"], 400)
            self.assertTrue(metadata["pose_means_sha256"])

    def test_summary_counts_improvement_and_regression_separately(self) -> None:
        records = [
            {
                "id": "zero", "family": "nominal", "status": "completed",
                "calibration_error_norms": {"accelerometer_m_s2": 0.001},
                "outcome": {
                    "baseline_passed": True, "multipose_passed": True,
                    "improved_to_pass": False, "regressed_from_pass": False,
                },
            },
            {
                "id": "improved", "family": "axis", "status": "completed",
                "calibration_error_norms": {"accelerometer_m_s2": 0.003},
                "outcome": {
                    "baseline_passed": False, "multipose_passed": True,
                    "improved_to_pass": True, "regressed_from_pass": False,
                },
            },
            {
                "id": "regressed", "family": "axis", "status": "completed",
                "calibration_error_norms": {"accelerometer_m_s2": 0.002},
                "outcome": {
                    "baseline_passed": True, "multipose_passed": False,
                    "improved_to_pass": False, "regressed_from_pass": True,
                },
            },
            {"id": "failure", "family": "axis", "status": "execution_error"},
        ]
        summary = campaign.summarize(records)
        self.assertEqual(summary["total"], 4)
        self.assertEqual(summary["completed"], 3)
        self.assertEqual(summary["baseline_passed"], 2)
        self.assertEqual(summary["multipose_passed"], 2)
        self.assertEqual(summary["improved_to_pass"], 1)
        self.assertEqual(summary["regressed_from_pass"], 1)
        self.assertAlmostEqual(
            summary["calibration_accelerometer_error_norm_m_s2"]["maximum"], 0.003
        )

    def test_common_calibration_noise_seed_does_not_depend_on_case(self) -> None:
        self.assertEqual(campaign.calibration_noise_seed(41001), campaign.calibration_noise_seed(41001))
        self.assertNotEqual(campaign.calibration_noise_seed(41001), campaign.calibration_noise_seed(41003))
        self.assertTrue(np.array_equal(campaign.POSE_DIRECTIONS[0], [1.0, 0.0, 0.0]))

    def test_campaign_provenance_captures_dirty_source_inputs(self) -> None:
        manifest = campaign.source_manifest(ROOT)
        self.assertIn("src/static_imu_calibration.c", manifest)
        self.assertIn("validation/static_imu_calibration_cli.c", manifest)
        self.assertEqual(len(manifest["src/static_imu_calibration.c"]), 64)


if __name__ == "__main__":
    unittest.main()
