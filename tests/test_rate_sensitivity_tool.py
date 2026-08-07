from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_rate_sensitivity as study  # noqa: E402


class RateSensitivityToolTests(unittest.TestCase):
    def test_noise_profiles_have_explicit_semantics(self) -> None:
        protocol = {"noise": {
            "accel_density_m_s2_sqrt_hz": 0.002,
            "gyro_density_rad_s_sqrt_hz": 0.0001,
        }}
        continuous = study.profile_settings("continuous_density", 100.0, protocol)
        self.assertEqual(continuous["accel_density"], 0.002)
        self.assertEqual(continuous["gyro_density"], 0.0001)
        self.assertEqual(continuous["mag_noise_ut"], 0.20)
        zero = study.profile_settings("zero_all_measurement_noise", 100.0, protocol)
        self.assertEqual(set(zero.values()), {0.0})
        fixed = study.profile_settings("fixed_sample_imu_noise", 100.0, protocol)
        self.assertAlmostEqual(fixed["accel_density"] * 10.0, 0.01)
        self.assertAlmostEqual(fixed["gyro_density"] * 10.0, 0.0003490658504)
        self.assertEqual(fixed["gps_position_noise_m"], 0.50)

    def test_summary_distinguishes_structural_readiness_from_rank(self) -> None:
        report = {
            "evaluation_count": 2,
            "first_structural_information_ready_time_s_analyzer_only": 1.0,
            "evaluations": [
                {
                    "causal_window": {"stop_time_s": 1.0},
                    "effective_rank": 5,
                    "structural_information_ready_analyzer_only": True,
                    "minimum_eigenvalue": 0.2,
                    "condition_number": 10.0,
                    "structural_readiness_checks_analyzer_only": {
                        "full_target_rank": True,
                        "condition_number": True,
                    },
                    "aiding_coverage": {"accepted_unique_timestamps": 5},
                },
                {
                    "causal_window": {"stop_time_s": 2.0},
                    "effective_rank": 3,
                    "structural_information_ready_analyzer_only": False,
                    "minimum_eigenvalue": 0.01,
                    "condition_number": None,
                    "structural_readiness_checks_analyzer_only": {
                        "full_target_rank": False,
                        "condition_number": False,
                    },
                    "aiding_coverage": {"accepted_unique_timestamps": 5},
                },
            ],
        }
        summary = study.summarize_analyzer(report, static_prefix_end_s=1.5)
        self.assertEqual(summary["effective_full_rank_window_count"], 1)
        self.assertEqual(summary["structural_ready_window_count"], 1)
        self.assertEqual(summary["final_effective_rank"], 3)
        self.assertEqual(summary["final_failed_checks"], ["condition_number", "full_target_rank"])
        self.assertEqual(summary["effective_full_rank_before_excitation_count"], 1)
        self.assertEqual(summary["structural_ready_before_excitation_count"], 1)


if __name__ == "__main__":
    unittest.main()
