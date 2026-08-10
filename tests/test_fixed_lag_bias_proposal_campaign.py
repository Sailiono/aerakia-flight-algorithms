from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_fixed_lag_bias_proposal_campaign as campaign  # noqa: E402


class FixedLagBiasProposalCampaignTests(unittest.TestCase):
    def test_campaign_has_two_non_holdout_candidate_motions(self) -> None:
        self.assertEqual(
            tuple(item[0] for item in campaign.MOTIONS),
            ("takeoff_box_land", "yaw_quadrant_hover"),
        )

    def test_vector_cases_are_nine_signed_groups(self) -> None:
        protocol = {"residual_bias_contract": {
            "accelerometer_sigma_m_s2": 0.05,
            "vectors": [
                {"id": str(index), "accel_sigma_multipliers": [index, 0, 0]}
                for index in range(9)
            ],
        }}
        vectors = campaign.vector_cases(protocol)
        self.assertEqual(len(vectors), 9)
        self.assertEqual(vectors[2]["accel_bias_m_s2"], [0.1, 0.0, 0.0])

    def test_terminal_aggregate_excludes_earlier_windows(self) -> None:
        trial = {
            "motion_id": "takeoff_box_land",
            "input_sha256": "input-a",
            "result_sha256": "result-a",
            "vector_id": "x_pos",
            "seed": 3,
            "proposals": {
                "1.0": [
                    {
                        "stop_time_s": 20.0,
                        "score_pass": True,
                        "baseline_error_norm_m_s2": 1.0,
                        "corrected_error_norm_m_s2": 0.5,
                        "improved": True,
                    },
                    {
                        "stop_time_s": 40.0,
                        "score_pass": True,
                        "baseline_error_norm_m_s2": 2.0,
                        "corrected_error_norm_m_s2": 3.0,
                        "improved": False,
                    },
                ]
            },
        }
        all_windows = campaign.aggregate([trial], (1.0,))["1.0"]
        terminal = campaign.aggregate([trial], (1.0,), terminal_only=True)["1.0"]
        self.assertEqual(all_windows["proposal_observations"], 2)
        self.assertEqual(terminal["proposal_observations"], 1)
        self.assertEqual(terminal["baseline_error_mean_m_s2"], 2.0)
        self.assertEqual(terminal["corrected_error_mean_m_s2"], 3.0)
        self.assertEqual(terminal["improved_observations"], 0)

    def test_trial_manifest_binds_input_and_result_identities(self) -> None:
        trials = [
            {
                "motion_id": "takeoff_box_land",
                "vector_id": "x_pos",
                "seed": 0,
                "input_sha256": "input-a",
                "result_sha256": "result-a",
            }
        ]
        input_manifest = campaign.trial_manifest_sha256(trials, "input_sha256")
        result_manifest = campaign.trial_manifest_sha256(trials, "result_sha256")
        self.assertNotEqual(input_manifest, result_manifest)
        trials[0]["input_sha256"] = "input-b"
        self.assertNotEqual(input_manifest, campaign.trial_manifest_sha256(trials, "input_sha256"))


if __name__ == "__main__":
    unittest.main()
