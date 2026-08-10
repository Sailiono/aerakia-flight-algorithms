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


if __name__ == "__main__":
    unittest.main()
