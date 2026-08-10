from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_excitation_score_campaign as study  # noqa: E402


class ExcitationScoreCampaignTests(unittest.TestCase):
    def test_motions_are_frozen_and_distinct(self) -> None:
        self.assertEqual(
            study.MOTIONS,
            ("bias_cv_takeoff_box_land", "bias_cv_yaw_quadrant_hover"),
        )
        self.assertNotEqual(*study.MOTIONS)


if __name__ == "__main__":
    unittest.main()
