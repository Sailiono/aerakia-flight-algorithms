from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_trajectory_screen as screen  # noqa: E402


class TrajectoryScreenTests(unittest.TestCase):
    def test_default_screen_contains_distinct_frozen_motion_classes(self) -> None:
        self.assertIn("bias_cv_hover_axis_pulses", screen.DEFAULT_MOTIONS)
        self.assertIn("bias_cv_takeoff_box_land", screen.DEFAULT_MOTIONS)
        self.assertIn("bias_cv_yaw_quadrant_hover", screen.DEFAULT_MOTIONS)
        self.assertIn("bias_cv_early_transition_s_curve", screen.DEFAULT_MOTIONS)
        self.assertEqual(len(screen.DEFAULT_MOTIONS), len(set(screen.DEFAULT_MOTIONS)))


if __name__ == "__main__":
    unittest.main()
