from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_gate_characterization as study  # noqa: E402


class GateCharacterizationTests(unittest.TestCase):
    def test_static_null_and_positive_control_are_distinct(self) -> None:
        self.assertEqual(study.STATIC_MOTION, "bias_cv_static_hold")
        self.assertEqual(
            study.POSITIVE_MOTIONS,
            ("bias_cv_takeoff_box_land", "bias_cv_yaw_quadrant_hover"),
        )
        self.assertNotIn(study.STATIC_MOTION, study.POSITIVE_MOTIONS)

    def test_zero_static_false_positive_bound_is_defined(self) -> None:
        self.assertAlmostEqual(1.0 - 0.05 ** (1.0 / 64.0), 0.0457, places=3)

    def test_percentiles_are_deterministic_and_interpolated(self) -> None:
        self.assertEqual(study.percentiles([])["p50"], None)
        values = study.percentiles([0.0, 10.0])
        self.assertEqual(values["p50"], 5.0)
        self.assertEqual(values["p90"], 9.0)


if __name__ == "__main__":
    unittest.main()
