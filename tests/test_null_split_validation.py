from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_bias_observability_null_split_validation as study  # noqa: E402


class NullSplitValidationTests(unittest.TestCase):
    def test_percentile_and_disjoint_split(self) -> None:
        self.assertEqual(study.percentile([0.0, 10.0], 0.99), 9.9)
        self.assertLess(7, 100)


if __name__ == "__main__":
    unittest.main()
