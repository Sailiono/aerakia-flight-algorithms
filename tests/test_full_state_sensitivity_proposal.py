from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import full_state_sensitivity_proposal as full  # noqa: E402


class FullStateSensitivityProposalTests(unittest.TestCase):
    def test_snapshot_requires_positive_definite_full_covariance(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot.json"
            path.write_text(json.dumps({"P": np.eye(15).tolist()}), encoding="utf-8")
            np.testing.assert_allclose(full.load_snapshot(path), np.eye(15))
            path.write_text(json.dumps({"P": np.zeros((15, 15)).tolist()}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "positive definite"):
                full.load_snapshot(path)

    def test_full_solve_recovers_known_innovation_correction(self) -> None:
        truth = np.linspace(-0.02, 0.02, 15)
        correction, eigenvalues, residual = full._solve_normalized(
            np.eye(15), -truth, np.ones(15), np.zeros((15, 15))
        )
        np.testing.assert_allclose(correction, truth, atol=1.0e-12)
        self.assertGreater(float(np.min(eigenvalues)), 0.0)
        np.testing.assert_allclose(residual, 0.0, atol=1.0e-12)


if __name__ == "__main__":
    unittest.main()
