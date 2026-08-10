from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import fixed_lag_bias_proposal as proposal  # noqa: E402


class FixedLagBiasProposalTests(unittest.TestCase):
    def test_whitened_residual_projection_recovers_target_without_nuisance(self) -> None:
        design = np.zeros((8, 15), dtype=np.float64)
        for index in range(5):
            design[index, proposal.TARGET_INDICES[index]] = 1.0 / proposal.STATE_SCALES[
                proposal.TARGET_INDICES[index]
            ]
        # Extra rows exercise nuisance projection without changing the target solution.
        design[5, proposal.TARGET_INDICES[0]] = 1.0 / proposal.STATE_SCALES[proposal.TARGET_INDICES[0]]
        design[5, 3] = 2.0 / proposal.STATE_SCALES[3]
        design[6, proposal.TARGET_INDICES[1]] = 1.0 / proposal.STATE_SCALES[proposal.TARGET_INDICES[1]]
        design[6, 4] = -1.0 / proposal.STATE_SCALES[4]
        design[7, proposal.TARGET_INDICES[2]] = 1.0 / proposal.STATE_SCALES[proposal.TARGET_INDICES[2]]
        design[7, 5] = 0.5 / proposal.STATE_SCALES[5]
        normalized_target = np.asarray([0.2, -0.1, 0.3, -0.25, 0.15])
        normalized_nuisance = np.zeros(10)
        residual = (design * proposal.STATE_SCALES[np.newaxis, :])[:, proposal.TARGET_INDICES] @ normalized_target
        target, projected_residual, rank = proposal._project_target_and_residual(design, residual, 1.0e-10)
        self.assertGreaterEqual(rank, 0)
        solution, *_ = np.linalg.lstsq(target, projected_residual, rcond=None)
        np.testing.assert_allclose(solution, normalized_target, atol=1.0e-12)

    def test_invalid_prior_scale_is_rejected_before_stream_access(self) -> None:
        with self.assertRaisesRegex(ValueError, "prior_information_scale"):
            proposal.solve_window(None, 0, 1, prior_information_scale=0.0, score_threshold=1.0)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
