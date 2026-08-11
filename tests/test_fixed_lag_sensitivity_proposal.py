from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import fixed_lag_sensitivity_proposal as candidate  # noqa: E402


class FixedLagSensitivityProposalTests(unittest.TestCase):
    def test_error_state_maps_only_the_declared_five_dimensions(self) -> None:
        correction = np.asarray([0.01, -0.02, 0.1, -0.2, 0.3])
        error = candidate._error_state_from_correction(correction)
        self.assertEqual(error.shape, (15,))
        np.testing.assert_allclose(error[:2], correction[:2])
        self.assertEqual(error[2], 0.0)
        np.testing.assert_allclose(error[9:12], correction[2:])
        np.testing.assert_allclose(error[3:9], 0.0)
        np.testing.assert_allclose(error[12:15], 0.0)

    def test_innovation_residual_sign_recovers_known_correction(self) -> None:
        physical_truth = np.asarray([0.012, -0.009, 0.04, -0.03, 0.02])
        design = np.eye(5)
        residual = -design @ physical_truth
        variance = np.ones(5)
        prior_information = np.zeros((5, 5))
        correction, eigenvalues, posterior_residual = candidate._solve_normalized(
            design, residual, variance, prior_information
        )
        np.testing.assert_allclose(correction, physical_truth, atol=1.0e-12)
        self.assertGreater(float(np.min(eigenvalues)), 0.0)
        np.testing.assert_allclose(posterior_residual, 0.0, atol=1.0e-12)

    def test_bad_innovation_variance_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "variance"):
            candidate._solve_normalized(
                np.ones((2, 5)), np.ones(2), np.asarray([1.0, 0.0]), np.eye(5)
            )

    def test_empty_validation_window_is_not_called_an_improvement(self) -> None:
        self.assertEqual(
            candidate._weighted_innovation_norm(
                candidate.ReplayTrace(({},), ({},), np.asarray([0.0])),
                ((False, False),), 0, 0, 0,
            ),
            float("inf"),
        )

    def test_schedule_requires_transaction_success_and_each_accepted_update(self) -> None:
        accepted = {
            "input_gps_status": "0",
            "input_position_update": "1",
            "input_velocity_update": "1",
            "eskf_position_accepted": "1",
            "eskf_velocity_accepted": "1",
        }
        self.assertEqual(candidate._accepted_schedule(accepted), (True, True))
        rejected = accepted | {"eskf_velocity_accepted": "0"}
        self.assertEqual(candidate._accepted_schedule(rejected), (True, False))
        stale = accepted | {"input_gps_status": "-6"}
        self.assertEqual(candidate._accepted_schedule(stale), (False, False))


if __name__ == "__main__":
    unittest.main()
