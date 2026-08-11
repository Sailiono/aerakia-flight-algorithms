from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_fixed_lag_replay_candidate_campaign as campaign  # noqa: E402


class FixedLagReplayCandidateCampaignTests(unittest.TestCase):
    @staticmethod
    def _record(trial: str, split: str, vector: str, delta: float) -> dict[str, object]:
        baseline = {
            "horizontal_bias_terminal_p95_m_s2": 0.1,
            "healthy_ratio": 1.0,
            "navigation_recoveries": 0,
        }
        corrected = {
            "horizontal_bias_terminal_p95_m_s2": 0.1 + delta,
            "healthy_ratio": 1.0,
            "navigation_recoveries": 0,
        }
        return {
            "trial_id": trial,
            "split": split,
            "trajectory_id": "motion",
            "vector_id": vector,
            "baseline_metrics": baseline,
            "candidate_metrics": corrected,
            "candidate": {"schedule_matches_after_replay": True},
            "paired_delta": {
                "horizontal_bias_terminal_p95_m_s2": delta,
                "attitude_rmse_deg": 0.0,
            },
        }

    def test_summary_rejects_material_zero_bias_regression(self) -> None:
        protocol = {"metrics": {
            "material_horizontal_bias_regression_m_s2": 0.005,
            "material_attitude_rmse_regression_deg": 0.05,
            "minimum_mean_horizontal_bias_improvement_m_s2": 0.005,
            "minimum_signed_nonzero_vector_improvement_fraction": 0.75,
        }}
        records = [
            self._record("train-zero", "train", "zero", 0.006),
            self._record("train-x", "train", "x_pos", -0.01),
            self._record("tune-x", "tune", "x_pos", -0.01),
        ]
        summary = campaign.summarize(records, protocol)
        self.assertEqual(summary["zero_bias_material_regressions"], ["train-zero"])
        self.assertFalse(summary["development_acceptance_pass"])


if __name__ == "__main__":
    unittest.main()
