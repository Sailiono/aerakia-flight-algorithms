from __future__ import annotations

import copy
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import compare_bias_observability_campaigns as comparison  # noqa: E402


class BiasObservabilityComparisonTests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = {
            "required_splits": ["train", "tune"],
            "expected_trial_count": 4,
            "holdout_forbidden": True,
            "primary_improvement_metric": "terminal_horizontal_error_p95_m_s2",
            "minimum_primary_mean_improvement": 0.01,
            "minimum_nonzero_group_improvement_fraction": 0.5,
            "maximum_nonzero_regressing_groups": 0,
            "mirror_asymmetry_worsening_tolerance": 0.005,
            "required_candidate_metrics": ["tilt_accel_bias_joint_nees_mean"],
            "zero_bias_regression_metrics": ["terminal_horizontal_error_p95_m_s2"],
            "global_regression_metrics": ["position_nis_mean"],
            "mirror_metrics": ["terminal_horizontal_error_p95_m_s2"],
            "mirror_pairs": [],
            "metrics": {
                "settling_time_s": {"direction": "lower", "material_regression_tolerance": 1.0},
                "terminal_horizontal_error_p95_m_s2": {
                    "direction": "lower", "material_regression_tolerance": 0.005
                },
                "position_nis_mean": {
                    "direction": "consistency", "expected_mean": 3.0,
                    "material_regression_tolerance": 0.5,
                },
                "navigation_nees_mean": {
                    "direction": "consistency", "expected_mean": 6.0,
                    "material_regression_tolerance": 1.0,
                },
                "tilt_accel_bias_joint_nees_mean": {
                    "direction": "consistency", "expected_mean": 5.0,
                    "material_regression_tolerance": 1.0,
                },
            },
        }

    def _trial(
        self,
        split: str,
        vector: str,
        seed: int,
        *,
        primary: float,
        nis: float = 3.0,
        joint_nees: float = 5.0,
        censored: bool = False,
        failed: bool = False,
        input_sha: str = "i" * 64,
    ) -> dict[str, object]:
        directory = f"trials/{split}/traj-a/{vector}/seed-{seed:05d}"
        return {
            "split": split,
            "trajectory_id": "traj-a",
            "bias_vector_id": vector,
            "seed": seed,
            "trial_dir": directory,
            "input_sha256": input_sha,
            "acceleration_bias_m_s2": [0.0, 0.0, 0.0],
            "gate_failures": ["synthetic failure"] if failed else [],
            "horizontal_bias": {
                "settling_time_s": 10.0,
                "terminal_horizontal_error_p95_m_s2": primary,
                "right_censored": censored,
            },
            "general": {},
        }

    def _campaign(
        self,
        root: Path,
        *,
        candidate: bool = False,
        primary_delta: float = -0.05,
        nis: float = 3.0,
        joint_nees: float = 5.0,
        extra: dict[str, object] | None = None,
    ) -> tuple[dict[str, object], Path]:
        trials: list[dict[str, object]] = []
        for split, seed in (("train", 1), ("train", 2), ("tune", 3), ("tune", 4)):
            vector = "zero" if seed % 2 else "x_pos"
            baseline_primary = 0.20 if vector == "zero" else 0.40
            trial = self._trial(
                split, vector, seed,
                primary=baseline_primary + (primary_delta if candidate else 0.0),
            )
            trials.append(trial)
            metrics_path = root / trial["trial_dir"] / "metrics.json"
            metrics_path.parent.mkdir(parents=True, exist_ok=True)
            metrics_path.write_text(
                json.dumps({
                    "eskf_consistency": {
                        "position_nis": {"mean": nis},
                        "velocity_nis": {"mean": 3.0},
                        "navigation_nees": {"mean": 6.0},
                    },
                    "bias_estimation": {
                        "tilt_accel_bias_joint_nees": {
                            "mean": joint_nees,
                            "invalid_covariance_samples": 0,
                        }
                    },
                }),
                encoding="utf-8",
            )
        campaign: dict[str, object] = {
            "mode": "train-tune",
            "truth_boundary": "synthetic-truth-only",
            "execution_status": "passed",
            "execution_failures": [],
            "protocol": {"semantic_sha256": "p" * 64},
            "provenance": {
                "git_commit": "a" * 40,
                "git_status_short": [],
                "generator_sha256": "g" * 64,
                "campaign_runner_sha256": "r" * 64,
                "analyzer_sha256": "z" * 64,
            },
            "trials": trials,
        }
        if extra:
            campaign.update(extra)
        path = root / ("candidate.json" if candidate else "baseline.json")
        path.write_text(json.dumps(campaign), encoding="utf-8")
        return campaign, path

    def _compare(self, *, candidate_kwargs: dict[str, object] | None = None):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            baseline, baseline_path = self._campaign(root / "baseline")
            candidate, candidate_path = self._campaign(
                root / "candidate", candidate=True, **(candidate_kwargs or {})
            )
            return comparison.compare_campaigns(
                baseline, candidate, copy.deepcopy(self.policy),
                baseline_path=baseline_path, candidate_path=candidate_path,
            )

    def test_exact_pairing_delta_and_sibling_joint_nees(self) -> None:
        result = self._compare()
        self.assertEqual(result["paired_trial_count"], 4)
        row = result["paired_trials"][0]
        self.assertAlmostEqual(
            row["metrics"]["terminal_horizontal_error_p95_m_s2"]["candidate_minus_baseline"],
            -0.05,
        )
        self.assertEqual(row["metrics"]["tilt_accel_bias_joint_nees_mean"]["baseline"], 5.0)
        self.assertEqual(result["status"], "passed")

    def test_missing_extra_key_and_holdout_are_rejected(self) -> None:
        result = self._compare(candidate_kwargs={"extra": {"trials": []}})
        self.assertTrue(any("trial-key mismatch" in f for f in result["failures"]))
        result = self._compare(candidate_kwargs={"extra": {"trials": [
            self._trial("holdout", "zero", 99, primary=0.1)
        ]}})
        self.assertTrue(any("forbidden holdout" in f for f in result["failures"]))

    def test_provenance_and_input_sha_failures_are_explicit(self) -> None:
        result = self._compare(candidate_kwargs={
            "extra": {"provenance": {
                "git_commit": "dirty",
                "git_status_short": ["M file"],
                "generator_sha256": "different",
                "campaign_runner_sha256": "r" * 64,
                "analyzer_sha256": "z" * 64,
            }},
        })
        self.assertTrue(any("immutable 40-hex" in f for f in result["failures"]))
        self.assertTrue(any("dirty" in f for f in result["failures"]))
        self.assertTrue(any("generator_sha256 mismatch" in f for f in result["failures"]))
        result = self._compare(candidate_kwargs={"extra": {"trials": [
            self._trial("train", "zero", 1, primary=0.1, input_sha="x" * 64),
        ]}})
        self.assertTrue(any("input SHA-256 mismatch" in f for f in result["failures"]))

    def test_zero_bias_regression_and_censoring_transition(self) -> None:
        result = self._compare(candidate_kwargs={
            "primary_delta": 0.10,
        })
        self.assertTrue(any("zero-bias material regression" in f for f in result["failures"]))
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            baseline, baseline_path = self._campaign(root / "baseline")
            candidate, candidate_path = self._campaign(root / "candidate", candidate=True)
            candidate["trials"][0]["horizontal_bias"]["right_censored"] = True
            result = comparison.compare_campaigns(
                baseline, candidate, copy.deepcopy(self.policy),
                baseline_path=baseline_path, candidate_path=candidate_path,
            )
            zero_row = next(
                row for row in result["paired_trials"]
                if row["split"] == "train" and row["bias_vector_id"] == "zero"
            )
            self.assertEqual(zero_row["censoring_transition"], "settled_to_censored")
            self.assertIsNone(
                zero_row["metrics"]["settling_time_s"]["candidate_minus_baseline"]
            )

    def test_consistency_score_is_distance_from_theory(self) -> None:
        result = self._compare(candidate_kwargs={"nis": 3.2, "joint_nees": 4.8})
        row = result["paired_trials"][0]["metrics"]
        self.assertAlmostEqual(row["position_nis_mean"]["regression_score_delta"], 0.2)
        self.assertAlmostEqual(row["tilt_accel_bias_joint_nees_mean"]["regression_score_delta"], 0.2)

    def test_mirror_worsening_and_required_metric_missing(self) -> None:
        policy = copy.deepcopy(self.policy)
        policy["mirror_pairs"] = [{"id": "x", "left": "x_neg", "right": "x_pos"}]
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            baseline, baseline_path = self._campaign(root / "baseline")
            candidate, candidate_path = self._campaign(root / "candidate", candidate=True)
            for campaign, delta in ((baseline, 0.0), (candidate, 0.0)):
                for trial in campaign["trials"]:
                    trial["bias_vector_id"] = "x_neg" if trial["seed"] % 2 else "x_pos"
                    trial["horizontal_bias"]["terminal_horizontal_error_p95_m_s2"] += delta
            # Increase only one side of the candidate mirror.  A common-mode
            # shift would preserve asymmetry and would not exercise the gate.
            candidate_x_pos = next(
                trial for trial in candidate["trials"]
                if trial["bias_vector_id"] == "x_pos"
            )
            candidate_x_pos["horizontal_bias"]["terminal_horizontal_error_p95_m_s2"] += 0.10
            result = comparison.compare_campaigns(
                baseline, candidate, policy,
                baseline_path=baseline_path, candidate_path=candidate_path,
            )
            self.assertTrue(any("mirror asymmetry worsened" in f for f in result["failures"]))

    def test_output_files_include_json_markdown_and_csv(self) -> None:
        result = self._compare()
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            (out / "summary.json").write_text(json.dumps(result), encoding="utf-8")
            comparison.write_paired_csv(result, out / "paired_trials.csv")
            comparison.write_report(result, out / "report.md")
            self.assertIn("candidate_terminal_horizontal_error_p95_m_s2", (out / "paired_trials.csv").read_text())
            self.assertIn("Evidence boundaries", (out / "report.md").read_text())
            with (out / "paired_trials.csv").open(newline="") as handle:
                self.assertEqual(len(list(csv.DictReader(handle))), 4)


if __name__ == "__main__":
    unittest.main()
