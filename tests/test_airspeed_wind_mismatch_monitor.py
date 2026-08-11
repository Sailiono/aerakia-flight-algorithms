"""Tests for the v2 validation-only TAS residual-pattern monitor."""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
import run_airspeed_wind_mismatch_monitor as candidate  # noqa: E402


class AirspeedWindMismatchMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.protocol = candidate.load_protocol()
        cls.base = candidate.resolve_base_protocol(cls.protocol)

    def monitor(self) -> candidate.CumulativeNisMonitor:
        return candidate.monitor_from_protocol(self.protocol)

    def test_one_large_impulse_cannot_latch_without_a_second_contribution(self) -> None:
        monitor = self.monitor()
        for timestamp_us, nis in ((0, 0.2), (500_000, 0.3), (1_000_000, 0.4), (1_500_000, 100.0)):
            self.assertFalse(monitor.observe(timestamp_us, nis))
        self.assertEqual(monitor.completed_windows, 1)
        self.assertFalse(monitor.latched)
        self.assertEqual(monitor.maximum_window_contributing_observations, 1)

    def test_multiple_bounded_residuals_latch_within_the_short_window(self) -> None:
        monitor = self.monitor()
        for timestamp_us, nis in ((0, 4.0), (500_000, 5.0), (1_000_000, 5.0)):
            self.assertFalse(monitor.observe(timestamp_us, nis))
        self.assertTrue(monitor.observe(1_500_000, 5.0))
        self.assertTrue(monitor.latched)
        self.assertEqual(monitor.first_latch_timestamp_us, 1_500_000)
        self.assertEqual(len(monitor.latch_window or ()), 4)

    def test_gap_resets_incomplete_history_before_a_later_impulse(self) -> None:
        monitor = self.monitor()
        self.assertFalse(monitor.observe(0, 6.0))
        self.assertFalse(monitor.observe(500_000, 6.0))
        self.assertFalse(monitor.observe(2_000_000, 100.0))
        self.assertEqual(monitor.reset_reasons["inter_observation_gap"], 1)
        self.assertEqual(monitor.completed_windows, 0)
        self.assertFalse(monitor.latched)

    def test_long_nominal_has_real_postqualification_window_coverage(self) -> None:
        case = next(item for item in self.protocol["case_matrix"] if item["name"] == "long_nominal")
        record = candidate.evaluate_candidate_case(
            case, self.protocol, self.base, synthetic_seed=32001
        )
        self.assertTrue(record["passed"], record["errors"])
        self.assertGreaterEqual(
            record["pre_injection_eligible_observations"],
            self.protocol["coverage"]["minimum_pre_injection_eligible_observations"],
        )
        self.assertGreaterEqual(
            record["pre_injection_completed_windows"],
            self.protocol["coverage"]["minimum_pre_injection_completed_windows"],
        )
        self.assertFalse(record["monitor_latched"])

    def test_persistent_vertical_mismatch_needs_multiple_injected_samples(self) -> None:
        case = next(item for item in self.protocol["case_matrix"] if item["name"] == "persistent_vertical_wind")
        record = candidate.evaluate_candidate_case(
            case, self.protocol, self.base, synthetic_seed=32001
        )
        self.assertTrue(record["passed"], record["errors"])
        self.assertTrue(record["monitor_latched"])
        self.assertGreaterEqual(
            record["monitor_latch_window_injection_observations"],
            case["expected"]["minimum_latch_window_injection_observations"],
        )
        self.assertGreaterEqual(record["monitor_latch_delay_s"], 0.0)

    def test_protocol_rejects_duplicate_seed_and_production_scope_drift(self) -> None:
        for mutate, pattern in (
            (lambda value: value["seed_sets"]["development"].__setitem__(1, 32001), "must be unique"),
            (lambda value: value["scope"].__setitem__("production_eskf", "17_error_state"), "leave production ESKF unchanged"),
        ):
            bad = copy.deepcopy(self.protocol)
            mutate(bad)
            with tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / "protocol.json"
                path.write_text(json.dumps(bad), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, pattern):
                    candidate.load_protocol(path)

    def test_v2_is_development_only_and_cannot_run_as_a_sealed_campaign(self) -> None:
        self.assertEqual(self.protocol["status"], "development_only_validation")
        self.assertEqual(set(self.protocol["seed_sets"]), {"development"})
        with self.assertRaisesRegex(ValueError, "has no 'sealed_holdout' seed set"):
            candidate.run_protocol(self.protocol, self.base, phase="sealed_holdout", jobs=1)

    def test_single_job_protocol_execution_does_not_need_a_worker_process(self) -> None:
        minimal = copy.deepcopy(self.protocol)
        minimal["seed_sets"]["development"] = [32001]
        minimal["case_matrix"] = [
            next(item for item in self.protocol["case_matrix"] if item["name"] == "long_nominal")
        ]
        result = candidate.run_protocol(minimal, self.base, phase="development", jobs=1)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["totals"]["replication_cases"], 1)

    def test_seed_selector_uses_only_canonical_contiguous_protocol_members(self) -> None:
        self.assertEqual(
            candidate.select_phase_seeds(self.protocol, "development", start_index=2, count=3),
            [32003, 32004, 32005],
        )
        with self.assertRaisesRegex(ValueError, "empty or starts beyond"):
            candidate.select_phase_seeds(self.protocol, "development", start_index=99, count=1)

    def test_strict_shard_merge_requires_complete_raw_record_coverage(self) -> None:
        minimal = copy.deepcopy(self.protocol)
        minimal["seed_sets"]["development"] = [32001, 32002]
        minimal["case_matrix"] = [
            next(item for item in self.protocol["case_matrix"] if item["name"] == "long_nominal")
        ]
        shards = []
        for seed in minimal["seed_sets"]["development"]:
            result = candidate.run_protocol(
                minimal,
                self.base,
                phase="development",
                jobs=1,
                selected_seeds=[seed],
                include_records=True,
            )
            result["protocol"] = {"semantic_sha256": candidate.base.canonical_sha256(minimal)}
            result["provenance"] = {
                "runner_sha256": candidate.file_sha256(candidate.RUNNER_PATH),
                "git_status": "",
                "git_commit": "test-commit",
            }
            shards.append(result)
        with tempfile.TemporaryDirectory() as temporary:
            paths = []
            for index, shard in enumerate(shards):
                path = Path(temporary) / f"shard-{index}.json"
                path.write_text(json.dumps(shard), encoding="utf-8")
                paths.append(path)
            merged = candidate.merge_shard_results(
                minimal,
                self.base,
                phase="development",
                shard_paths=paths,
            )
        self.assertEqual(merged["status"], "passed")
        self.assertEqual(merged["seed_count"], 2)
        self.assertEqual(merged["totals"]["replication_cases"], 2)
        self.assertNotIn("records", merged)
        self.assertEqual(merged["campaign_assembly"]["shard_count"], 2)

    def test_sealed_v4_exposes_only_the_unopened_holdout_set(self) -> None:
        sealed_path = ROOT / "validation" / "airspeed_wind_mismatch_monitor_protocol_v4.json"
        sealed = candidate.load_protocol(sealed_path)
        candidate.resolve_base_protocol(sealed)
        self.assertEqual(sealed["status"], "sealed_holdout_validation_only")
        self.assertEqual(set(sealed["seed_sets"]), {"sealed_holdout"})
        self.assertEqual(len(sealed["seed_sets"]["sealed_holdout"]), 128)
        self.assertEqual(sealed["seed_sets"]["sealed_holdout"][0], 43001)
        self.assertEqual(sealed["seed_sets"]["sealed_holdout"][-1], 43128)
        self.assertEqual(
            candidate.file_sha256(candidate.RUNNER_PATH),
            sealed["candidate_runner_file_sha256"],
        )
        with self.assertRaisesRegex(ValueError, "has no 'development' seed set"):
            candidate.run_protocol(sealed, self.base, phase="development", jobs=1)

    def test_default_artifacts_are_versioned_by_frozen_protocol_id(self) -> None:
        sealed_path = ROOT / "validation" / "airspeed_wind_mismatch_monitor_protocol_v4.json"
        sealed = candidate.load_protocol(sealed_path)
        self.assertEqual(
            candidate.default_output_path(self.protocol, "development"),
            ROOT / "build" / "airspeed-wind-mismatch-monitor-v2-development.json",
        )
        self.assertEqual(
            candidate.default_output_path(self.protocol, "sealed_holdout"),
            ROOT / "validation" / "public" / "airspeed_wind_mismatch_monitor_v2.json",
        )
        self.assertEqual(
            candidate.default_output_path(sealed, "sealed_holdout"),
            ROOT / "validation" / "public" / "airspeed_wind_mismatch_monitor_v4.json",
        )

    def test_default_artifact_rejects_an_unsafe_protocol_id(self) -> None:
        unsafe = copy.deepcopy(self.protocol)
        unsafe["protocol_id"] = "../../overwrite"
        with self.assertRaisesRegex(ValueError, "safe versioned output name"):
            candidate.default_output_path(unsafe, "development")


if __name__ == "__main__":
    unittest.main()
