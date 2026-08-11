"""Focused checks for the v5 opened residual-episode diagnostic."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import run_airspeed_wind_mismatch_monitor_v5 as v5


class AirspeedWindMismatchMonitorV5Tests(unittest.TestCase):
    def make_monitor(self, **changes: object) -> v5.ConsecutiveEvidenceMonitor:
        config: dict[str, object] = {
            "warmup_eligible_observations": 0,
            "maximum_inter_observation_gap_us": 1_000_000,
            "maximum_episode_elapsed_us": 2_000_000,
            "contribution_nis_floor": 4.0,
            "per_observation_nis_cap": 6.0,
            "minimum_consecutive_high_observations": 4,
            "minimum_episode_elapsed_us": 1_500_000,
            "cumulative_nis_threshold": 16.0,
            "trace_observations": 12,
        }
        config.update(changes)
        return v5.ConsecutiveEvidenceMonitor(**config)  # type: ignore[arg-type]

    def test_one_large_residual_with_only_two_preceding_highs_cannot_latch(self) -> None:
        monitor = self.make_monitor()
        for index, nis in enumerate((4.1, 4.2, 500.0)):
            monitor.observe(index * 500_000, nis)
        self.assertFalse(monitor.latched)
        self.assertEqual(len(monitor.episode or ()), 3)

    def test_four_consecutive_high_observations_latch_after_required_span(self) -> None:
        monitor = self.make_monitor()
        for index in range(4):
            monitor.observe(index * 500_000, 5.0)
        self.assertTrue(monitor.latched)
        self.assertEqual(monitor.first_latch_timestamp_us, 1_500_000)
        self.assertEqual(len(monitor.latch_episode or ()), 4)

    def test_low_residual_resets_an_episode(self) -> None:
        monitor = self.make_monitor()
        for index, nis in enumerate((5.0, 5.0, 1.0, 5.0, 5.0, 5.0)):
            monitor.observe(index * 500_000, nis)
        self.assertFalse(monitor.latched)
        self.assertEqual(monitor.reset_reasons["low_nis"], 1)
        self.assertEqual(len(monitor.episode or ()), 3)

    def test_gap_resets_an_episode_before_a_new_outlier(self) -> None:
        monitor = self.make_monitor()
        monitor.observe(0, 5.0)
        monitor.observe(500_000, 5.0)
        monitor.observe(2_000_000, 500.0)
        self.assertFalse(monitor.latched)
        self.assertEqual(monitor.reset_reasons["inter_observation_gap"], 1)
        self.assertEqual(len(monitor.episode or ()), 1)

    def test_protocol_rejects_release_status_and_unknown_injection(self) -> None:
        protocol = v5.load_protocol()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "protocol.json"
            broken = copy.deepcopy(protocol)
            broken["status"] = "sealed_holdout"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                v5.load_protocol(path)
            broken = copy.deepcopy(protocol)
            broken["case_matrix"][0]["injection"] = {"kind": "unknown"}
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                v5.load_protocol(path)

    def test_same_seed_standard_cases_have_identical_common_prefix(self) -> None:
        protocol = v5.load_protocol()
        base_protocol = v5.resolve_base_protocol(protocol)
        standard = [case for case in protocol["case_matrix"] if case["paired_prefix_group"] == "standard_noise"]
        fingerprints = {
            v5.build_case_stream(case, protocol, base_protocol, synthetic_seed=52001)[4]
            for case in standard
        }
        self.assertEqual(len(fingerprints), 1)

    def test_parallel_records_match_sequential_for_a_small_fixed_matrix(self) -> None:
        protocol = copy.deepcopy(v5.load_protocol())
        protocol["seed_sets"]["development"] = [52001]
        protocol["case_matrix"] = protocol["case_matrix"][:2]
        base_protocol = v5.resolve_base_protocol(protocol)
        sequential = v5.run_records(protocol, base_protocol, jobs=1)
        parallel = v5.run_records(protocol, base_protocol, jobs=2)
        self.assertEqual(
            v5.base.canonical_sha256(sequential),
            v5.base.canonical_sha256(parallel),
        )
        with self.assertRaises(ValueError):
            v5.run_records(protocol, base_protocol, jobs=0)


if __name__ == "__main__":
    unittest.main()
