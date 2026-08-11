"""Focused contract checks for the standalone v6 persistence monitor."""

from __future__ import annotations

import math
import copy
import json
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

import airspeed_wind_residual_persistence_v6 as v6
import run_airspeed_wind_residual_persistence_v6 as campaign


class ResidualPersistenceV6Tests(unittest.TestCase):
    def make_monitor(self, **changes: object) -> v6.SourceTimeResidualPersistenceMonitor:
        values: dict[str, object] = {
            "warmup_min_valid_observations": 3,
            "warmup_min_source_span_s": 0.5,
            "quiet_nis_threshold": 2.0,
            "high_nis_threshold": 4.0,
            "high_episode_min_observations": 4,
            "high_episode_min_source_span_s": 1.5,
            "maximum_source_gap_s": 1.25,
            "maximum_arrival_gap_s": 1.25,
            "pre_trigger_trace_capacity": 16,
            "post_latch_tail_capacity": 5,
        }
        values.update(changes)
        return v6.SourceTimeResidualPersistenceMonitor(
            v6.ResidualPersistenceConfig(**values)  # type: ignore[arg-type]
        )

    @staticmethod
    def observe(
        monitor: v6.SourceTimeResidualPersistenceMonitor,
        source_us: int,
        nis: float | None,
        *,
        arrival_us: int | None = None,
        epoch: int = 1,
        valid: bool = True,
    ) -> v6.MonitorDecision:
        return monitor.observe(
            source_epoch=epoch,
            source_timestamp_us=source_us,
            arrival_timestamp_us=source_us if arrival_us is None else arrival_us,
            nis=nis,
            source_valid=valid,
        )

    def qualify_at_rate(
        self,
        monitor: v6.SourceTimeResidualPersistenceMonitor,
        *,
        rate_hz: float,
        epoch: int = 1,
        arrival_delay_us: int = 0,
    ) -> int:
        interval_us = int(round(1.0e6 / rate_hz))
        source_us = 0
        for _ in range(1000):
            self.observe(
                monitor,
                source_us,
                1.0,
                arrival_us=source_us + arrival_delay_us,
                epoch=epoch,
            )
            if monitor.state is v6.MonitorState.QUIET_CONFIRMED:
                return source_us + interval_us
            source_us += interval_us
        self.fail("monitor did not obtain quiet qualification")

    def test_explicit_state_path_and_reauthorization(self) -> None:
        monitor = self.make_monitor()
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        rejected = self.observe(monitor, 0, 1.0)
        self.assertFalse(rejected.accepted_residual)
        self.assertEqual(rejected.reason, "source_epoch_not_authorized")

        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        self.assertEqual(monitor.state, v6.MonitorState.QUIET_CONFIRMED)

        for index in range(4):
            decision = self.observe(monitor, next_source + index * 500_000, 5.0)
        self.assertEqual(decision.reason, "high_episode_latched")
        self.assertEqual(monitor.state, v6.MonitorState.LATCHED)
        self.assertIsNotNone(monitor.trigger_snapshot)

        retained = monitor.trigger_snapshot
        monitor.reauthorize(2)
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        self.assertEqual(monitor.authorized_source_epoch, 2)
        self.assertIs(monitor.trigger_snapshot, retained)

    def test_high_episode_uses_source_time_across_supported_rate_matrix(self) -> None:
        for rate_hz in (1.0, 1.5, 2.0, 5.0, 10.0, 20.0):
            with self.subTest(rate_hz=rate_hz):
                monitor = self.make_monitor(
                    maximum_source_gap_s=1.1,
                    maximum_arrival_gap_s=1.1,
                )
                monitor.reauthorize(1)
                interval_us = int(round(1.0e6 / rate_hz))
                next_source = self.qualify_at_rate(monitor, rate_hz=rate_hz)
                for index in range(1000):
                    source_us = next_source + index * interval_us
                    self.observe(monitor, source_us, 5.0)
                    if monitor.state is v6.MonitorState.LATCHED:
                        break
                self.assertEqual(monitor.state, v6.MonitorState.LATCHED)
                snapshot = monitor.trigger_snapshot
                self.assertIsNotNone(snapshot)
                assert snapshot is not None
                self.assertGreaterEqual(snapshot.episode_source_span_s, 1.5)
                self.assertGreaterEqual(len(snapshot.episode_observations), 4)
                self.assertEqual(
                    snapshot.latch_source_timestamp_us
                    - snapshot.episode_observations[0].source_timestamp_us,
                    int(round(snapshot.episode_source_span_s * 1.0e6)),
                )

    def test_source_and_arrival_gap_counters_are_independent(self) -> None:
        monitor = self.make_monitor(maximum_source_gap_s=1.0, maximum_arrival_gap_s=1.0)
        monitor.reauthorize(1)
        self.observe(monitor, 0, 1.0, arrival_us=5_000_000)
        source_only = self.observe(monitor, 2_000_000, 1.0, arrival_us=5_500_000)
        self.assertTrue(source_only.source_gap_observed)
        self.assertFalse(source_only.arrival_gap_observed)
        self.assertEqual(monitor.counters.source_gap_events, 1)
        self.assertEqual(monitor.counters.arrival_gap_events, 0)
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 1)

        monitor.reauthorize(2)
        self.observe(monitor, 0, 1.0, epoch=2)
        arrival_only = self.observe(monitor, 500_000, 1.0, arrival_us=2_000_000, epoch=2)
        self.assertFalse(arrival_only.source_gap_observed)
        self.assertTrue(arrival_only.arrival_gap_observed)
        self.assertEqual(monitor.counters.source_gap_events, 1)
        self.assertEqual(monitor.counters.arrival_gap_events, 1)
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 2)

    def test_gap_telemetry_distinguishes_empty_and_nonempty_episode(self) -> None:
        monitor = self.make_monitor(maximum_source_gap_s=1.0, maximum_arrival_gap_s=1.0)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        self.observe(monitor, next_source, 5.0)
        self.assertEqual(monitor.state, v6.MonitorState.HIGH_EPISODE)
        gap = self.observe(monitor, next_source + 2_000_000, 5.0)
        self.assertTrue(gap.source_gap_observed)
        self.assertTrue(gap.arrival_gap_observed)
        self.assertEqual(monitor.counters.gap_resets_nonempty_episode, 1)
        self.assertEqual(monitor.counters.gap_resets_empty_episode, 0)
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        self.assertEqual(len(monitor.high_episode_observations), 0)

    def test_monotonic_arrival_jitter_does_not_change_source_time_span(self) -> None:
        monitor = self.make_monitor(maximum_arrival_gap_s=2.0)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0, arrival_delay_us=200_000)
        delays = (180_000, 260_000, 210_000, 300_000)
        for index, delay_us in enumerate(delays):
            self.observe(
                monitor,
                next_source + index * 500_000,
                5.0,
                arrival_us=next_source + index * 500_000 + delay_us,
            )
        snapshot = monitor.trigger_snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertAlmostEqual(snapshot.episode_source_span_s, 1.5)
        self.assertEqual(
            snapshot.latch_arrival_timestamp_us - snapshot.latch_source_timestamp_us,
            delays[-1],
        )

    def test_nonmonotonic_source_or_arrival_never_contributes_and_revokes(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        self.observe(monitor, 0, 1.0, arrival_us=1_000_000)
        arrival_bad = self.observe(monitor, 500_000, 5.0, arrival_us=900_000)
        self.assertFalse(arrival_bad.accepted_residual)
        self.assertEqual(monitor.counters.nonmonotonic_arrival_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)
        self.assertEqual(len(monitor.high_episode_observations), 0)

        monitor.reauthorize(2)
        self.observe(monitor, 1_000_000, 1.0, arrival_us=1_100_000, epoch=2)
        source_bad = self.observe(monitor, 900_000, 5.0, arrival_us=1_200_000, epoch=2)
        self.assertFalse(source_bad.accepted_residual)
        self.assertEqual(monitor.counters.nonmonotonic_source_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)

    def test_invalid_no_nis_and_epoch_change_fail_closed(self) -> None:
        cases = (
            {"nis": None, "valid": True, "counter": "missing_nis_events"},
            {"nis": 1.0, "valid": False, "counter": "invalid_source_events"},
            {"nis": math.nan, "valid": True, "counter": "invalid_nis_events"},
        )
        for case in cases:
            with self.subTest(counter=case["counter"]):
                monitor = self.make_monitor()
                monitor.reauthorize(1)
                result = self.observe(
                    monitor,
                    0,
                    case["nis"],  # type: ignore[arg-type]
                    valid=bool(case["valid"]),
                )
                self.assertFalse(result.accepted_residual)
                self.assertIsNone(monitor.authorized_source_epoch)
                self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
                self.assertEqual(getattr(monitor.counters, str(case["counter"])), 1)

        monitor = self.make_monitor()
        monitor.reauthorize(1)
        changed = self.observe(monitor, 0, 1.0, epoch=2)
        self.assertFalse(changed.accepted_residual)
        self.assertEqual(monitor.counters.source_epoch_mismatch_events, 1)
        self.assertEqual(monitor.counters.source_epoch_reset_events, 1)
        self.assertIsNone(monitor.authorized_source_epoch)

        monitor.reauthorize(3)
        self.qualify_at_rate(monitor, rate_hz=2.0, epoch=3)
        monitor.source_epoch_reset()
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        self.assertIsNone(monitor.authorized_source_epoch)
        self.assertEqual(monitor.counters.source_epoch_reset_events, 2)
        after_reset = self.observe(monitor, 10_000_000, 1.0, epoch=3)
        self.assertEqual(after_reset.reason, "source_epoch_not_authorized")

    def test_insufficient_warmup_coverage_cannot_confirm_quiet(self) -> None:
        monitor = self.make_monitor(
            warmup_min_valid_observations=5,
            warmup_min_source_span_s=2.0,
        )
        monitor.reauthorize(1)
        for index in range(4):
            self.observe(monitor, index * 500_000, 1.0)
        self.assertFalse(monitor.warmup_ready)
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        self.assertEqual(monitor.counters.warmup_completions, 0)
        self.assertEqual(monitor.counters.quiet_confirmations, 0)

    def test_no_quiet_coverage_cannot_start_or_latch_high_episode(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        for index in range(20):
            self.observe(monitor, index * 500_000, 5.0)
        self.assertEqual(monitor.state, v6.MonitorState.UNQUALIFIED)
        self.assertTrue(monitor.warmup_ready)
        self.assertEqual(monitor.counters.warmup_completions, 1)
        self.assertEqual(monitor.counters.quiet_confirmations, 0)
        self.assertEqual(monitor.counters.high_episode_starts, 0)
        self.assertEqual(monitor.counters.latches, 0)

    def test_subthreshold_residual_preserves_quiet_and_aborts_high_episode(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        middle = self.observe(monitor, next_source, 3.0)
        self.assertEqual(middle.state, v6.MonitorState.QUIET_CONFIRMED)
        self.assertEqual(middle.reason, "quiet_confirmed_residual")

        self.observe(monitor, next_source + 500_000, 5.0)
        self.assertEqual(monitor.state, v6.MonitorState.HIGH_EPISODE)
        aborted = self.observe(monitor, next_source + 1_000_000, 3.0)
        self.assertEqual(aborted.state, v6.MonitorState.QUIET_CONFIRMED)
        self.assertEqual(aborted.reason, "high_episode_aborted_by_low_residual")
        self.assertEqual(len(monitor.high_episode_observations), 0)

    def test_trigger_snapshot_is_immutable_and_post_tail_is_bounded(self) -> None:
        monitor = self.make_monitor(post_latch_tail_capacity=5)
        monitor.reauthorize(1)
        next_source = self.qualify_at_rate(monitor, rate_hz=2.0)
        for index in range(4):
            self.observe(monitor, next_source + index * 500_000, 5.0)
        snapshot = monitor.trigger_snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        snapshot_value = snapshot
        snapshot_trace = snapshot.trace
        snapshot_episode = snapshot.episode_observations

        for index in range(100):
            self.observe(monitor, next_source + (index + 4) * 500_000, float(index % 7))
        self.assertIs(monitor.trigger_snapshot, snapshot_value)
        self.assertEqual(monitor.trigger_snapshot.trace, snapshot_trace)
        self.assertEqual(monitor.trigger_snapshot.episode_observations, snapshot_episode)
        self.assertEqual(len(monitor.post_latch_tail), 5)
        self.assertEqual(monitor.counters.post_latch_events, 100)
        self.assertTrue(all(event.reason == "already_latched" for event in monitor.post_latch_tail))
        with self.assertRaises(FrozenInstanceError):
            snapshot.episode_source_span_s = 0.0  # type: ignore[misc]

    def test_layered_telemetry_exposes_semantic_counts(self) -> None:
        monitor = self.make_monitor()
        monitor.reauthorize(7)
        self.observe(monitor, 0, 1.0, epoch=7)
        telemetry = monitor.telemetry()
        self.assertEqual(
            set(telemetry),
            {"input", "authorization", "continuity", "qualification", "evidence", "terminal"},
        )
        self.assertEqual(telemetry["input"]["input_events"], 1)
        self.assertEqual(telemetry["authorization"]["authorized_source_epoch"], 7)
        self.assertEqual(telemetry["qualification"]["state"], "UNQUALIFIED")

    def test_configuration_rejects_ambiguous_or_nonfinite_bounds(self) -> None:
        with self.assertRaises(ValueError):
            v6.ResidualPersistenceConfig(quiet_nis_threshold=4.0, high_nis_threshold=4.0)
        with self.assertRaises(ValueError):
            v6.ResidualPersistenceConfig(high_episode_min_source_span_s=math.nan)
        with self.assertRaises(ValueError):
            v6.ResidualPersistenceConfig(high_episode_min_observations=1)


class ResidualPersistenceV6CampaignTests(unittest.TestCase):
    def test_protocol_and_dependency_contract_fail_closed(self) -> None:
        protocol = campaign.load_protocol()
        self.assertEqual(protocol["status"], "opened_train_tune_no_holdout")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "protocol.json"
            broken = copy.deepcopy(protocol)
            broken["status"] = "sealed_holdout"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                campaign.load_protocol(path)
            broken = copy.deepcopy(protocol)
            broken["dependencies"]["base_oracle_runner_file_sha256"] = "0" * 64
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaises(ValueError):
                campaign.load_protocol(path)

    def test_common_random_stream_covers_the_complete_paired_case(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        nominal = next(case for case in protocol["case_matrix"] if case["name"] == "nominal")
        pulse = next(
            case for case in protocol["case_matrix"] if case["name"] == "tas_offset_pulse_1p0s"
        )
        first = campaign.generate_case_stream(nominal, protocol, base_protocol, seed=62001)
        second = campaign.generate_case_stream(pulse, protocol, base_protocol, seed=62001)
        self.assertEqual(first.random_stream_sha256, second.random_stream_sha256)
        self.assertEqual(len(first.observations), len(second.observations))
        injection_index = next(
            index
            for index, observation in enumerate(first.observations)
            if observation.tas_timestamp_us >= first.injection_start_source_timestamp_us
        )
        self.assertEqual(first.observations[:injection_index], second.observations[:injection_index])
        self.assertNotEqual(
            first.observations[injection_index].tas_m_s,
            second.observations[injection_index].tas_m_s,
        )

    def test_exact_one_sided_binomial_bounds_cover_zero_and_all_events(self) -> None:
        confidence = 0.975
        self.assertAlmostEqual(
            campaign.clopper_pearson_upper(0, 128, confidence),
            1.0 - math.pow(1.0 - confidence, 1.0 / 128.0),
            places=12,
        )
        self.assertAlmostEqual(
            campaign.clopper_pearson_lower(128, 128, confidence),
            math.pow(1.0 - confidence, 1.0 / 128.0),
            places=12,
        )

    def test_nominal_and_gap_cases_have_real_monitor_coverage(self) -> None:
        protocol = campaign.load_protocol()
        base_protocol = campaign.resolve_base_protocol(protocol)
        cases = {
            case["name"]: case
            for case in protocol["case_matrix"]
            if case["name"] in {"nominal", "gap_then_tas_offset_pulse_1p0s"}
        }
        nominal = campaign.evaluate_case(cases["nominal"], protocol, base_protocol, seed=62001)
        gap = campaign.evaluate_case(
            cases["gap_then_tas_offset_pulse_1p0s"], protocol, base_protocol, seed=62001
        )
        minimum_active = float(protocol["scenario"]["minimum_pre_injection_active_source_time_s"])
        self.assertTrue(nominal["monitor"]["armed"])
        self.assertGreaterEqual(
            float(nominal["monitor"]["pre_injection_active_source_time_s"]), minimum_active
        )
        self.assertGreater(int(nominal["nis_fed_observations"]), 0)
        self.assertFalse(nominal["monitor_latched"])
        self.assertGreaterEqual(int(gap["monitor"]["source_gap_events"]), 1)
        self.assertFalse(gap["monitor_latched"])


if __name__ == "__main__":
    unittest.main()
