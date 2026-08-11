#!/usr/bin/env python3
"""Standalone source-time residual-persistence monitor for v7 development.

This module is deliberately independent from the v2/v4/v5 campaign runners.
It defines only causal monitor semantics and telemetry; it does not identify
which physical source caused a residual and it does not authorize flight use.

The state machine is intentionally conservative::

    UNQUALIFIED -> QUIET_CONFIRMED -> HIGH_EPISODE -> LATCHED

An explicit :meth:`reauthorize` call begins a source epoch.  Consecutive valid,
quiet residuals must then provide both source-time coverage and observation
coverage before high evidence is accepted.  Invalid/no-NIS input and source
epoch changes revoke authorization.  Timestamp gaps restart qualification,
while non-monotonic timestamps revoke authorization and never contribute.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque


class MonitorState(str, Enum):
    """Externally visible residual-persistence state."""

    UNQUALIFIED = "UNQUALIFIED"
    QUIET_CONFIRMED = "QUIET_CONFIRMED"
    HIGH_EPISODE = "HIGH_EPISODE"
    LATCHED = "LATCHED"


@dataclass(frozen=True)
class ResidualPersistenceConfig:
    """Frozen causal timing and evidence contract.

    All durations use source time except ``maximum_arrival_gap_s``.  Warmup
    first requires uninterrupted valid-source coverage by both observation
    count and source-time span.  A separate quiet run then establishes an
    explicit causal onset boundary.  A high episode likewise needs both count
    and source-time span, so the condition remains meaningful when the input
    rate changes.
    """

    warmup_min_valid_observations: int = 4
    warmup_min_source_span_s: float = 1.0
    quiet_confirmation_min_observations: int = 2
    quiet_confirmation_min_source_span_s: float = 0.5
    quiet_nis_threshold: float = 2.0
    high_nis_threshold: float = 4.0
    high_episode_min_observations: int = 4
    high_episode_min_source_span_s: float = 1.5
    maximum_source_gap_s: float = 1.25
    maximum_arrival_gap_s: float = 1.25
    pre_trigger_trace_capacity: int = 24
    post_latch_tail_capacity: int = 12

    def __post_init__(self) -> None:
        for name, value, minimum in (
            ("warmup_min_valid_observations", self.warmup_min_valid_observations, 1),
            (
                "quiet_confirmation_min_observations",
                self.quiet_confirmation_min_observations,
                1,
            ),
            ("high_episode_min_observations", self.high_episode_min_observations, 2),
            ("pre_trigger_trace_capacity", self.pre_trigger_trace_capacity, 1),
            ("post_latch_tail_capacity", self.post_latch_tail_capacity, 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        for name, value, allow_zero in (
            ("warmup_min_source_span_s", self.warmup_min_source_span_s, True),
            (
                "quiet_confirmation_min_source_span_s",
                self.quiet_confirmation_min_source_span_s,
                True,
            ),
            ("quiet_nis_threshold", self.quiet_nis_threshold, True),
            ("high_nis_threshold", self.high_nis_threshold, False),
            ("high_episode_min_source_span_s", self.high_episode_min_source_span_s, False),
            ("maximum_source_gap_s", self.maximum_source_gap_s, False),
            ("maximum_arrival_gap_s", self.maximum_arrival_gap_s, False),
        ):
            if not math.isfinite(value) or value < 0.0 or (not allow_zero and value <= 0.0):
                relation = "non-negative" if allow_zero else "positive"
                raise ValueError(f"{name} must be finite and {relation}")
        if self.quiet_nis_threshold >= self.high_nis_threshold:
            raise ValueError("quiet NIS threshold must be below high NIS threshold")


@dataclass(frozen=True)
class EvidencePoint:
    """One accepted high residual in source and arrival time."""

    source_epoch: int
    source_timestamp_us: int
    arrival_timestamp_us: int
    nis: float


@dataclass(frozen=True)
class TraceEvent:
    """One bounded diagnostic event; hidden labels and truth are absent."""

    input_index: int
    source_epoch: int
    source_timestamp_us: int
    arrival_timestamp_us: int
    nis: float | None
    source_valid: bool
    state_before: MonitorState
    state_after: MonitorState
    accepted_residual: bool
    contributed_to_high_episode: bool
    reason: str


@dataclass(frozen=True)
class TriggerSnapshot:
    """Immutable evidence captured exactly when the terminal latch occurs."""

    source_epoch: int
    latch_source_timestamp_us: int
    latch_arrival_timestamp_us: int
    episode_start_source_timestamp_us: int
    episode_start_arrival_timestamp_us: int
    quiet_boundary_source_timestamp_us: int | None
    quiet_boundary_arrival_timestamp_us: int | None
    episode_source_span_s: float
    episode_observations: tuple[EvidencePoint, ...]
    trace: tuple[TraceEvent, ...]


@dataclass(frozen=True)
class MonitorDecision:
    """Result of consuming one input event."""

    state: MonitorState
    accepted_residual: bool
    contributed_to_high_episode: bool
    source_gap_observed: bool
    arrival_gap_observed: bool
    reason: str


@dataclass
class MonitorCounters:
    """Mutable counters exposed through a layered read-only snapshot."""

    input_events: int = 0
    valid_residual_events: int = 0
    invalid_source_events: int = 0
    missing_nis_events: int = 0
    invalid_nis_events: int = 0
    invalid_timestamp_events: int = 0
    unauthorized_events: int = 0
    source_epoch_mismatch_events: int = 0
    source_epoch_reset_events: int = 0
    nonmonotonic_source_events: int = 0
    nonmonotonic_arrival_events: int = 0
    source_gap_events: int = 0
    arrival_gap_events: int = 0
    gap_resets_empty_episode: int = 0
    gap_resets_nonempty_episode: int = 0
    continuity_resets: int = 0
    authorization_revocations: int = 0
    reauthorizations: int = 0
    warmup_contributions: int = 0
    warmup_completions: int = 0
    warmup_resets: int = 0
    quiet_contributions: int = 0
    quiet_resets: int = 0
    quiet_confirmations: int = 0
    quiet_confirmation_losses: int = 0
    high_episode_starts: int = 0
    high_episode_contributions: int = 0
    high_episode_aborts: int = 0
    latches: int = 0
    post_latch_events: int = 0


class SourceTimeResidualPersistenceMonitor:
    """Causal source-time persistence monitor with explicit authorization.

    ``observe`` never receives injection labels or truth.  A source epoch is an
    upstream identity/continuity token.  Changing it without first calling
    :meth:`reauthorize` revokes the monitor, as do invalid source data, missing
    NIS, invalid NIS, and non-monotonic timestamps.
    """

    def __init__(self, config: ResidualPersistenceConfig) -> None:
        self.config = config
        self.state = MonitorState.UNQUALIFIED
        self.counters = MonitorCounters()
        self.authorized_source_epoch: int | None = None
        self.last_source_timestamp_us: int | None = None
        self.last_arrival_timestamp_us: int | None = None
        self._warmup_start_source_timestamp_us: int | None = None
        self._warmup_observations = 0
        self._warmup_ready = False
        self._quiet_start_source_timestamp_us: int | None = None
        self._quiet_start_arrival_timestamp_us: int | None = None
        self._quiet_observations = 0
        self._high_episode: list[EvidencePoint] = []
        self._input_index = 0
        self._pre_trigger_trace: Deque[TraceEvent] = deque(
            maxlen=config.pre_trigger_trace_capacity
        )
        self._post_latch_tail: Deque[TraceEvent] = deque(
            maxlen=config.post_latch_tail_capacity
        )
        self.trigger_snapshot: TriggerSnapshot | None = None

    @staticmethod
    def _seconds_to_us(value_s: float) -> int:
        return int(round(value_s * 1.0e6))

    @staticmethod
    def _valid_epoch(source_epoch: int) -> bool:
        return not isinstance(source_epoch, bool) and isinstance(source_epoch, int) and source_epoch >= 0

    @staticmethod
    def _valid_timestamp(timestamp_us: int) -> bool:
        return not isinstance(timestamp_us, bool) and isinstance(timestamp_us, int) and timestamp_us >= 0

    @property
    def warmup_observations(self) -> int:
        return self._warmup_observations

    @property
    def high_episode_observations(self) -> tuple[EvidencePoint, ...]:
        return tuple(self._high_episode)

    @property
    def warmup_ready(self) -> bool:
        return self._warmup_ready

    @property
    def quiet_observations(self) -> int:
        return self._quiet_observations

    @property
    def pre_trigger_trace(self) -> tuple[TraceEvent, ...]:
        return tuple(self._pre_trigger_trace)

    @property
    def post_latch_tail(self) -> tuple[TraceEvent, ...]:
        return tuple(self._post_latch_tail)

    def telemetry(self) -> dict[str, dict[str, int | str | None]]:
        """Return counters grouped by semantic layer."""

        return {
            "input": {
                "input_events": self.counters.input_events,
                "valid_residual_events": self.counters.valid_residual_events,
                "invalid_source_events": self.counters.invalid_source_events,
                "missing_nis_events": self.counters.missing_nis_events,
                "invalid_nis_events": self.counters.invalid_nis_events,
                "invalid_timestamp_events": self.counters.invalid_timestamp_events,
            },
            "authorization": {
                "authorized_source_epoch": self.authorized_source_epoch,
                "unauthorized_events": self.counters.unauthorized_events,
                "source_epoch_mismatch_events": self.counters.source_epoch_mismatch_events,
                "source_epoch_reset_events": self.counters.source_epoch_reset_events,
                "authorization_revocations": self.counters.authorization_revocations,
                "reauthorizations": self.counters.reauthorizations,
            },
            "continuity": {
                "nonmonotonic_source_events": self.counters.nonmonotonic_source_events,
                "nonmonotonic_arrival_events": self.counters.nonmonotonic_arrival_events,
                "source_gap_events": self.counters.source_gap_events,
                "arrival_gap_events": self.counters.arrival_gap_events,
                "gap_resets_empty_episode": self.counters.gap_resets_empty_episode,
                "gap_resets_nonempty_episode": self.counters.gap_resets_nonempty_episode,
                "continuity_resets": self.counters.continuity_resets,
            },
            "qualification": {
                "state": self.state.value,
                "warmup_observations": self._warmup_observations,
                "warmup_ready": 1 if self._warmup_ready else 0,
                "warmup_contributions": self.counters.warmup_contributions,
                "warmup_completions": self.counters.warmup_completions,
                "warmup_resets": self.counters.warmup_resets,
                "quiet_observations": self._quiet_observations,
                "quiet_contributions": self.counters.quiet_contributions,
                "quiet_resets": self.counters.quiet_resets,
                "quiet_confirmations": self.counters.quiet_confirmations,
                "quiet_confirmation_losses": self.counters.quiet_confirmation_losses,
            },
            "evidence": {
                "high_episode_observations": len(self._high_episode),
                "high_episode_starts": self.counters.high_episode_starts,
                "high_episode_contributions": self.counters.high_episode_contributions,
                "high_episode_aborts": self.counters.high_episode_aborts,
            },
            "terminal": {
                "latches": self.counters.latches,
                "post_latch_events": self.counters.post_latch_events,
                "trigger_snapshot_available": 1 if self.trigger_snapshot is not None else 0,
            },
        }

    def reauthorize(self, source_epoch: int) -> None:
        """Explicitly authorize one epoch and restart qualification.

        A retained trigger snapshot is intentionally not mutated by this call;
        it remains available for audit until a later latch replaces the
        monitor's ``trigger_snapshot`` reference.
        """

        if not self._valid_epoch(source_epoch):
            raise ValueError("source epoch must be a non-negative integer")
        self.authorized_source_epoch = source_epoch
        self.state = MonitorState.UNQUALIFIED
        self.last_source_timestamp_us = None
        self.last_arrival_timestamp_us = None
        self._reset_warmup(count_reset=False)
        self._high_episode.clear()
        self._pre_trigger_trace.clear()
        self._post_latch_tail.clear()
        self.counters.reauthorizations += 1

    def source_epoch_reset(self) -> None:
        """Fail closed after an upstream source-identity/continuity reset."""

        self.counters.source_epoch_reset_events += 1
        self._revoke_authorization()

    def _reset_warmup(self, *, count_reset: bool) -> None:
        if count_reset and (self._warmup_observations or self._warmup_ready):
            self.counters.warmup_resets += 1
        self._warmup_start_source_timestamp_us = None
        self._warmup_observations = 0
        self._warmup_ready = False
        self._reset_quiet(count_reset=count_reset)

    def _reset_quiet(self, *, count_reset: bool) -> None:
        if count_reset and self._quiet_observations:
            self.counters.quiet_resets += 1
        self._quiet_start_source_timestamp_us = None
        self._quiet_start_arrival_timestamp_us = None
        self._quiet_observations = 0

    def _contribute_quiet(self, source_timestamp_us: int, arrival_timestamp_us: int) -> bool:
        if self._quiet_start_source_timestamp_us is None:
            self._quiet_start_source_timestamp_us = source_timestamp_us
            self._quiet_start_arrival_timestamp_us = arrival_timestamp_us
        self._quiet_observations += 1
        self.counters.quiet_contributions += 1
        quiet_span_us = source_timestamp_us - self._quiet_start_source_timestamp_us
        return (
            self._quiet_observations >= self.config.quiet_confirmation_min_observations
            and quiet_span_us
            >= self._seconds_to_us(self.config.quiet_confirmation_min_source_span_s)
        )

    def _reset_continuity(self, *, nonempty_high_episode: bool) -> None:
        self.counters.continuity_resets += 1
        if nonempty_high_episode:
            self.counters.gap_resets_nonempty_episode += 1
        else:
            self.counters.gap_resets_empty_episode += 1
        if self.state is MonitorState.QUIET_CONFIRMED:
            self.counters.quiet_confirmation_losses += 1
        if self.state is MonitorState.HIGH_EPISODE and self._high_episode:
            self.counters.high_episode_aborts += 1
        self.state = MonitorState.UNQUALIFIED
        self._reset_warmup(count_reset=True)
        self._high_episode.clear()

    def _revoke_authorization(self) -> None:
        if self.authorized_source_epoch is not None:
            self.counters.authorization_revocations += 1
        self.authorized_source_epoch = None
        if self.state is not MonitorState.LATCHED:
            self.state = MonitorState.UNQUALIFIED
        self.last_source_timestamp_us = None
        self.last_arrival_timestamp_us = None
        self._reset_warmup(count_reset=True)
        self._high_episode.clear()

    def _trace_event(
        self,
        *,
        source_epoch: int,
        source_timestamp_us: int,
        arrival_timestamp_us: int,
        nis: float | None,
        source_valid: bool,
        state_before: MonitorState,
        accepted_residual: bool,
        contributed_to_high_episode: bool,
        reason: str,
    ) -> TraceEvent:
        return TraceEvent(
            input_index=self._input_index,
            source_epoch=source_epoch,
            source_timestamp_us=source_timestamp_us,
            arrival_timestamp_us=arrival_timestamp_us,
            nis=nis,
            source_valid=source_valid,
            state_before=state_before,
            state_after=self.state,
            accepted_residual=accepted_residual,
            contributed_to_high_episode=contributed_to_high_episode,
            reason=reason,
        )

    def _retain_trace(self, event: TraceEvent) -> None:
        if event.state_before is MonitorState.LATCHED or self.state is MonitorState.LATCHED:
            if self.trigger_snapshot is not None and event.reason != "high_episode_latched":
                self._post_latch_tail.append(event)
                self.counters.post_latch_events += 1
            return
        self._pre_trigger_trace.append(event)

    def _decision(
        self,
        *,
        accepted: bool,
        contributed: bool,
        source_gap: bool,
        arrival_gap: bool,
        reason: str,
    ) -> MonitorDecision:
        return MonitorDecision(
            state=self.state,
            accepted_residual=accepted,
            contributed_to_high_episode=contributed,
            source_gap_observed=source_gap,
            arrival_gap_observed=arrival_gap,
            reason=reason,
        )

    def _reject_event(
        self,
        *,
        source_epoch: int,
        source_timestamp_us: int,
        arrival_timestamp_us: int,
        nis: float | None,
        source_valid: bool,
        state_before: MonitorState,
        reason: str,
        revoke: bool,
        source_gap: bool = False,
        arrival_gap: bool = False,
    ) -> MonitorDecision:
        if revoke and self.state is not MonitorState.LATCHED:
            self._revoke_authorization()
        event = self._trace_event(
            source_epoch=source_epoch,
            source_timestamp_us=source_timestamp_us,
            arrival_timestamp_us=arrival_timestamp_us,
            nis=nis,
            source_valid=source_valid,
            state_before=state_before,
            accepted_residual=False,
            contributed_to_high_episode=False,
            reason=reason,
        )
        self._retain_trace(event)
        return self._decision(
            accepted=False,
            contributed=False,
            source_gap=source_gap,
            arrival_gap=arrival_gap,
            reason=reason,
        )

    def observe(
        self,
        *,
        source_epoch: int,
        source_timestamp_us: int,
        arrival_timestamp_us: int,
        nis: float | None,
        source_valid: bool = True,
    ) -> MonitorDecision:
        """Consume one causal residual event without hidden labels or truth."""

        self._input_index += 1
        self.counters.input_events += 1
        state_before = self.state

        if not self._valid_epoch(source_epoch):
            self.counters.source_epoch_mismatch_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="invalid_source_epoch",
                revoke=True,
            )
        if (
            not self._valid_timestamp(source_timestamp_us)
            or not self._valid_timestamp(arrival_timestamp_us)
            or arrival_timestamp_us < source_timestamp_us
        ):
            self.counters.invalid_timestamp_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="invalid_causal_timestamp",
                revoke=True,
            )
        if self.state is MonitorState.LATCHED:
            event = self._trace_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                accepted_residual=False,
                contributed_to_high_episode=False,
                reason="already_latched",
            )
            self._retain_trace(event)
            return self._decision(
                accepted=False,
                contributed=False,
                source_gap=False,
                arrival_gap=False,
                reason="already_latched",
            )
        if self.authorized_source_epoch is None:
            self.counters.unauthorized_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="source_epoch_not_authorized",
                revoke=False,
            )
        if source_epoch != self.authorized_source_epoch:
            self.counters.source_epoch_mismatch_events += 1
            self.counters.source_epoch_reset_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="source_epoch_changed_without_reauthorization",
                revoke=True,
            )

        source_nonmonotonic = (
            self.last_source_timestamp_us is not None
            and source_timestamp_us <= self.last_source_timestamp_us
        )
        arrival_nonmonotonic = (
            self.last_arrival_timestamp_us is not None
            and arrival_timestamp_us <= self.last_arrival_timestamp_us
        )
        if source_nonmonotonic:
            self.counters.nonmonotonic_source_events += 1
        if arrival_nonmonotonic:
            self.counters.nonmonotonic_arrival_events += 1
        if source_nonmonotonic or arrival_nonmonotonic:
            reasons = []
            if source_nonmonotonic:
                reasons.append("source")
            if arrival_nonmonotonic:
                reasons.append("arrival")
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason=f"nonmonotonic_{'_and_'.join(reasons)}_timestamp",
                revoke=True,
            )

        source_gap = (
            self.last_source_timestamp_us is not None
            and source_timestamp_us - self.last_source_timestamp_us
            > self._seconds_to_us(self.config.maximum_source_gap_s)
        )
        arrival_gap = (
            self.last_arrival_timestamp_us is not None
            and arrival_timestamp_us - self.last_arrival_timestamp_us
            > self._seconds_to_us(self.config.maximum_arrival_gap_s)
        )
        if source_gap:
            self.counters.source_gap_events += 1
        if arrival_gap:
            self.counters.arrival_gap_events += 1
        if source_gap or arrival_gap:
            self._reset_continuity(nonempty_high_episode=bool(self._high_episode))

        self.last_source_timestamp_us = source_timestamp_us
        self.last_arrival_timestamp_us = arrival_timestamp_us

        if not source_valid:
            self.counters.invalid_source_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="source_invalid",
                revoke=True,
                source_gap=source_gap,
                arrival_gap=arrival_gap,
            )
        if nis is None:
            self.counters.missing_nis_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="missing_nis",
                revoke=True,
                source_gap=source_gap,
                arrival_gap=arrival_gap,
            )
        if not math.isfinite(nis) or nis < 0.0:
            self.counters.invalid_nis_events += 1
            return self._reject_event(
                source_epoch=source_epoch,
                source_timestamp_us=source_timestamp_us,
                arrival_timestamp_us=arrival_timestamp_us,
                nis=nis,
                source_valid=source_valid,
                state_before=state_before,
                reason="invalid_nis",
                revoke=True,
                source_gap=source_gap,
                arrival_gap=arrival_gap,
            )

        self.counters.valid_residual_events += 1
        contributed = False
        reason = ""

        if self.state is MonitorState.UNQUALIFIED:
            if not self._warmup_ready:
                if self._warmup_start_source_timestamp_us is None:
                    self._warmup_start_source_timestamp_us = source_timestamp_us
                self._warmup_observations += 1
                self.counters.warmup_contributions += 1
                warmup_span_us = source_timestamp_us - self._warmup_start_source_timestamp_us
                if (
                    self._warmup_observations >= self.config.warmup_min_valid_observations
                    and warmup_span_us
                    >= self._seconds_to_us(self.config.warmup_min_source_span_s)
                ):
                    self._warmup_ready = True
                    self.counters.warmup_completions += 1
                    reason = "warmup_coverage_complete"
                else:
                    reason = "warmup_coverage"
            else:
                reason = "warmup_complete_waiting_for_quiet"

            if self._warmup_ready:
                if nis <= self.config.quiet_nis_threshold:
                    if self._contribute_quiet(source_timestamp_us, arrival_timestamp_us):
                        self.state = MonitorState.QUIET_CONFIRMED
                        self.counters.quiet_confirmations += 1
                        reason = "quiet_confirmed"
                    else:
                        reason = "quiet_confirmation"
                else:
                    self._reset_quiet(count_reset=True)
                    reason = "warmup_complete_nonquiet"

        elif self.state is MonitorState.QUIET_CONFIRMED:
            if nis <= self.config.quiet_nis_threshold:
                reason = "quiet_confirmed_residual"
            elif nis < self.config.high_nis_threshold:
                # A mid-band residual invalidates the prior quiet boundary.
                # Requiring a fresh quiet run prevents an episode that began
                # before an unlabelled transition from being credited later.
                self.counters.quiet_confirmation_losses += 1
                self.state = MonitorState.UNQUALIFIED
                self._reset_quiet(count_reset=True)
                reason = "quiet_confirmation_lost_midband"
            else:
                point = EvidencePoint(
                    source_epoch=source_epoch,
                    source_timestamp_us=source_timestamp_us,
                    arrival_timestamp_us=arrival_timestamp_us,
                    nis=float(nis),
                )
                self._high_episode = [point]
                self.state = MonitorState.HIGH_EPISODE
                self.counters.high_episode_starts += 1
                self.counters.high_episode_contributions += 1
                contributed = True
                reason = "high_episode_started"

        elif self.state is MonitorState.HIGH_EPISODE:
            if nis >= self.config.high_nis_threshold:
                point = EvidencePoint(
                    source_epoch=source_epoch,
                    source_timestamp_us=source_timestamp_us,
                    arrival_timestamp_us=arrival_timestamp_us,
                    nis=float(nis),
                )
                self._high_episode.append(point)
                self.counters.high_episode_contributions += 1
                contributed = True
                source_span_us = (
                    source_timestamp_us - self._high_episode[0].source_timestamp_us
                )
                if (
                    len(self._high_episode) >= self.config.high_episode_min_observations
                    and source_span_us
                    >= self._seconds_to_us(self.config.high_episode_min_source_span_s)
                ):
                    self.state = MonitorState.LATCHED
                    self.counters.latches += 1
                    reason = "high_episode_latched"
                else:
                    reason = "high_episode_persisting"
            else:
                self.counters.high_episode_aborts += 1
                self._high_episode.clear()
                self.state = MonitorState.UNQUALIFIED
                self._reset_quiet(count_reset=True)
                reason = "high_episode_aborted_by_low_residual"
        else:  # pragma: no cover - LATCHED returns before residual processing.
            raise RuntimeError(f"unsupported monitor state {self.state}")

        event = self._trace_event(
            source_epoch=source_epoch,
            source_timestamp_us=source_timestamp_us,
            arrival_timestamp_us=arrival_timestamp_us,
            nis=float(nis),
            source_valid=source_valid,
            state_before=state_before,
            accepted_residual=True,
            contributed_to_high_episode=contributed,
            reason=reason,
        )
        if reason == "high_episode_latched":
            source_span_s = (
                self._high_episode[-1].source_timestamp_us
                - self._high_episode[0].source_timestamp_us
            ) * 1.0e-6
            self.trigger_snapshot = TriggerSnapshot(
                source_epoch=source_epoch,
                latch_source_timestamp_us=source_timestamp_us,
                latch_arrival_timestamp_us=arrival_timestamp_us,
                episode_start_source_timestamp_us=self._high_episode[0].source_timestamp_us,
                episode_start_arrival_timestamp_us=self._high_episode[0].arrival_timestamp_us,
                quiet_boundary_source_timestamp_us=self._quiet_start_source_timestamp_us,
                quiet_boundary_arrival_timestamp_us=self._quiet_start_arrival_timestamp_us,
                episode_source_span_s=source_span_s,
                episode_observations=tuple(self._high_episode),
                trace=tuple(self._pre_trigger_trace) + (event,),
            )
            self._post_latch_tail.clear()
        else:
            self._retain_trace(event)
        return self._decision(
            accepted=True,
            contributed=contributed,
            source_gap=source_gap,
            arrival_gap=arrival_gap,
            reason=reason,
        )


__all__ = [
    "EvidencePoint",
    "MonitorCounters",
    "MonitorDecision",
    "MonitorState",
    "ResidualPersistenceConfig",
    "SourceTimeResidualPersistenceMonitor",
    "TraceEvent",
    "TriggerSnapshot",
]
