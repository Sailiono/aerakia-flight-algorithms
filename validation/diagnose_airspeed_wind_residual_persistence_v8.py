#!/usr/bin/env python3
"""Focused, non-promoting design diagnostics for a possible v8 residual monitor.

This file is intentionally *not* a v8 protocol, train runner, or flight
component.  It leaves v7 source, protocol, scorer, and immutable train
artifact untouched.  Its purpose is narrower: make the v7 quiet-boundary
failure mode executable, then compare four causal policy shapes on tiny,
hand-authored traces before anyone freezes another development protocol.

The probes accept only source/arrival timestamps, a residual NIS, validity,
and epoch continuity.  Scenario names and optional injection timestamps are
used only by the offline report after a probe has run; they are never supplied
to a policy decision.

The optional ``--replay-v7-case`` path reuses v7 only as a frozen synthetic
NIS provider.  It replays one existing stream, not the v7 train campaign, and
does not write any v7 artifact.  It is useful for reproducing the known seed
71101 mechanism, not for selecting parameters.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parents[1]


class PolicyKind(str, Enum):
    """Small causal-policy family explored by this development-only probe."""

    RECENT_BOUNDARY = "recent_boundary"
    GRADED_EVIDENCE = "graded_evidence"
    BOUNDED_RETRY = "bounded_retry"
    PARTIAL_QUIET_PROBATION = "partial_quiet_probation"


class ProbeState(str, Enum):
    """Externally visible state for diagnostics; this is not a product API."""

    UNQUALIFIED = "UNQUALIFIED"
    BOUNDARY_ACTIVE = "BOUNDARY_ACTIVE"
    HIGH_EPISODE = "HIGH_EPISODE"
    LATCHED = "LATCHED"


@dataclass(frozen=True)
class ProbeConfig:
    """Source-time parameters shared by the candidate-shape probes.

    ``recent_quiet_boundary_max_age_s`` deliberately bounds historic quiet
    evidence.  It is a design value for a focused diagnostic only, not a
    tuned or promoted flight threshold.
    """

    warmup_min_observations: int = 4
    warmup_min_source_span_s: float = 1.0
    quiet_min_observations: int = 2
    # Match the v7 protocol's one-second quiet requirement by default.  The
    # focused probe must reproduce the root-cause trace where two 0.5 s quiet
    # samples after a mid-band point are *not* a fresh boundary.
    quiet_min_source_span_s: float = 1.0
    quiet_nis_threshold: float = 2.0
    high_nis_threshold: float = 4.0
    high_min_observations: int = 4
    high_min_source_span_s: float = 1.5
    maximum_source_gap_s: float = 1.25
    maximum_arrival_gap_s: float = 1.25
    recent_quiet_boundary_max_age_s: float = 3.0
    graded_midband_decay_per_source_s: float = 0.5
    retry_max_aborts_after_boundary: int = 1
    partial_retry_max_aborts_after_boundary: int = 1
    partial_quiet_min_observations: int = 2
    partial_quiet_min_source_span_s: float = 0.5
    partial_high_min_observations: int = 5
    partial_high_min_source_span_s: float = 2.0

    def __post_init__(self) -> None:
        if self.quiet_nis_threshold >= self.high_nis_threshold:
            raise ValueError("quiet threshold must be below high threshold")
        for name in (
            "warmup_min_observations",
            "quiet_min_observations",
            "high_min_observations",
            "partial_quiet_min_observations",
            "partial_high_min_observations",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be an integer >= 1")
        if (
            isinstance(self.retry_max_aborts_after_boundary, bool)
            or not isinstance(self.retry_max_aborts_after_boundary, int)
            or self.retry_max_aborts_after_boundary < 0
        ):
            raise ValueError("retry_max_aborts_after_boundary must be an integer >= 0")
        if (
            isinstance(self.partial_retry_max_aborts_after_boundary, bool)
            or not isinstance(self.partial_retry_max_aborts_after_boundary, int)
            or self.partial_retry_max_aborts_after_boundary < 0
        ):
            raise ValueError(
                "partial_retry_max_aborts_after_boundary must be an integer >= 0"
            )
        for name in (
            "warmup_min_source_span_s",
            "quiet_min_source_span_s",
            "high_min_source_span_s",
            "maximum_source_gap_s",
            "maximum_arrival_gap_s",
            "recent_quiet_boundary_max_age_s",
            "graded_midband_decay_per_source_s",
            "partial_quiet_min_source_span_s",
            "partial_high_min_source_span_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class ProbeInput:
    """One causal input.  No scenario label or injection truth is present."""

    source_timestamp_us: int
    arrival_timestamp_us: int
    nis: float | None
    source_valid: bool = True
    source_epoch: int = 0


@dataclass(frozen=True)
class ProbeEvent:
    """Audit event emitted by a probe after it consumes one input."""

    source_timestamp_us: int
    arrival_timestamp_us: int
    nis: float | None
    state_before: ProbeState
    state_after: ProbeState
    reason: str
    high_observations: int
    high_source_span_s: float
    graded_evidence_s: float
    recent_boundary_age_s: float | None
    retry_aborts_used: int
    boundary_is_partial: bool
    partial_retry_aborts_used: int
    partial_retry_exhausted: bool
    full_boundary_seen: bool


@dataclass(frozen=True)
class ProbeResult:
    """Compact per-policy trace outcome."""

    policy: str
    final_state: str
    latched: bool
    latch_source_timestamp_us: int | None
    episode_start_source_timestamp_us: int | None
    boundary_source_timestamp_us: int | None
    boundary_is_partial: bool
    episode_boundary_is_partial: bool
    full_boundary_seen: bool
    reset_count: int
    retry_aborts_used: int
    partial_retry_aborts_used: int
    partial_retry_exhausted: bool
    trace: tuple[ProbeEvent, ...]


def _seconds_to_us(value_s: float) -> int:
    return int(round(value_s * 1.0e6))


def _duration_s(later_us: int, earlier_us: int) -> float:
    return (later_us - earlier_us) * 1.0e-6


class CausalPolicyProbe:
    """Executable policy-shape model with v7-style fail-closed continuity.

    The four candidates deliberately share all safety foundations:

    * explicit epoch authorization;
    * strictly monotonic source and arrival time;
    * source/arrival gap reset;
    * invalid/no-NIS reset; and
    * a bounded, causally established quiet boundary.

    They differ only after a valid boundary exists:

    ``RECENT_BOUNDARY``
        A new *contiguous* high episode may begin after a recent boundary even
        if a finite mid-band interval intervened.  It is the minimal fix for
        the v7 dead-end.

    ``GRADED_EVIDENCE``
        High evidence accumulates in source time and mid-band input decays it.
        It must still be inside the recent-boundary horizon and satisfy a high
        observation count.  This is a CUSUM-like shape, not an implemented
        estimator statistic.

    ``BOUNDED_RETRY``
        A broken high episode may restart only a finite number of times before
        a fresh quiet boundary is required.  It prevents an unlimited retry
        loop from turning a stale boundary into eventual certainty.

    ``PARTIAL_QUIET_PROBATION``
        A short, explicitly bounded quiet run may create a probationary
        boundary.  Any episode that starts from that boundary must satisfy a
        stricter high-observation/source-span requirement.  It is only
        admissible after one complete quiet boundary in the current epoch and
        has its own finite retry budget.  This is a diagnostic candidate for
        the v7 dead-end, not a relaxed qualification rule.
    """

    def __init__(self, policy: PolicyKind, config: ProbeConfig) -> None:
        self.policy = policy
        self.config = config
        self.state = ProbeState.UNQUALIFIED
        self.authorized_epoch: int | None = None
        self.last_source_us: int | None = None
        self.last_arrival_us: int | None = None
        self.warmup_start_us: int | None = None
        self.warmup_observations = 0
        self.warmup_ready = False
        self.quiet_start_us: int | None = None
        self.quiet_observations = 0
        self.last_quiet_boundary_us: int | None = None
        self.boundary_is_partial = False
        # Partial probation cannot bootstrap the initial qualification.  A
        # complete quiet boundary must be observed in this authorization epoch
        # before any partial boundary is admissible.
        self.full_boundary_seen = False
        # A partial boundary is only available after a full boundary was
        # degraded by a non-quiet event or an aborted episode.  This prevents
        # the shortened quiet rule from becoming an alternative startup path.
        self.partial_requalification_eligible = False
        self.high_start_us: int | None = None
        # Snapshot the boundary validity at episode onset.  A valid episode
        # must not be invalidated merely because its 1.5 s evidence window
        # crosses the boundary-age deadline; a *new* retry still has to pass
        # the age check.
        self.high_boundary_source_us: int | None = None
        self.high_boundary_is_partial = False
        self.high_observations = 0
        self.graded_evidence_s = 0.0
        self.retry_aborts_used = 0
        self.retry_exhausted = False
        self.partial_retry_aborts_used = 0
        self.partial_retry_exhausted = False
        self.latch_source_timestamp_us: int | None = None
        self.reset_count = 0
        self.trace: list[ProbeEvent] = []

    def reauthorize(self, source_epoch: int = 0) -> None:
        if isinstance(source_epoch, bool) or not isinstance(source_epoch, int) or source_epoch < 0:
            raise ValueError("source_epoch must be a non-negative integer")
        self.authorized_epoch = source_epoch
        self._clear_all(keep_authorization=True)

    def _clear_all(self, *, keep_authorization: bool) -> None:
        self.state = ProbeState.UNQUALIFIED
        self.last_source_us = None
        self.last_arrival_us = None
        self.warmup_start_us = None
        self.warmup_observations = 0
        self.warmup_ready = False
        self.quiet_start_us = None
        self.quiet_observations = 0
        self.last_quiet_boundary_us = None
        self.boundary_is_partial = False
        self.full_boundary_seen = False
        self.partial_requalification_eligible = False
        self.high_start_us = None
        self.high_boundary_source_us = None
        self.high_boundary_is_partial = False
        self.high_observations = 0
        self.graded_evidence_s = 0.0
        self.retry_aborts_used = 0
        self.retry_exhausted = False
        self.partial_retry_aborts_used = 0
        self.partial_retry_exhausted = False
        self.latch_source_timestamp_us = None
        if not keep_authorization:
            self.authorized_epoch = None

    def _clear_high_episode(self) -> None:
        self.high_start_us = None
        self.high_boundary_source_us = None
        self.high_boundary_is_partial = False
        self.high_observations = 0
        self.graded_evidence_s = 0.0
        if self.state is not ProbeState.LATCHED:
            self.state = (
                ProbeState.BOUNDARY_ACTIVE
                if self.last_quiet_boundary_us is not None
                else ProbeState.UNQUALIFIED
            )

    def _boundary_age_s(self, source_us: int) -> float | None:
        if self.last_quiet_boundary_us is None:
            return None
        return _duration_s(source_us, self.last_quiet_boundary_us)

    def _boundary_is_recent(self, source_us: int) -> bool:
        age_s = self._boundary_age_s(source_us)
        return age_s is not None and age_s <= self.config.recent_quiet_boundary_max_age_s

    def _reset_for_discontinuity(self, *, keep_authorization: bool = False) -> None:
        self.reset_count += 1
        self._clear_all(keep_authorization=keep_authorization)

    def _retire_boundary_for_partial_requalification(self) -> None:
        """Drop a degraded boundary and require fresh causal quiet evidence.

        The old full/partial boundary cannot authorize another high episode.
        A partial quiet run becomes admissible only because this epoch has
        already demonstrated one full boundary; a discontinuity clears that
        historical fact in ``_clear_all``.
        """

        self.last_quiet_boundary_us = None
        self.boundary_is_partial = False
        self.partial_requalification_eligible = self.full_boundary_seen
        if self.state is not ProbeState.LATCHED:
            self.state = ProbeState.UNQUALIFIED

    def _event(self, item: ProbeInput, state_before: ProbeState, reason: str) -> ProbeEvent:
        age_s = self._boundary_age_s(item.source_timestamp_us)
        span_s = (
            0.0
            if self.high_start_us is None
            else _duration_s(item.source_timestamp_us, self.high_start_us)
        )
        event = ProbeEvent(
            source_timestamp_us=item.source_timestamp_us,
            arrival_timestamp_us=item.arrival_timestamp_us,
            nis=item.nis,
            state_before=state_before,
            state_after=self.state,
            reason=reason,
            high_observations=self.high_observations,
            high_source_span_s=span_s,
            graded_evidence_s=self.graded_evidence_s,
            recent_boundary_age_s=age_s,
            retry_aborts_used=self.retry_aborts_used,
            boundary_is_partial=self.boundary_is_partial,
            partial_retry_aborts_used=self.partial_retry_aborts_used,
            partial_retry_exhausted=self.partial_retry_exhausted,
            full_boundary_seen=self.full_boundary_seen,
        )
        self.trace.append(event)
        return event

    def _advance_warmup(self, source_us: int) -> bool:
        if self.warmup_start_us is None:
            self.warmup_start_us = source_us
        self.warmup_observations += 1
        if (
            self.warmup_observations >= self.config.warmup_min_observations
            and _duration_s(source_us, self.warmup_start_us)
            >= self.config.warmup_min_source_span_s
        ):
            self.warmup_ready = True
        return self.warmup_ready

    def _advance_quiet(self, source_us: int) -> str | None:
        if self.quiet_start_us is None:
            self.quiet_start_us = source_us
            self.quiet_observations = 0
        self.quiet_observations += 1
        span_s = _duration_s(source_us, self.quiet_start_us)
        full_confirmed = (
            self.quiet_observations >= self.config.quiet_min_observations
            and span_s >= self.config.quiet_min_source_span_s
        )
        partial_confirmed = (
            self.policy is PolicyKind.PARTIAL_QUIET_PROBATION
            and self.full_boundary_seen
            and self.partial_requalification_eligible
            and self.quiet_observations >= self.config.partial_quiet_min_observations
            and span_s >= self.config.partial_quiet_min_source_span_s
        )
        if full_confirmed:
            # A running quiet interval is still a quiet interval.  Updating the
            # end time on each qualified quiet sample bounds recency by the most
            # recent causally observed baseline, not by its historical start.
            self.last_quiet_boundary_us = source_us
            self.full_boundary_seen = True
            self.boundary_is_partial = False
            self.partial_requalification_eligible = False
            self.retry_aborts_used = 0
            self.retry_exhausted = False
            self.partial_retry_aborts_used = 0
            self.partial_retry_exhausted = False
            self._clear_high_episode()
            self.state = ProbeState.BOUNDARY_ACTIVE
            return "full"
        if partial_confirmed:
            # This is a distinct boundary generation.  Its retry accounting
            # belongs only to the partial path and cannot be refreshed by more
            # sub-full quiet samples.
            self.last_quiet_boundary_us = source_us
            self.boundary_is_partial = True
            self.partial_requalification_eligible = False
            self.partial_retry_aborts_used = 0
            self.partial_retry_exhausted = False
            self._clear_high_episode()
            self.state = ProbeState.BOUNDARY_ACTIVE
            return "partial"
        return None

    def _start_or_extend_contiguous_high(self, source_us: int) -> bool:
        if self.high_start_us is None:
            self.high_start_us = source_us
            self.high_boundary_source_us = self.last_quiet_boundary_us
            self.high_boundary_is_partial = self.boundary_is_partial
            self.high_observations = 1
            self.state = ProbeState.HIGH_EPISODE
        else:
            self.high_observations += 1
        minimum_observations = self.config.high_min_observations
        minimum_span_s = self.config.high_min_source_span_s
        if self.policy is PolicyKind.PARTIAL_QUIET_PROBATION and self.high_boundary_is_partial:
            minimum_observations = self.config.partial_high_min_observations
            minimum_span_s = self.config.partial_high_min_source_span_s
        if (
            self.high_observations >= minimum_observations
            and _duration_s(source_us, self.high_start_us)
            >= minimum_span_s
        ):
            self.state = ProbeState.LATCHED
            self.latch_source_timestamp_us = source_us
            return True
        return False

    def _advance_graded_high(self, source_us: int, elapsed_s: float) -> bool:
        if self.high_start_us is None:
            self.high_start_us = source_us
            self.high_boundary_source_us = self.last_quiet_boundary_us
            self.high_observations = 0
            # The first high sample defines episode onset.  Time before that
            # sample is not high evidence and must not be credited.
            elapsed_s = 0.0
            self.state = ProbeState.HIGH_EPISODE
        self.high_observations += 1
        self.graded_evidence_s += elapsed_s
        if (
            self.high_observations >= self.config.high_min_observations
            and self.graded_evidence_s >= self.config.high_min_source_span_s
        ):
            self.state = ProbeState.LATCHED
            self.latch_source_timestamp_us = source_us
            return True
        return False

    def _abort_contiguous_high(self, *, is_midband: bool) -> None:
        had_high = self.high_start_us is not None
        high_boundary_is_partial = self.high_boundary_is_partial
        self._clear_high_episode()
        if self.policy is PolicyKind.BOUNDED_RETRY and had_high and is_midband:
            self.retry_aborts_used += 1
            if self.retry_aborts_used > self.config.retry_max_aborts_after_boundary:
                self.retry_exhausted = True
            return
        if self.policy is not PolicyKind.PARTIAL_QUIET_PROBATION or not is_midband:
            return

        if had_high and high_boundary_is_partial:
            self.partial_retry_aborts_used += 1
            if (
                self.partial_retry_aborts_used
                <= self.config.partial_retry_max_aborts_after_boundary
            ):
                # A direct retry is permitted only by this independent, finite
                # budget.  It retains exactly this partial boundary and never
                # creates fresh quiet evidence.
                self.state = ProbeState.BOUNDARY_ACTIVE
                return
            self.partial_retry_exhausted = True

        # A high episode that began from a full boundary, a mid-band without a
        # high episode, or an exhausted partial retry cannot fall back to the
        # historical full boundary.  The next high sample needs new quiet.
        self._retire_boundary_for_partial_requalification()

    def observe(self, item: ProbeInput) -> ProbeEvent:
        """Consume a causal residual event; no labels or truth influence decisions."""

        state_before = self.state
        valid_epoch = (
            not isinstance(item.source_epoch, bool)
            and isinstance(item.source_epoch, int)
            and item.source_epoch >= 0
        )
        valid_timestamps = (
            not isinstance(item.source_timestamp_us, bool)
            and isinstance(item.source_timestamp_us, int)
            and item.source_timestamp_us >= 0
            and not isinstance(item.arrival_timestamp_us, bool)
            and isinstance(item.arrival_timestamp_us, int)
            and item.arrival_timestamp_us >= item.source_timestamp_us
        )
        if not valid_epoch or not valid_timestamps:
            self._reset_for_discontinuity()
            return self._event(item, state_before, "invalid_epoch_or_timestamp_reset")
        if self.state is ProbeState.LATCHED:
            return self._event(item, state_before, "already_latched")
        if self.authorized_epoch is None or item.source_epoch != self.authorized_epoch:
            self._reset_for_discontinuity()
            return self._event(item, state_before, "unauthorized_or_epoch_changed_reset")
        if (
            self.last_source_us is not None
            and (
                item.source_timestamp_us <= self.last_source_us
                or item.arrival_timestamp_us <= self.last_arrival_us  # type: ignore[operator]
            )
        ):
            self._reset_for_discontinuity()
            return self._event(item, state_before, "nonmonotonic_time_reset")

        elapsed_s = 0.0 if self.last_source_us is None else _duration_s(item.source_timestamp_us, self.last_source_us)
        source_gap = elapsed_s > self.config.maximum_source_gap_s
        arrival_gap = (
            self.last_arrival_us is not None
            and _duration_s(item.arrival_timestamp_us, self.last_arrival_us)
            > self.config.maximum_arrival_gap_s
        )
        self.last_source_us = item.source_timestamp_us
        self.last_arrival_us = item.arrival_timestamp_us
        if source_gap or arrival_gap:
            # A transport gap breaks evidence continuity but does not by
            # itself prove that the upstream source epoch changed.  Keep the
            # authorization token, require a fresh warmup/quiet run, and fail
            # closed until that run completes.  Invalid samples and epoch/
            # timestamp violations still revoke authorization below.
            self._reset_for_discontinuity(keep_authorization=True)
            return self._event(item, state_before, "source_or_arrival_gap_reset")
        if (
            not item.source_valid
            or item.nis is None
            or not math.isfinite(item.nis)
            or item.nis < 0.0
        ):
            self._reset_for_discontinuity()
            return self._event(item, state_before, "invalid_or_missing_residual_reset")

        nis = float(item.nis)
        if not self.warmup_ready:
            if not self._advance_warmup(item.source_timestamp_us):
                return self._event(item, state_before, "warmup_coverage")
            # The sample which completes warmup can start quiet confirmation,
            # matching v7's causal order.

        if nis <= self.config.quiet_nis_threshold:
            # A genuinely quiet sample ends any unfinished high episode.  It
            # is new baseline evidence, not an excuse to carry high evidence
            # through a quiet interval.  The boundary is refreshed only after
            # the configured quiet run completes.
            if self.high_start_us is not None:
                if self.policy is PolicyKind.PARTIAL_QUIET_PROBATION:
                    # A quiet termination begins a new qualification run; it
                    # cannot reuse the full/partial boundary that started the
                    # aborted episode.
                    self._retire_boundary_for_partial_requalification()
                self._clear_high_episode()
            boundary_kind = self._advance_quiet(item.source_timestamp_us)
            return self._event(
                item,
                state_before,
                "quiet_boundary_refreshed"
                if boundary_kind == "full"
                else (
                    "partial_quiet_boundary_confirmed"
                    if boundary_kind == "partial"
                    else "quiet_confirmation"
                ),
            )

        # A nonquiet point cannot contribute to a new quiet run.  A previously
        # confirmed boundary remains usable only until its bounded age expires.
        self.quiet_start_us = None
        self.quiet_observations = 0
        # Once a high episode has started on a valid recent boundary, retain
        # that onset authorization until the episode either latches or aborts.
        # The age limit applies again only when starting a new episode/retry.
        episode_in_progress = self.high_start_us is not None
        if not episode_in_progress and not self._boundary_is_recent(item.source_timestamp_us):
            self.last_quiet_boundary_us = None
            self._clear_high_episode()
            return self._event(item, state_before, "no_recent_quiet_boundary")

        if nis >= self.config.high_nis_threshold:
            if self.policy is PolicyKind.BOUNDED_RETRY and self.retry_exhausted:
                return self._event(item, state_before, "retry_budget_exhausted")
            if (
                self.policy is PolicyKind.PARTIAL_QUIET_PROBATION
                and self.partial_retry_exhausted
            ):
                return self._event(item, state_before, "partial_retry_budget_exhausted")
            if self.policy is PolicyKind.GRADED_EVIDENCE:
                latched = self._advance_graded_high(item.source_timestamp_us, elapsed_s)
                return self._event(
                    item,
                    state_before,
                    "graded_evidence_latched" if latched else "graded_evidence_accumulating",
                )
            latched = self._start_or_extend_contiguous_high(item.source_timestamp_us)
            return self._event(
                item,
                state_before,
                "high_episode_latched" if latched else "high_episode_accumulating",
            )

        # Mid-band input is ambiguous.  The first three diagnostic shapes keep
        # their own bounded inheritance rules; partial probation instead retires
        # the current boundary and requires a new quiet run (or its finite
        # partial retry budget when an episode was already admitted).
        if self.policy is PolicyKind.GRADED_EVIDENCE:
            self.graded_evidence_s = max(
                0.0,
                self.graded_evidence_s
                - self.config.graded_midband_decay_per_source_s * elapsed_s,
            )
            if self.graded_evidence_s == 0.0:
                self.high_start_us = None
                self.high_observations = 0
                self.state = ProbeState.BOUNDARY_ACTIVE
            return self._event(item, state_before, "graded_evidence_decayed_midband")
        self._abort_contiguous_high(is_midband=True)
        if self.policy is PolicyKind.PARTIAL_QUIET_PROBATION:
            if self.partial_retry_exhausted:
                reason = "partial_retry_budget_exhausted_midband"
            elif self.last_quiet_boundary_us is not None and self.boundary_is_partial:
                reason = "partial_retry_reserved_midband"
            else:
                reason = "partial_boundary_retired_midband"
            return self._event(item, state_before, reason)
        return self._event(
            item,
            state_before,
            "bounded_retry_reserved_midband"
            if self.policy is PolicyKind.BOUNDED_RETRY
            else "recent_boundary_midband",
        )

    def result(self) -> ProbeResult:
        return ProbeResult(
            policy=self.policy.value,
            final_state=self.state.value,
            latched=self.state is ProbeState.LATCHED,
            latch_source_timestamp_us=self.latch_source_timestamp_us,
            episode_start_source_timestamp_us=self.high_start_us,
            boundary_source_timestamp_us=self.last_quiet_boundary_us,
            boundary_is_partial=self.boundary_is_partial,
            episode_boundary_is_partial=self.high_boundary_is_partial,
            full_boundary_seen=self.full_boundary_seen,
            reset_count=self.reset_count,
            retry_aborts_used=self.retry_aborts_used,
            partial_retry_aborts_used=self.partial_retry_aborts_used,
            partial_retry_exhausted=self.partial_retry_exhausted,
            trace=tuple(self.trace),
        )


def _event_at(source_s: float, nis: float | None, *, valid: bool = True, arrival_offset_s: float = 0.05) -> ProbeInput:
    return ProbeInput(
        source_timestamp_us=_seconds_to_us(source_s),
        arrival_timestamp_us=_seconds_to_us(source_s + arrival_offset_s),
        nis=nis,
        source_valid=valid,
    )


def _quiet_prefix(*, until_s: float, rate_hz: float = 2.0) -> list[ProbeInput]:
    count = int(round(until_s * rate_hz)) + 1
    return [_event_at(index / rate_hz, 1.0) for index in range(count)]


def diagnostic_scenarios() -> dict[str, tuple[ProbeInput, ...]]:
    """Return fixed adversarial traces; only runtime inputs are included."""

    # This mirrors the *shape* of the retained v7 seed 71101 root cause:
    # last quiet -> one mid-band sample -> insufficient fresh quiet -> high.
    # It is hand-authored so the policy semantics remain unit-testable without
    # importing a train seed or a physical-injection label.
    pre_midband = _quiet_prefix(until_s=67.0)
    midband_then_persistent = pre_midband + [
        _event_at(67.5, 2.075),
        _event_at(68.0, 1.0),
        _event_at(68.5, 1.0),
        _event_at(69.0, 10.685),
        _event_at(69.5, 11.371),
        _event_at(70.0, 29.723),
        _event_at(70.5, 12.601),
        _event_at(71.0, 11.461),
    ]

    # No candidate may turn a distant quiet condition into an indefinitely
    # valid antecedent.  The high run starts after the configured age horizon.
    expired_boundary = _quiet_prefix(until_s=4.0) + [
        _event_at(4.5, 3.0),
        _event_at(5.0, 3.0),
        _event_at(5.5, 3.0),
        _event_at(6.0, 3.0),
        _event_at(6.5, 3.0),
        _event_at(8.0, 5.0),
        _event_at(8.5, 5.0),
        _event_at(9.0, 5.0),
        _event_at(9.5, 5.0),
    ]

    # A one-sample mid-band interruption separates the three proposed shapes.
    # Recent-boundary needs a fully new contiguous episode; graded evidence may
    # retain decayed source-time evidence; bounded retry permits one restart.
    intermittent_high = _quiet_prefix(until_s=4.0) + [
        _event_at(4.5, 5.0),
        _event_at(5.0, 5.0),
        _event_at(5.5, 3.0),
        _event_at(6.0, 5.0),
        _event_at(6.5, 5.0),
        _event_at(7.0, 5.0),
        _event_at(7.5, 5.0),
    ]

    # A continuity fault invalidates all historic evidence.  The subsequently
    # high residuals must not latch without a fresh warmup + quiet boundary.
    gap_then_high = _quiet_prefix(until_s=4.0) + [
        _event_at(6.5, 5.0),
        _event_at(7.0, 5.0),
        _event_at(7.5, 5.0),
        _event_at(8.0, 5.0),
    ]

    # This exhausts the retry-only safety budget.  It is not a performance
    # target; it guards against an unlimited sequence of broken episodes.
    retry_exhaustion = _quiet_prefix(until_s=4.0) + [
        _event_at(4.5, 5.0),
        _event_at(5.0, 3.0),
        _event_at(5.5, 5.0),
        _event_at(6.0, 3.0),
        _event_at(6.5, 5.0),
        _event_at(7.0, 5.0),
        _event_at(7.5, 5.0),
        _event_at(8.0, 5.0),
    ]
    return {
        "midband_then_persistent_high": tuple(midband_then_persistent),
        "expired_boundary": tuple(expired_boundary),
        "intermittent_high": tuple(intermittent_high),
        "gap_then_high": tuple(gap_then_high),
        "retry_exhaustion": tuple(retry_exhaustion),
    }


def run_probe(policy: PolicyKind, events: Iterable[ProbeInput], config: ProbeConfig) -> ProbeResult:
    probe = CausalPolicyProbe(policy, config)
    probe.reauthorize(0)
    for event in events:
        probe.observe(event)
    return probe.result()


def _result_payload(result: ProbeResult) -> dict[str, object]:
    return {
        "policy": result.policy,
        "final_state": result.final_state,
        "latched": result.latched,
        "latch_source_timestamp_us": result.latch_source_timestamp_us,
        "episode_start_source_timestamp_us": result.episode_start_source_timestamp_us,
        "boundary_source_timestamp_us": result.boundary_source_timestamp_us,
        "boundary_is_partial": result.boundary_is_partial,
        "episode_boundary_is_partial": result.episode_boundary_is_partial,
        "full_boundary_seen": result.full_boundary_seen,
        "reset_count": result.reset_count,
        "retry_aborts_used": result.retry_aborts_used,
        "partial_retry_aborts_used": result.partial_retry_aborts_used,
        "partial_retry_exhausted": result.partial_retry_exhausted,
        "trace": [asdict(event) for event in result.trace],
    }


def run_handcrafted_diagnostics(
    config: ProbeConfig | None = None,
    *,
    boundary_ages_s: Sequence[float] = (1.0, 2.0, 3.0),
) -> dict[str, object]:
    """Run the fixed, tiny semantic matrix and return an auditable JSON payload."""

    active_config = config or ProbeConfig()
    ages = tuple(float(age) for age in boundary_ages_s)
    if not ages or any(not math.isfinite(age) or age <= 0.0 for age in ages):
        raise ValueError("boundary_ages_s must contain finite positive values")
    policy_results: dict[str, object] = {}
    for age_s in ages:
        age_config = replace(active_config, recent_quiet_boundary_max_age_s=age_s)
        age_key = f"{age_s:g}s"
        policy_results[age_key] = {}
        for scenario_name, inputs in diagnostic_scenarios().items():
            policy_results[age_key][scenario_name] = {
                kind.value: _result_payload(run_probe(kind, inputs, age_config))
                for kind in PolicyKind
            }
    return {
        "schema_version": 1,
        "study_id": "aerakia-v8-policy-shape-focused-diagnostic",
        "status": "completed_handcrafted_development_diagnostic_only",
        "scope": {
            "production_eskf": "unchanged",
            "mahony": "unchanged",
            "public_api": "unchanged",
            "v7_source_protocol_scorer_artifact": "unchanged",
            "runtime_receives_injection_labels_or_truth": False,
        },
        "config": asdict(active_config),
        "boundary_age_sweep": list(ages),
        "scenario_count": len(diagnostic_scenarios()),
        "policy_comparison_count": len(ages) * len(diagnostic_scenarios()) * len(PolicyKind),
        "policies": policy_results,
        "limitations": [
            "Handcrafted traces test state-machine semantics only; they are not a statistical campaign.",
            "No threshold is selected or promoted by this diagnostic.",
            "A latch denotes residual persistence, not TAS-fault, wind, sideslip, timing, GNSS, or model-mismatch classification.",
            "No result authorizes a production ESKF change, source selector, estimator reset, or flight-control action.",
        ],
    }


def _config_from_v7_protocol(protocol: dict[str, object]) -> ProbeConfig:
    """Derive only shared timing semantics for one optional frozen-stream replay."""

    monitor = protocol["monitor"]
    scenario = protocol["scenario"]
    assert isinstance(monitor, dict)
    assert isinstance(scenario, dict)
    rate_hz = float(scenario["monitor_nominal_rate_hz"])
    return ProbeConfig(
        warmup_min_observations=int(
            math.ceil(
                float(monitor["warmup_valid_source_time_s"])
                * rate_hz
                * float(monitor["minimum_warmup_coverage_fraction"])
            )
        ),
        warmup_min_source_span_s=float(monitor["warmup_valid_source_time_s"]),
        quiet_min_observations=2,
        quiet_min_source_span_s=float(monitor["initial_quiet_source_time_s"]),
        quiet_nis_threshold=float(monitor["quiet_nis_threshold"]),
        high_nis_threshold=float(monitor["high_nis_threshold"]),
        high_min_observations=int(monitor["minimum_high_observations"]),
        high_min_source_span_s=float(monitor["minimum_high_episode_source_time_s"]),
        maximum_source_gap_s=max(
            float(monitor["minimum_source_gap_s"]),
            float(monitor["source_gap_periods"]) / rate_hz,
        ),
        maximum_arrival_gap_s=max(
            float(monitor["minimum_arrival_gap_s"]),
            float(monitor["arrival_gap_periods"]) / rate_hz,
        ),
        # Intentionally a design probe value.  Freeze/select it only in a
        # future v8 protocol after this semantic diagnostic is reviewed.
        recent_quiet_boundary_max_age_s=3.0,
        graded_midband_decay_per_source_s=0.5,
        retry_max_aborts_after_boundary=1,
    )


def replay_one_v7_stream(*, seed: int, case_name: str) -> dict[str, object]:
    """Replay one v7-generated NIS stream through the independent probes.

    Imports are local so the tiny handcrafted path has no NumPy/v7 dependency.
    This path does not invoke the v7 campaign's train/tune entry point and
    writes nothing under ``validation/public``.
    """

    validation_dir = ROOT / "validation"
    if str(validation_dir) not in sys.path:
        sys.path.insert(0, str(validation_dir))
    import airspeed_wind_residual_persistence_v7 as v7_monitor  # noqa: PLC0415
    import run_airspeed_wind_residual_persistence_v7 as v7  # noqa: PLC0415

    protocol = v7.load_protocol()
    base_protocol = v7.resolve_base_protocol(protocol)
    case = next(
        (
            item
            for item in protocol["case_matrix"]
            if isinstance(item, dict) and item.get("name") == case_name
        ),
        None,
    )
    if case is None:
        raise ValueError(f"unknown v7 case {case_name!r}")
    stream = v7.generate_case_stream(case, protocol, base_protocol, seed=seed)
    shadow, _ = v7.build_residual_evaluator(protocol, base_protocol)
    original = v7._monitor_from_protocol(protocol, stream.nominal_rate_hz)
    original.reauthorize(0)
    config = _config_from_v7_protocol(protocol)
    probes = {kind: CausalPolicyProbe(kind, config) for kind in PolicyKind}
    for probe in probes.values():
        probe.reauthorize(0)

    feed_count = 0
    for observation in sorted(
        stream.observations,
        key=lambda item: (item.arrival_timestamp_us, item.tas_timestamp_us),
    ):
        geometry_before = v7.evaluator_geometry_before_step(shadow)
        residual_event = shadow.step(observation)
        if not geometry_before:
            continue
        feed_count += 1
        valid = residual_event.innovation_nis is not None
        item = ProbeInput(
            source_timestamp_us=observation.tas_timestamp_us,
            arrival_timestamp_us=observation.arrival_timestamp_us,
            nis=None if residual_event.innovation_nis is None else float(residual_event.innovation_nis),
            source_valid=valid,
            source_epoch=0,
        )
        original.observe(
            source_epoch=0,
            source_timestamp_us=item.source_timestamp_us,
            arrival_timestamp_us=item.arrival_timestamp_us,
            nis=item.nis,
            source_valid=item.source_valid,
        )
        for probe in probes.values():
            probe.observe(item)

    def delay(latch_us: int | None) -> float | None:
        if latch_us is None:
            return None
        return _duration_s(latch_us, stream.injection_start_source_timestamp_us)

    original_latch = (
        None if original.trigger_snapshot is None else original.trigger_snapshot.latch_source_timestamp_us
    )
    return {
        "kind": "single_existing_v7_stream_read_only_replay",
        "seed": seed,
        "case": case_name,
        "runtime_receives_injection_labels_or_truth": False,
        "offline_injection_start_source_timestamp_us": stream.injection_start_source_timestamp_us,
        "monitor_fed_observations": feed_count,
        "v7_reference": {
            "latched": original.state is v7_monitor.MonitorState.LATCHED,
            "latch_source_timestamp_us": original_latch,
            "source_detection_delay_s": delay(original_latch),
            "final_state": original.state.value,
        },
        "candidate_config": asdict(config),
        "candidate_policies": {
            kind.value: {
                **_result_payload(probe.result()),
                "source_detection_delay_s": delay(probe.latch_source_timestamp_us),
            }
            for kind, probe in probes.items()
        },
        "limitations": [
            "This is one historical development stream used for root-cause replay, not a new v8 train/tune result.",
            "The injected-step timestamp is used only to report an offline delay after the causal policies have run.",
            "Do not select, tune, or promote a policy from this one-stream replay.",
        ],
    }


def _screen_one_v7_stream(
    *, seed: int, case_name: str, boundary_ages_s: Sequence[float], protocol_path: str,
) -> dict[str, object]:
    """Evaluate one fresh seed/case against v7 and all unselected probes."""

    validation_dir = ROOT / "validation"
    if str(validation_dir) not in sys.path:
        sys.path.insert(0, str(validation_dir))
    import airspeed_wind_residual_persistence_v7 as v7_monitor  # noqa: PLC0415
    import run_airspeed_wind_residual_persistence_v7 as v7  # noqa: PLC0415

    protocol_path_obj = Path(protocol_path)
    protocol = v7.load_protocol(protocol_path_obj)
    base_protocol = v7.resolve_base_protocol(protocol)
    case = next(
        item for item in protocol["case_matrix"]
        if isinstance(item, dict) and item.get("name") == case_name
    )
    stream = v7.generate_case_stream(case, protocol, base_protocol, seed=seed)
    shadow, evaluator_sha256 = v7.build_residual_evaluator(protocol, base_protocol)
    original = v7._monitor_from_protocol(protocol, stream.nominal_rate_hz)
    original.reauthorize(0)
    configs = {
        float(age): _config_from_v7_protocol(protocol)
        for age in boundary_ages_s
    }
    probes = {
        (float(age), kind): CausalPolicyProbe(
            kind,
            replace(config, recent_quiet_boundary_max_age_s=float(age)),
        )
        for age, config in configs.items()
        for kind in PolicyKind
    }
    for probe in probes.values():
        probe.reauthorize(0)
    feed_count = 0
    for observation in sorted(
        stream.observations,
        key=lambda item: (item.arrival_timestamp_us, item.tas_timestamp_us),
    ):
        geometry_before = v7.evaluator_geometry_before_step(shadow)
        residual_event = shadow.step(observation)
        if not geometry_before:
            continue
        feed_count += 1
        item = ProbeInput(
            source_timestamp_us=observation.tas_timestamp_us,
            arrival_timestamp_us=observation.arrival_timestamp_us,
            nis=None if residual_event.innovation_nis is None else float(residual_event.innovation_nis),
            source_valid=residual_event.innovation_nis is not None,
            source_epoch=0,
        )
        original.observe(
            source_epoch=0,
            source_timestamp_us=item.source_timestamp_us,
            arrival_timestamp_us=item.arrival_timestamp_us,
            nis=item.nis,
            source_valid=item.source_valid,
        )
        for probe in probes.values():
            probe.observe(item)

    def delay(latch_us: int | None) -> float | None:
        if latch_us is None:
            return None
        return _duration_s(latch_us, stream.injection_start_source_timestamp_us)

    original_latch = (
        None if original.trigger_snapshot is None else original.trigger_snapshot.latch_source_timestamp_us
    )
    original_payload = {
        "latched": original.state is v7_monitor.MonitorState.LATCHED,
        "latch_source_timestamp_us": original_latch,
        "source_detection_delay_s": delay(original_latch),
        "final_state": original.state.value,
    }
    candidate_payload: dict[str, object] = {}
    for (age, kind), probe in probes.items():
        result = probe.result()
        candidate_payload.setdefault(f"{age:g}s", {})[kind.value] = {
            **_result_payload(result),
            "source_detection_delay_s": delay(result.latch_source_timestamp_us),
        }
    return {
        "seed": seed,
        "case": case_name,
        "role": case["role"],
        "injection_kind": case["injection"]["kind"],
        "injection_start_source_timestamp_us": stream.injection_start_source_timestamp_us,
        "monitor_fed_observations": feed_count,
        "random_stream_sha256": stream.random_stream_sha256,
        "arrival_schedule_sha256": stream.arrival_schedule_sha256,
        "residual_evaluator_sha256": evaluator_sha256,
        "runtime_receives_injection_labels_or_truth": False,
        "v7_reference": original_payload,
        "candidate_policies": candidate_payload,
    }


def _screen_task(task: tuple[int, str, tuple[float, ...], str]) -> dict[str, object]:
    seed, case_name, ages, protocol_path = task
    return _screen_one_v7_stream(
        seed=seed,
        case_name=case_name,
        boundary_ages_s=ages,
        protocol_path=protocol_path,
    )


def run_fresh_screen(
    *,
    seed_start: int = 74101,
    seed_count: int = 16,
    jobs: int = 1,
    boundary_ages_s: Sequence[float] = (1.0, 2.0, 3.0),
    case_names: Sequence[str] = (
        "nominal",
        "nominal_high_noise_calibrated",
        "persistent_tas_offset_positive",
        "persistent_tas_offset_negative",
        "persistent_tas_offset_positive_bounded_jitter",
        "persistent_tas_offset_negative_bounded_jitter",
        "persistent_tas_scale_positive",
        "persistent_tas_scale_negative",
        "persistent_horizontal_wind_step",
        "persistent_vertical_wind",
        "tas_offset_pulse_0p25s",
        "tas_offset_pulse_0p5s",
        "tas_offset_pulse_1p0s",
        "tas_offset_pulse_1p5s",
        "tas_offset_pulse_2p0s",
        "gap_then_tas_offset_pulse_1p0s",
    ),
) -> dict[str, object]:
    """Run a fresh, non-promoting development screen on disjoint seeds."""

    if seed_start < 0 or seed_count <= 0 or jobs <= 0:
        raise ValueError("seed_start, seed_count, and jobs must be positive where applicable")
    ages = tuple(float(age) for age in boundary_ages_s)
    if not ages or any(not math.isfinite(age) or age <= 0.0 for age in ages):
        raise ValueError("boundary_ages_s must contain finite positive values")
    protocol_path = ROOT / "validation" / "airspeed_wind_residual_persistence_protocol_v7.json"
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    available = {str(item["name"]): item for item in protocol["case_matrix"]}
    selected = [str(name) for name in case_names]
    if len(set(selected)) != len(selected) or any(name not in available for name in selected):
        raise ValueError("screen case list contains unknown or duplicate case names")
    seeds = list(range(seed_start, seed_start + seed_count))
    tasks = [(seed, name, ages, str(protocol_path)) for seed in seeds for name in selected]
    if jobs == 1:
        records = [_screen_task(task) for task in tasks]
    else:
        methods = multiprocessing.get_all_start_methods()
        context = multiprocessing.get_context("fork") if "fork" in methods else None
        with ProcessPoolExecutor(max_workers=jobs, mp_context=context) as executor:
            records = list(executor.map(_screen_task, tasks))
    records.sort(key=lambda item: (str(item["case"]), int(item["seed"])))
    by_key = {(int(item["seed"]), str(item["case"])): item for item in records}

    persistent_names = [
        "persistent_tas_offset_positive",
        "persistent_tas_offset_negative",
        "persistent_tas_offset_positive_bounded_jitter",
        "persistent_tas_offset_negative_bounded_jitter",
    ]
    pulse_names = ["tas_offset_pulse_1p0s", "tas_offset_pulse_1p5s", "tas_offset_pulse_2p0s"]
    descriptive_names = [
        "persistent_tas_scale_positive",
        "persistent_tas_scale_negative",
        "persistent_horizontal_wind_step",
        "persistent_vertical_wind",
    ]
    summaries: dict[str, object] = {}
    for age in ages:
        age_key = f"{age:g}s"
        summaries[age_key] = {}
        for kind in PolicyKind:
            nominal = [
                by_key[(seed, "nominal")]["candidate_policies"][age_key][kind.value]
                for seed in seeds
            ]
            persistent = [
                by_key[(seed, name)]["candidate_policies"][age_key][kind.value]
                for seed in seeds for name in persistent_names
            ]
            family_ok = 0
            family_failures: list[int] = []
            for seed in seeds:
                ok = True
                for name in persistent_names:
                    item = by_key[(seed, name)]["candidate_policies"][age_key][kind.value]
                    case = available[name]
                    delay = item["source_detection_delay_s"]
                    episode_start = item["episode_start_source_timestamp_us"]
                    injection = by_key[(seed, name)]["injection_start_source_timestamp_us"]
                    if (
                        not item["latched"]
                        or delay is None
                        or float(delay) > float(case.get("maximum_source_detection_delay_s", 6.0))
                        or episode_start is None
                        or int(episode_start) < int(injection)
                    ):
                        ok = False
                if ok:
                    family_ok += 1
                else:
                    family_failures.append(seed)
            pulse = [
                by_key[(seed, name)]["candidate_policies"][age_key][kind.value]
                for seed in seeds for name in pulse_names
            ]
            summaries[age_key][kind.value] = {
                "nominal_false_latches": sum(bool(item["latched"]) for item in nominal),
                "nominal_trials": len(nominal),
                "persistent_member_latches": sum(bool(item["latched"]) for item in persistent),
                "persistent_member_trials": len(persistent),
                "persistent_family_passes": family_ok,
                "persistent_family_trials": len(seeds),
                "persistent_family_failures": family_failures,
                "pulse_latches_by_case": {
                    name: sum(
                        bool(by_key[(seed, name)]["candidate_policies"][age_key][kind.value]["latched"])
                        for seed in seeds
                    )
                    for name in pulse_names
                },
                "descriptive_latches_by_case": {
                    name: sum(
                        bool(by_key[(seed, name)]["candidate_policies"][age_key][kind.value]["latched"])
                        for seed in seeds
                    )
                    for name in descriptive_names
                },
                "high_noise_nominal_latches": sum(
                    bool(
                        by_key[(seed, "nominal_high_noise_calibrated")]["candidate_policies"][age_key][kind.value]["latched"]
                    )
                    for seed in seeds
                ),
                "gap_structural_latches": sum(
                    bool(
                        by_key[(seed, "gap_then_tas_offset_pulse_1p0s")]["candidate_policies"][age_key][kind.value]["latched"]
                    )
                    for seed in seeds
                ),
            }
    manifest = {
        "seed_start": seed_start,
        "seed_count": seed_count,
        "seed_end": seed_start + seed_count - 1,
        "case_names": selected,
        "boundary_ages_s": list(ages),
        "record_count": len(records),
        "protocol_path": str(protocol_path.relative_to(ROOT)),
        "protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
    }
    return {
        "schema_version": 1,
        "study_id": "aerakia-v8-fresh-development-screen",
        "status": "completed_non_promoting_development_screen",
        "manifest": manifest,
        "summary": summaries,
        "records": records,
        "limitations": [
            "This is a fresh development screen, not a frozen v8 train, tune, or holdout.",
            "The v7 residual oracle is reused only as a host-side synthetic NIS provider.",
            "No candidate threshold, policy, estimator state, or FCOne source authority is promoted.",
            "Fresh seeds are disjoint from the v7 train/tune ranges but are not a protected holdout.",
        ],
    }


def build_report(
    *,
    replay_seed: int | None,
    replay_case: str,
    boundary_ages_s: Sequence[float] = (1.0, 2.0, 3.0),
    screen_seed_start: int | None = None,
    screen_seed_count: int = 16,
    screen_jobs: int = 1,
) -> dict[str, object]:
    report = run_handcrafted_diagnostics(boundary_ages_s=boundary_ages_s)
    if replay_seed is not None:
        report["optional_frozen_stream_replay"] = replay_one_v7_stream(
            seed=replay_seed,
            case_name=replay_case,
        )
    if screen_seed_start is not None:
        report["fresh_development_screen"] = run_fresh_screen(
            seed_start=screen_seed_start,
            seed_count=screen_seed_count,
            jobs=screen_jobs,
            boundary_ages_s=boundary_ages_s,
        )
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replay-v7-seed",
        type=int,
        default=None,
        help="read-only replay of one existing v7 stream; omit for handcrafted diagnostics only",
    )
    parser.add_argument(
        "--replay-v7-case",
        default="persistent_tas_offset_positive",
        help="existing v7 case name used only with --replay-v7-seed",
    )
    parser.add_argument(
        "--boundary-ages",
        default="1,2,3",
        help="comma-separated recent quiet-boundary ages in seconds (focused comparison only)",
    )
    parser.add_argument(
        "--screen-seed-start",
        type=int,
        default=None,
        help="start a fresh non-promoting v8 development screen at this disjoint seed",
    )
    parser.add_argument(
        "--screen-seed-count",
        type=int,
        default=16,
        help="number of fresh development-screen seeds (default: 16)",
    )
    parser.add_argument(
        "--screen-jobs",
        type=int,
        default=1,
        help="parallel workers for the optional fresh screen (default: 1)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="optional JSON output path; no validation/public artifact is written by default",
    )
    args = parser.parse_args(argv)
    try:
        boundary_ages = tuple(float(item.strip()) for item in str(args.boundary_ages).split(","))
    except ValueError as error:
        parser.error(f"invalid --boundary-ages: {error}")
    report = build_report(
        replay_seed=args.replay_v7_seed,
        replay_case=args.replay_v7_case,
        boundary_ages_s=boundary_ages,
        screen_seed_start=args.screen_seed_start,
        screen_seed_count=args.screen_seed_count,
        screen_jobs=args.screen_jobs,
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.out is None:
        print(payload, end="")
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload, encoding="utf-8")
        print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
