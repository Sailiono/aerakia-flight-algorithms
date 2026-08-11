# v8 residual-persistence failure analysis

This note records why the current development candidates still miss some
persistent synthetic cases. It is a diagnosis of the host-only monitor probe,
not a change to the production ESKF, Mahony, FCOne adapter, or v7 artifact.

## Evidence used

Two disjoint development screens are available:

- `74101--74132`: 32 families, 512 replays;
- `74201--74216`: 16 families, 256 replays, after the partial-policy safety
  tightening.

Both screens use the frozen v7 synthetic residual oracle only as a host-side
NIS provider. The policy receives source/arrival time, validity, epoch, and NIS;
it never receives injection labels or truth.

## Observed failure mechanism

At recent-boundary age `3 s`, the graded candidate missed complete persistent
families `74108`, `74119`, `74124`, and `74132` in the first screen. In those
traces, a valid quiet boundary exists earlier, but the residual stream has a
short high/mid/quiet sequence before the injected persistent high episode. The
last boundary then ages out. Once the persistent high run begins, the causal
input contains no new quiet evidence from which the monitor can establish a
post-event boundary, so it remains `UNQUALIFIED` and emits
`no_recent_quiet_boundary`.

This is not a numerical instability, a timestamp pairing error, or a scorer
crash. It is an information limitation:

> From NIS and timing alone, a high episode that begins while the monitor is
> unqualified cannot be distinguished from a pre-existing anomaly without
> importing the offline injection boundary.

Seed `74123` demonstrates the complementary attribution problem. Its high
episode begins before the offline injection boundary and later latches. The
later latch must remain an ambiguity failure; crediting it as a clean detection
would leak truth into the causal score.

## What the partial candidate proved

`partial_quiet_probation` was tightened before screening:

1. a complete quiet boundary must occur first;
2. degradation retires the current full/partial boundary;
3. a new partial boundary requires post-degradation quiet evidence;
4. partial high episodes have an independent finite retry budget.

The safety contract passes focused tests, but sensitivity falls to `5/16`
clean families at ages `1`, `2`, and `3 s` in the follow-up screen. This is a
valid negative result. Removing any of the four constraints would reintroduce
the exact pre-onset inheritance risk that v7 exposed.

## Decision

Do not:

- increase the boundary age until the misses disappear;
- allow an unqualified high run to become a control-authority fault;
- count a post-injection latch whose episode began before injection;
- promote any current candidate to FCOne or the public runtime API.

The descriptive pooled result for graded evidence at age `3 s` is `43/48`
clean families (`27/32` in the first screen plus `16/16` in the follow-up), or
about `89.6%`. This is below the provisional `95%` development target and is
not a promotion statistic because the protocol and holdout were not frozen.

## Next experiment: causal regime re-anchor

The next candidate should add an independently observable, causal regime
transition rather than more historical quiet inheritance. Suitable inputs must
be defined before implementation, for example:

- a validated air-data/GNSS pairing or flight-phase transition;
- a source-validity transition with bounded age and monotonic timestamps;
- an externally qualified estimator reset/arming epoch.

The candidate may open a **diagnostic anomaly episode** after such a transition,
but a **control-qualified fault** still requires fresh high evidence and a
separate supervisor gate. If no independent transition is available, the
correct output is `UNQUALIFIED_PERSISTENT_RESIDUAL`, not a fault claim.

The next protocol must score these two products separately:

| Output | Allowed evidence | Use |
| --- | --- | --- |
| Diagnostic anomaly | causal residual persistence, even after loss of quiet boundary | logging, post-flight analysis, experiment triage |
| Control-qualified fault | fresh causal onset plus independent regime/source evidence | private FCOne supervisor only, after hardware validation |

This preserves the safety boundary while allowing the diagnostic tool to report
the cases that NIS alone can observe.

## Diagnostic-lane screen result

The proposed separation was implemented as a host-only orthogonal diagnostic
lane and screened on fresh seeds `74301--74364` (`1,088` replays). It did not
modify the normal policy latch or give the diagnostic state any authority.

At `graded_evidence @ 3 s`, the raw `234/256` persistent-member latches contain
`10` pre-existing ambiguous episodes and therefore reduce to `224/256` clean
control members. The diagnostic lane adds `14` clean post-injection members,
giving a descriptive union of `238/256`. At family level it raises complete
coverage from `55/64` to `58/64`, not to the provisional `>=95%` target.
Nominal, bounded-jitter nominal, calibrated-high-noise nominal, and structural-
gap diagnostic counts are all zero in this screen.

The six unresolved families are `74306`, `74321`, `74349`, `74352`, `74354`,
and `74362`. Three complete families (`74304`, `74335`, `74344`) are rescued
only in the diagnostic/descriptive lane. This confirms the intended split:
the new lane preserves evidence that the primary path must reject, but cannot
turn the information limit into a control-qualified fault.

The machine-readable audit is
`validation/public/airspeed_wind_residual_persistence_v8_diagnostic_lane_screen_74301_64_audit.json`.
Its `descriptive_union_clean` field must never be read as control authority.

### Provenance-retention correction

Review of the six unresolved families showed a second, diagnostic-only
mechanism. When a stale mid-band point caused the normal lane to retire its
boundary before the later high episode, the diagnostic lane also erased the
last full-boundary timestamp. In cases such as `74354`, all 24 post-injection
samples were high NIS, yet no diagnostic snapshot could start. Retaining that
timestamp as non-authoritative provenance fixes the omission without changing
normal qualification.

An exact paired replay verified `13,056/13,056` primary comparator invariants.
At `graded_evidence @ 3 s`, the clean descriptive union becomes `246/256`
members and `61/64` families. The only remaining misses are the ten members in
families `74306`, `74349`, and `74352` whose episodes began before injection.
They correctly remain ambiguous. This is the maximum clean descriptive
coverage available on that opened screen without using truth in the runtime.

The paired replay is retained at
`validation/public/airspeed_wind_residual_persistence_v8_diagnostic_provenance_fix_paired_replay.json`.
It is not fresh validation and does not authorize the diagnostic lane.

## Reproduction references

- Full first screen: `build/airspeed_wind_residual_persistence_v8_screen_74101_32.json`
- Full follow-up screen: `build/airspeed_wind_residual_persistence_v8_screen_74201_16.json`
- Compact follow-up summary:
  `validation/public/airspeed_wind_residual_persistence_v8_screen_74201_16_summary.json`
- Policy implementation:
  `validation/diagnose_airspeed_wind_residual_persistence_v8.py`
