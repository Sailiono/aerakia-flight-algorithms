# Barometer source supervision study

This study asks two separate questions:

1. How much does relative barometric height help during a GNSS outage?
2. What additional supervision is required before that aiding can be trusted?

It is a hardware-free synthetic study. It does not qualify FCOne pressure conversion, thermal
behavior, installation, pitot/static plumbing, redundant sensors, or flight-control handoff.

## Method

The paired generator uses identical IMU, GNSS truth/noise, vertical motion, and outage windows for
every arm. It covers `5/10/30/60/120 s` outage durations and nominal, constant datum bias, random
walk, weather step, frozen output, and delayed physical timestamps. The current development smoke
uses a bounded `4 m` climb/descent profile, 100 Hz IMU, 20 Hz barometer, and 10 Hz GNSS outside the
outage.

The four diagnostic arms are:

| Arm | Meaning |
| --- | --- |
| IMU only | no barometric height; degraded-navigation baseline |
| Raw barometer | current ESKF scalar height update and 1D NIS gate |
| Supervised barometer | causal timestamp, increment, quantization-aware freeze, fault-latch, externally authorized return-to-baseline recovery, and two-stage fused-baseline commit |
| Shadow failover | fail-closed offline output mux to a continuously running barometer-free ESKF at first qualified latch |

The last arm is an upper-bound architecture experiment. It switches only `p_D`, `v_D`, and the
health flag after timestamp/configuration/provenance/health checks. It does not switch the complete
nominal state or covariance and proves neither real-time scheduling nor FCOne controller reset
handling.

## Instrumentation repair

The first delay injection shifted values but retained the current IMU timestamp, so it did not
exercise the adapter's age gate. The replay contract now optionally carries `baro_timestamp_us`,
and the runner exports physical age, adapter status, pre-gate innovation, variance, NIS, test ratio,
source-supervisor decision, fault flags, latch, and probation. A `0.60 s` delayed stream is now
rejected 100% before fusion by the existing `0.5 s` ESKF aiding-age gate.

A rejected duplicate, stale, future, non-finite, invalid-variance, or supervisor-filtered call now
also clears the per-call `barometer_accepted` flag instead of leaving the preceding result visible.
The fused residual baseline advances only after the ESKF also accepts the update; a supervisor pass
followed by core NIS rejection cannot become the next accepted residual baseline. Timestamp,
freeze-anchor, and consecutive-fault classification history advances during `evaluate()` even when
the core later rejects the update. Latch/fault/probation state is retained on intervening IMU rows
rather than appearing as a 20 Hz pulse.

## Retained results

The first 2-seed, 5/30-second smoke established the positive and negative boundaries:

| Fault, 30 s outage | IMU-only vertical RMSE | Raw barometer RMSE |
| --- | ---: | ---: |
| Nominal | `0.4695 m` | `0.1004 m` |
| Constant uncalibrated datum bias | `0.4695 m` | `1.0155 m` |
| Weather step | `0.4695 m` | `1.3552 m` |
| Frozen output | `0.4695 m` | `2.9543 m` |
| `0.60 s` physical delay | `0.4695 m` | `0.4695 m`, 100% stale rejection |

Numerical health remained 100% in every group. It therefore means finite state/covariance, not
barometer-source trust.

The current `3 sigma` source supervisor preserves nominal results and detects exact frozen output in
about `0.15 s`, but detection alone is insufficient. In the reviewed 30-second freeze trial, three
bad measurements passed before latch and changed `p_D`, `v_D`, `b_az`, and covariance through ESKF
cross terms. Stopping later barometer fusion did not undo that state contamination.

The offline hot-shadow mux demonstrates the required architecture direction:

| Fault, 30 s outage | Supervised single lane | Shadow failover | IMU-only |
| --- | ---: | ---: | ---: |
| Frozen output | `1.8075 m` | `0.4600 m` | `0.4695 m` |
| Weather step | `0.4522 m` | `0.4348 m` | `0.4695 m` |

This final smoke used two seeds and is not promotion evidence. It was regenerated from clean commit
`dcc002c1317a9eb3fac4a1f1f60126dfc14797b0`; the summary SHA-256 is
`94b7f9e20a37787e074e8057f1c3470a4302a2f80df24fe7dbc1ad5c0777cbce`, and all 12 shadow
input/command audits passed. Constant datum bias remained undetected and retained the approximately
`1.02 m` failure because a single relative-height source cannot
distinguish an unknown datum offset from real height without an independent vertical reference.

Freeze recovery may use sequential normal samples after the original source value changes again.
A jump/datum latch is different: it cannot silently adopt a new pressure datum. Recovery requires
an explicit authorization bound to source identity, generation, quality snapshot, and time, backed
by an independent GNSS/visual/range/redundant-baro reference in the application layer. Declared
The authorization currently permits probation only after the source returns or is externally realigned
to the old residual basin; it carries no bounded rebase value and cannot adopt a persistent new
datum. The four-arm campaign does not inject a source-generation reset or call this authorization
path end to end; that behavior is C-unit-tested only. Declared measurement quantization is included
in freeze detection, and 18 slow climb/descent combinations
at `0.5/2/5/10 m/s` provide negative controls against repeated quantized samples.

## Rejected threshold experiment

An exploratory `2 sigma` jump threshold was tested on 10 train seeds, 5/30-second outages, and
nominal, weather-step, and freeze groups: 60 trials. It detected and switched on all 20 weather-step
trials, but also switched on all 20 nominal trials. Thirty-second nominal supervised barometer
acceptance fell to about 25%. The threshold was rejected and restored to `3 sigma`.

This result rules out further blind tuning of one instantaneous residual threshold as the closure
path. The next candidate must use a pre-registered sequential test, an independent GNSS/visual/
range datum while available, redundant barometer voting, or the hot-shadow architecture.

## FCOne architecture implication

The public repository can validate source diagnostics and multi-lane state behavior. The private
FCOne supervisor must own physical source identity, redundant voting, active-lane selection, and
controller reset deltas. A practical first layout is:

```text
primary lane: IMU + selected barometer + GNSS/heading
shadow lane:  same IMU + GNSS/heading, no selected barometer
```

After a source latch, output may move to the synchronized healthy shadow only as a complete nominal
state and covariance. States or covariances must not be blended. Position, velocity, and attitude
reset deltas, source generation, and reset counter must be published to downstream control code.
The current offline mux explicitly reports `shadow_full_state_verified=false` and
`shadow_reset_semantics_verified=false`; its RMSE is an architectural upper bound, not evidence that
this private handoff exists.

A separate [multi-lane handoff contract](barometer-multilane-handoff-contract.md)
now closes the host-only transaction prerequisite: three deterministic IMU-rate
paths copy the complete synchronized shadow `AerakiaEskf` image after a real
freeze latch and block timestamp, provenance, configuration, source-generation,
alignment, health, non-finite, duplicate-switch, and automatic-return failures.
It does **not** retroactively turn this Python output mux into a complete-state
handoff or a flight-control implementation; this study's existing RMSE results
remain an offline architecture diagnostic.

## Current capability boundary

- Nominal barometric height materially improves vertical position during synthetic 5--30 second
  GNSS outages.
- Physical timestamp delay is safely rejected when the true sample timestamp is preserved.
- Fast freeze detection and latch exist, but a single contaminated lane is not fault tolerant.
- Persistent datum bias and slow weather drift remain unobservable without another height source or
  a declared pressure-datum model.
- Barometric position updates do not independently qualify vertical velocity.
- No result here supports indefinite GNSS-independent navigation or flight readiness.

Before promotion, the four-arm campaign must cover all planned outage durations, all fault classes,
at least 100 frozen seeds per development split, valid hover/climb/descent/transition negative
controls, train/tune separation, and a newly sealed holdout. Real FCOne thresholds remain blocked on
physical sensor data.

## Resumable release-candidate runner

The broad synthetic matrix is now run by
`validation/run_baro_outage_campaign.py`. It shards exact, deterministic
fault/outage/seed combinations, records the native runner and source hashes,
and invokes a fail-closed merger which rejects missing or duplicated matrix
cells. The replay runner's `--compact-output` mode exports only the fields used
by this study; it does not change estimator execution, fault injection, or
scoring. A one-seed 120 s weather-step smoke completed in 18.34 s after this
I/O change, versus 31.96 s with the old full diagnostic export. Its four-arm
result retained the expected architecture boundary: raw barometer vertical
RMSE was `1.366 m`, supervised single-lane RMSE was `7.845 m`, IMU-only was
`6.689 m`, and offline shadow output-mux RMSE was `6.670 m`.

This smoke is a performance and provenance check, not a release gate. No
threshold or estimator parameter was changed from it.

## Complete Development Matrix (v1)

The complete v1 development matrix is now retained as
[`validation/public/barometer_outage_campaign_v1.json`](../validation/public/barometer_outage_campaign_v1.json).
It contains `600/600` exact, non-duplicated trials: six fault profiles,
five GNSS-outage durations (`5/10/30/60/120 s`), and 20 deterministic seeds
per cell. The compact record carries SHA-256 identities for the merged source
summary, campaign manifest, C99 runner, generator, supervisor, and shard
script. The full 360-shard output is deliberately disposable.

Every arm retained finite state/covariance and a health ratio of `1.0` in this
synthetic matrix. That is a numerical-integrity result, not a source-trust or
flight-readiness result.

| Representative cell | IMU only | Raw baro | Supervised lane | Offline partial shadow mux | What the number does and does not show |
| --- | ---: | ---: | ---: | ---: | --- |
| Nominal, 120 s | `41.6243 m` | `0.1008 m` | `0.1014 m` | `1.0460 m` | Relative height is highly valuable when its source is healthy. One late in-window false latch makes the diagnostic mux worse; it is not a valid handoff design. |
| Physical `0.60 s` delay, 120 s | `41.6243 m` | `41.6243 m` | `41.6243 m` | `41.6243 m` | The timestamp-age gate rejected every delayed barometer update before fusion. |
| Weather step, 30 s | `1.9690 m` | `1.3474 m` | `0.5071 m` | `1.7818 m` | The single lane can reduce this particular transient, but its shadow output cannot be read as a complete-state failover result. |
| Weather step, 120 s | `41.6243 m` | `1.3764 m` | `14.9062 m` | `32.4256 m` | A later latch leaves a contaminated active lane and a diverged barometer-free lane; neither is a safe automated handoff. |
| Frozen stream, 120 s | `41.6243 m` | `1.1825 m` | `7.3938 m` | `41.6029 m` | The synthetic cyclic vertical path can make a stale held value look numerically close. This is exactly why a raw RMSE alone cannot certify a source. |

The scoring counts a shadow transition only on a `fault_latched` rising edge
inside the declared GNSS-outage window. A prior latch is retained as a separate
diagnostic event; it is not retrospectively counted as detection at the outage
boundary. In this matrix there were 191 latch-event trials, 183 with an event
inside the outage. The two nominal/random-walk 120-second cases with a late
in-window latch are retained as false-positive evidence, not discarded.

### Decision after v1

This closes the **diagnostic campaign**, not the barometer fault-tolerance
problem. The current public supervisor remains experimental and must not be
promoted as an FCOne control-authority gate. A viable private FCOne solution
needs source identity, at least a second independently qualified vertical
reference (redundant barometer, GNSS/RTK, range/vision where applicable), and
a complete nominal-state/covariance/controller-reset handoff. It must then be
tested against a sealed split with actual sensor pressure, temperature,
vibration, plumbing, and source-reset behavior.
