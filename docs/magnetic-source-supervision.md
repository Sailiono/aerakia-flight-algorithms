# Magnetic source supervision

## Purpose

The current ESKF magnetometer measurement is intentionally a tilt-conditioned, horizontal NED-yaw
pseudo observation. It is not a three-dimensional magnetic-vector update: inclination and the
magnetic reference Down component do not enter its residual or Jacobian. This protects roll and
pitch from an uncertain field model, but it means that passing the current pseudo-NIS only says that
the yaw pseudo measurement is self-consistent with the filter covariance; it is not proof that the
physical field is trustworthy.

`INSANE indoor_1` exposed that distinction. The paired physical replay holds all inputs equal except
the magnetometer validity/update bits. On the frozen historical holdout, all 8,776 physical magnetic
updates were accepted, yet ESKF yaw RMSE changed from `0.270 deg` with magnetometer disabled to
`12.528 deg` with it enabled. The existing magnitude EMA gate cannot recognize slow or
equal-magnitude direction changes.

The governed procedure is machine-readable in
[`magnetic_supervision_protocol_v1.json`](../validation/magnetic_supervision_protocol_v1.json).
This document describes why that protocol precedes any runtime source gate.

## Corrected diagnosis

It would be incorrect to attribute this failure to a missing magnetic-reference Down component.
With the current horizontal `atan2(E, N)` observation, reference vectors with the same North/East
ratio have exactly the same update. A complete three-dimensional reference remains useful for
offline field diagnostics and a future experimental 3D model, but adding it alone cannot repair the
current product candidate.

The evidence instead supports a narrower statement: the source or its local datum is untrustworthy
in these intervals, but the existing protection accepts it. Possible contributors remain physical
environment or current, hard/soft-iron or installation residual, time/frame mismatch, and ESKF
state/covariance coupling after a bad yaw observation. The evidence does not yet identify a single
physical cause.

## Offline evidence only

`validation/analyze_magnetic_source_ab.py` is an offline truth-scored diagnostic. It rotates raw
body magnetic samples with an independent reference quaternion to measure physical heading relative
to the declared datum, field norm, and inclination. It then compares paired magnetometer-on and
magnetometer-off ESKF outputs.

The reference quaternion is never input to the runtime estimator or an allowed future source-quality
decision. The tool refuses mismatched timestamps, changing declared datum, unpaired magnetometer
flags, missing physical updates, and non-finite measurements.

On the rebuilt disjoint `indoor_1` windows:

| Window | Physical updates | All accepted | Mag on yaw / tilt RMSE | Mag off yaw / tilt RMSE | Physical datum residual P95 abs |
| --- | ---: | ---: | ---: | ---: | ---: |
| calibration `[10,110) s` | 8,788 | 100% | `6.248 / 12.279 deg` | `0.298 / 0.608 deg` | `16.234 deg` |
| development `[110,210) s` | 8,782 | 100% | `10.879 / 6.317 deg` | `0.250 / 0.615 deg` | `18.636 deg` |

In the development interval, the predeclared `>=15 deg` physical-datum-residual bin contains 977
updates. Its magnetometer-on yaw/tilt RMSE is `15.532/8.645 deg`, versus `0.218/0.630 deg` with
magnetometer disabled. This is strong evidence that accepted updates can be harmful. It is not
evidence that a truth-driven runtime threshold is permitted.

Generated reports remain under `build/` because they include raw-data-derived provenance hashes and
are regenerated locally:

```bash
python3 validation/analyze_magnetic_source_ab.py \
  <replay.csv> <magnetometer-on-results.csv> <magnetometer-off-results.csv> \
  --out-dir <output-directory>
```

## Causal feature screening

Two causal candidate features were screened before any runtime policy was written. The results are
negative for this specific slow magnetic-source failure and are retained to prevent an attractive but
unsupported fix from entering the estimator.

| Feature | Calibration result | Development result | Decision |
| --- | --- | --- | --- |
| Existing field-norm EMA plus yaw pseudo-NIS | all 8,788 updates accepted while fusion is harmful | all 8,782 updates accepted while fusion is harmful | insufficient |
| Gyro-propagated consecutive body-field direction residual | P95 `1.172 deg`; correlation with absolute on-minus-off yaw effect `-0.014` | P95 `1.188 deg`; correlation `-0.106` | useful for an abrupt direction fault family, but does not identify this slow failure |
| Low-dynamic magnetic inclination proxy relative to raw specific force | 3,352 samples; proxy-to-physical-inclination correlation `0.779`; yaw-effect correlation `-0.339` | 1,653 samples; correlations `0.480` and `+0.332` | not stable enough to select as a source supervisor |

The direction residual integrates the preceding physical gyro samples only and compares the result
to the next physical magnetometer direction. The inclination proxy only evaluates samples satisfying
the existing static-alignment bounds: acceleration norm within `0.20 g` of gravity and gyro norm at
most `0.05 rad/s`. Its reference-attitude correlation is reported only to quantify the proxy offline;
reference attitude is not an allowed runtime input.

Consequently, this branch deliberately enables no new runtime magnetometer supervisor and changes no
magnetic tuning value. `fuse_magnetometer` remains false in the public ESKF default configuration.
The practical current policy is to treat unqualified magnetic yaw as unavailable, use an explicitly
qualified trusted-heading source when one exists, and keep the source-quality work separate from the
primary navigation estimator.

## Causal supervisor boundary

Any future opt-in source supervisor may use only causal physical information:

- physical magnetic vector, timestamp, validity, source, generation, and freshness;
- physical gyro information available before the decision;
- field-norm and gyro-propagated consecutive-direction envelopes frozen from calibration;
- prior sequential state for latch and recovery.

It must not consume ground-truth attitude/yaw, estimator error, future samples, trajectory labels,
or manually selected problem intervals. It must also report usable update coverage, so rejecting all
magnetic updates cannot be declared a solution.

There is a fundamental observability limit. A slowly changing or fixed magnetic heading offset with
nearly unchanged field magnitude is indistinguishable from yaw drift when the only information is
IMU plus magnetometer. A supervisor can reject detectable inconsistency, not infer which of those
two explanations is true. Reliable acceptance across such a condition requires an independent
heading reference, a stronger calibration/environment guarantee, or a separately validated model.

## Data discipline and release path

The `indoor_1` calibration and development windows are the only allowed design data. Its holdout was
already opened before this protocol and is historical adverse evidence only; it cannot be reused to
choose a candidate. `transition_1` is diagnostic only because the frozen datum transfer failed.

The next untouched sequence is `indoor_2`. It must remain unopened until the source-supervision
candidate code and configuration are frozen. It may receive independent local frame/time/datum
calibration, but no candidate thresholds or state-machine behavior may change. `indoor_3` then
replicates `indoor_2` using exactly the same commit and configuration.

Before a candidate is selected, it also requires synthetic fault families for constant-magnitude
direction steps, slow drift, persistent offset, field-norm changes, frozen samples, delay/reorder,
and sequential recovery. The candidate must preserve strict C99, API, covariance, and input-
integrity gates in addition to its source-specific evidence.
