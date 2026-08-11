# Fixed-lag covariance composition prerequisite

## Purpose

The rejected replay candidates used one covariance snapshot at their retained
boundary. A mathematically valid fixed-lag smoother instead needs the complete
transition and process covariance accumulated over every retained IMU interval:

```text
Phi(k, 0) = F(k) ... F(1)
Q(k, 0) = F(k) Q(k - 1, 0) F(k)^T + Q(k).
```

This host-only oracle verifies that this propagation identity agrees with the
production `eskf_predict()` implementation before any smoother is designed. It
does not add a smoother, a history buffer, delayed fusion, or a new public API.

## Executable Oracle

[`fixed_lag_covariance_composition_oracle.c`](../validation/fixed_lag_covariance_composition_oracle.c)
uses the validation-only helper
[`eskf_lag_covariance.h`](../validation/eskf_lag_covariance.h). For each chain
it creates a dense positive-semidefinite full 15-state boundary covariance,
then repeatedly:

1. Evaluates the same pre-integration `F` and interval `Q` that production
   prediction uses.
2. Appends them to the retained `Phi/Q` composition.
3. Calls `eskf_predict()` on an independently copied filter.
4. Compares the composed endpoint covariance with the production covariance.

The chains cover 1--10 ms intervals, random bounded attitude, acceleration,
angular rate, initial velocity/position/bias, and process-noise parameters. At
every interval the accumulated process covariance and endpoint covariance must
remain finite, symmetric, and positive semidefinite. The helper also exposes
the future smoother relation:

```text
Cov(dx_k, dx_0) = Phi(k, 0) P_0.
```

## Current Result

The deterministic seed executed 384 variable-rate chains containing 4,592 IMU
intervals:

| Build | Maximum endpoint covariance difference | Gate |
| --- | ---: | --- |
| Reviewed double default | `7.81597009e-14` | `<= 2e-10` |
| Host float evaluation | `6.10351562e-05` | `<= 5e-04` |

Both configurations pass. This closes only the no-measurement propagation
identity required by later lag work.

## Explicitly Not Closed

This is not a fixed-lag smoother and is not evidence for a production
correction. It does **not** model or validate:

- Measurement-update cross covariance within the lag.
- ESKF relinearization and reset effects across a backward pass.
- Innovation correlations between sources.
- Source timestamp versus arrival timestamp, interpolation, or loss.
- FCOne buffer memory, execution time, or sensor-selector behavior.
- Any flight accuracy or bias-observability claim.

A future candidate must still use a complete forward/backward or equivalent
formulation, a physical FCOne transport contract, a clean protocol, and a
sealed holdout. The rejected 5D and full-state replay candidates remain
rejected; this prerequisite is not a reason to reopen their tuning.

## Reproduction

```bash
cmake -S . -B build/dev -DCMAKE_BUILD_TYPE=Release
cmake --build build/dev --parallel
ctest --test-dir build/dev --output-on-failure \
  -R fixed_lag_covariance_composition
```

The float evaluation is intentionally a host check only:

```bash
cmake -S . -B build/lag-float -DCMAKE_BUILD_TYPE=Release \
  -DAERAKIA_ESKF_CORE_PRECISION=float
cmake --build build/lag-float --parallel
ctest --test-dir build/lag-float --output-on-failure \
  -R fixed_lag_covariance_composition
```
