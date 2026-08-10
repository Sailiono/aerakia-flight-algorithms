# Sampling, delivery delay, and precision evidence

This note records the host-only timing/precision boundary before FCOne v2 hardware is available.
It does not replace target-MCU timing, HIL, or flight evidence.

## IMU rate invariance

`validation/run_multirate_accuracy.py` is the reviewed rate-invariance gate. It uses the same
physical IMU/GNSS/magnetometer streams at `100`, `200`, `400`, and `1000 Hz`:

- interval-average gyro samples reproduce the reference rotation deterministically;
- IMU per-sample noise scales with the square root of rate to preserve continuous white-noise
  density;
- magnetometer and GNSS publication rates remain fixed at `100 Hz` and `10 Hz`;
- deterministic integration and paired-seed stochastic accuracy, NIS, and NEES are gated
  separately.

The frozen 32-seed result is recorded in the execution log: position RMSE means span
`0.3770--0.3777 m`, velocity `0.1658--0.1667 m/s`, post-alignment attitude
`0.6694--0.6766 deg`, and navigation NEES `4.8601--4.8628`. This supports the reviewed
high-rate reduced process-noise approximation across that range; it is not an exact full-model
discretization proof outside it.

## Timestamped aiding delivery

The public adapter validates physical observation timestamp, source ordering, and configurable
age. It intentionally has no history buffer, state rewind, or output re-propagation. Therefore an
older GPS/heading observation accepted inside `maximum_aiding_age_s` updates the *current* state;
it is not out-of-sequence measurement (OOSM) fusion.

`validation/run_aiding_delay_sensitivity.py` separates GNSS acquisition time from delivery time
using optional replay columns:

- `gps_timestamp_us`;
- `gnss_heading_timestamp_us`.

The runner accepts `--maximum-aiding-age-s` only to reproduce a chosen adapter policy. It emits
input timestamp, age, and API status in the result CSV so an accepted delayed observation cannot
be mistaken for a same-time observation.

The checked host matrix used a 12 s synthetic cold-start/navigation-outage stream, fixed seed,
IMU rates `100/200/400/1000 Hz`, and GNSS delivery delays `0/25/50/100/200 ms`. IMU noise was
rate-scaled; magnetometer fusion was disabled to isolate navigation aiding.

| Policy | 0 ms | 200 ms | Interpretation |
| --- | --- | --- | --- |
| Best-effort age window (`0.2 s`) | All 70 delivery attempts accepted | 68 terminally deliverable attempts accepted | Numerically healthy, but position RMSE grows from `0.206--0.213 m` to `0.385--0.398 m`; this is a sensitivity result, not delay compensation. |
| Strict zero age | All 70 attempts accepted | Every delayed attempt rejected stale | Fail-closed transport behaviour. Lower error in this short synthetic track is not a reason to discard valid GNSS; it only shows that this track cannot score a navigation policy by RMSE alone. |

All 40 policy/rate/delay cases remained numerically healthy and met their declared transport
outcome. The exact generated report stays outside Git under
`build/aiding-delay-sensitivity-final-v2/`. Its reproducible command is:

```bash
python validation/run_aiding_delay_sensitivity.py \
  --runner build/host-regression/aerakia_validation_runner
```

**FCOne v2 consequence:** the private adapter must publish physical sample timestamps, must not
overwrite them with task-delivery time, and must keep delayed GPS/heading out of normal fusion
until a separately designed and validated OOSM buffer/rewind/output-predictor path exists. This
study does not authorize a `0.2 s` production age window.

## Single-precision candidate

The public default remains `double`. CMake now exposes a candidate only:

```bash
cmake -S . -B build/float -DAERAKIA_ESKF_CORE_PRECISION=float
cmake --build build/float --parallel
ctest --test-dir build/float --output-on-failure
```

`float` is propagated publicly because callers allocate `ESKF_Handle`; a library/application
scalar-layout mismatch is forbidden. The analytical `test_eskf_models` target remains a
double-precision finite-difference/integration oracle, so it is intentionally omitted in the
float CTest configuration. Behavioural API, covariance, transport, supervisor, and adapter
tests remain enabled; their tolerances are explicitly widened only for the candidate build.

`validation/run_precision_comparison.py` replays byte-identical input through double and float
runners and gates output health, state divergence, and score deltas. The float core uses
type-matched scalar math; it no longer silently calls double `sin/cos/atan2/...` routines. On the
reviewed 20 s, 400 Hz host run:

| Scenario | Max attitude difference | Max position difference | Max velocity difference |
| --- | ---: | ---: | ---: |
| Clean motion | `0.001174 deg` | `0.003943 m` | `0.000670 m/s` |
| Cold-start GNSS outage | `0.000097 deg` | `0.000026 m` | `0.000012 m/s` |

Both float tracks were 100% healthy. Reproduce after building both runners:

```bash
python validation/run_precision_comparison.py \
  --double-runner build/double/aerakia_validation_runner \
  --float-runner build/float/aerakia_validation_runner
```

This clears a narrow host numerical-equivalence gate, not STM32H7 qualification. The companion
[Cortex-M7 cross-compile preflight](cortex-m7-cross-compile.md) records the target ABI, portable
layout, and remaining link dependencies. Before selecting float for FCOne v2, measure
prediction/update WCET, stack high-water mark, final Flash/RAM, FPU ABI, DMA/cache contention,
scheduler jitter, and multi-hour numerical health on the actual target.
