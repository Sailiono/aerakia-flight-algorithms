# Delayed GNSS rewind/replay oracle

## Purpose and boundary

This document records a host-only prerequisite for any future fixed-lag
estimator work. It is not a flight feature, does not change the public ESKF
API, and must not be interpreted as FCOne transport-delay compensation.

The current public timestamped aiding API correctly rejects stale, reordered,
and future observations according to its documented contract. An observation
that is still inside the configured freshness age can otherwise be fused into
the current state. Before changing that behavior or injecting a lagged bias
correction, we need evidence that restoring a complete historical estimator
state and replaying the accepted event sequence has the expected mathematics.

## Exact experiment

`validation/delayed_gnss_reprop_oracle.c` contains a bounded, deterministic
host oracle. For one exact IMU-boundary GNSS P/V epoch it:

1. stores a complete `AerakiaEskf` snapshot after IMU propagation and before
   same-timestamp aiding;
2. suppresses that one source epoch on a delayed lane while a baseline lane
   receives it on time;
3. at the delayed delivery time, restores the source snapshot;
4. calls the existing timestamped GPS update; and
5. replays immutable subsequent IMU samples and already committed GNSS events
   in the same canonical order as the reference lane.

Because the whole `AerakiaEskf` is copied, the experiment replays nominal
state, covariance, timestamps, gate state, recovery/probation metadata, and
the existing Joseph update plus attitude-reset path together. It does not
invent a partial state-copy shortcut.

The fixed scope is deliberately narrow:

- exactly one delayed paired GNSS position/velocity epoch;
- source and delivery timestamps lie exactly on retained IMU boundaries;
- every other GNSS P/V update is delivered on time;
- no delayed magnetometer, trusted heading, barometer, multi-IMU selector,
  supervisor, controller, measurement interpolation, or source-arrival clock;
- bounded 64-sample host ring only, with unsupported timing failing closed.

## Executed matrix

The compact result is
[`delayed_gnss_repropagation_oracle_v1.json`](../validation/public/delayed_gnss_repropagation_oracle_v1.json).
It covers one synthetic-v2 P/V stream at:

| IMU rate | Delivery delay | Cases |
| ---: | ---: | ---: |
| 100 Hz | 20, 50, 100 ms | 3 |
| 200 Hz | 20, 50, 100 ms | 3 |
| 400 Hz | 20, 50, 100 ms | 3 |

All `9/9` cases pass. The source update is accepted, delayed and baseline lanes
diverge before delivery by roughly `8.5e-4` to `9.0e-4` in maximum state
component difference, and after replay all cases report zero state difference,
zero covariance difference, matching estimator metadata, finite state, and PSD
covariance under the double-precision tolerance of `1e-12`.

This proves equivalence for the specified synthetic, isolated event pattern. It
does not quantify navigation accuracy, GNSS delay tolerance, processor cost,
memory use on STM32H7, or flight safety.

## Reproduction

```bash
cmake -S . -B build/delayed-oracle -DCMAKE_BUILD_TYPE=Release
cmake --build build/delayed-oracle --parallel
python3 validation/run_delayed_gnss_repropagation_oracle.py \
  --oracle build/delayed-oracle/aerakia_delayed_gnss_reprop_oracle \
  --out build/delayed-gnss-reprop/oracle-v1.json
```

The CTest entry runs one representative 400 Hz / 100 ms case. The Python
campaign runs the full nine-case matrix and fails closed if a native result
omits the research-only status, source-contract label, divergence sensitivity,
post-replay equivalence, health, or PSD condition.

## Consequence for G0

The rejected fixed-lag bias proposal remains rejected. This oracle only removes
one infrastructure uncertainty: full snapshot/update/replay can reproduce a
zero-delay reference under a controlled isolated event. A future correction
candidate still requires a full 15-state lag covariance, process/preintegration
covariance, atomic nominal-state injection/reset treatment, causal persistence,
clean train/tune separation, a sealed holdout, and physical FCOne source and
arrival timestamps. None of those requirements is closed here.
