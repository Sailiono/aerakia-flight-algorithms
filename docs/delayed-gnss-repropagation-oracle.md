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

- one isolated delayed paired GNSS position/velocity epoch, two sequential
  non-overlapping epochs, or exactly two overlapping/reordered epochs in a
  separately named host scenario;
- source and delivery timestamps lie exactly on retained IMU boundaries;
- every other GNSS P/V update is delivered on time;
- no delayed magnetometer, trusted heading, barometer, multi-IMU selector,
  supervisor, controller, measurement interpolation, or source-arrival clock;
- bounded 64-sample host ring only, with unsupported timing failing closed.

## Executed matrix

The compact result is
[`delayed_gnss_repropagation_oracle_v1.json`](../validation/public/delayed_gnss_repropagation_oracle_v1.json).
It covers one deterministic, exact-timestamp synthetic P/V stream at:

| IMU rate | Delivery delay | Cases |
| ---: | ---: | ---: |
| 100 Hz | 20, 50, 100, 150 ms | 4 |
| 200 Hz | 20, 50, 100, 150 ms | 4 |
| 400 Hz | 20, 50, 100, 150 ms | 4 |

All `12/12` cases pass. The source update is accepted, delayed and baseline lanes
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

The CTest entry runs one representative 400 Hz / 150 ms near-ring-limit case. The Python
campaign runs the full twelve-case matrix and fails closed if a native result
omits the research-only status, source-contract label, divergence sensitivity,
post-replay equivalence, health, or PSD condition.

## Sequential extension

The original matrix exercises one withheld source epoch. The same host oracle
now also has a separately named `sequential` scenario with two delayed GNSS
P/V epochs: sources at 3.0 s and 4.0 s. The second source is deliberately
after the first delivery, so its retained snapshot already includes the first
completed replay transaction. This is a meaningful repeat-transaction check,
but it is **not** a generic overlapping out-of-sequence measurement solution.

The compact sequential result is
[`delayed_gnss_repropagation_sequential_v1.json`](../validation/public/delayed_gnss_repropagation_sequential_v1.json).
It repeats the 100/200/400 Hz by 20/50/100/150 ms matrix: `12/12` cases pass.
Both source updates are accepted and replayed in every case. The first delayed
event produces a pre-delivery state difference of `8.53e-4`--`8.96e-4`; the
second produces `9.95e-4`--`1.06e-3`. After each replay and at the final time,
state and full covariance differences are exactly zero with matching metadata.

Reproduce it with:

```bash
python3 validation/run_delayed_gnss_repropagation_oracle.py \
  --oracle build/delayed-oracle/aerakia_delayed_gnss_reprop_oracle \
  --scenario sequential \
  --out build/delayed-gnss-reprop/sequential-v1.json
```

An overlapping source/delivery schedule, arbitrary reordering, loss of a
pending delayed event, interpolated source time, and mixed delayed sensor types
were intentionally outside this sequential scenario.

## Overlapping/reordered extension

The `overlap` scenario closes one specific ordering gap without becoming a
general out-of-sequence-measurement implementation. It uses two source epochs
at `3.000 s` and `3.020 s`, both on the 20 ms synthetic GNSS cadence. The
newer source is delivered one IMU interval later. The older source arrives
after `50`, `100`, or `150 ms`, so the newer observation is applied while one
earlier observation remains pending.

At each delivery, the delayed lane rebuilds from the earliest affected complete
pre-aiding snapshot. It replays the immutable IMU/P/V stream, but applies a
source observation only if that source has already been delivered. This is the
critical distinction from the isolated helper: the first replay cannot silently
fuse the older observation before its declared arrival.

The compact result is
[`delayed_gnss_repropagation_overlap_v1.json`](../validation/public/delayed_gnss_repropagation_overlap_v1.json).
It covers `100/200/400 Hz` at `50/100/150 ms`: `9/9` cases pass. In every
case, source order is chronological, delivery order is reversed, and the newer
delivery reports exactly one pending earlier source. Its post-delivery lane
remains observably different from the zero-delay baseline (state difference
about `1.19e-4`; covariance difference about `1.25e-3`) while that source is
pending. When the older observation arrives, the full state, covariance, and
reported metadata return to exact zero-delay equivalence under the declared
double-precision tolerance of `1e-12`; both lanes remain finite and PSD.

Reproduce the matrix with:

```bash
python3 validation/run_delayed_gnss_repropagation_oracle.py \
  --oracle build/delayed-oracle/aerakia_delayed_gnss_reprop_oracle \
  --scenario overlap \
  --out build/delayed-gnss-reprop/overlap-v1.json
```

The Python campaign fails closed if it sees fewer or more than two events,
non-chronological source order, non-overlapping windows, delivery order that is
not reversed, premature zero-delay equivalence for the newer delivery, or a
missing final equivalence after the older delivery. A 20 ms old-source delay is
rejected for this scenario because it cannot leave the older source pending at
the newer delivery.

This covers neither arbitrary event count nor a product buffer. It does not
validate loss of a pending event, interpolated source time, delayed
heading/barometer, mixed delayed sensor types, multi-IMU switching, physical
source/arrival timing, target resource cost, controller policy, or flight
safety. Those require a separately designed ordered event buffer and revised
snapshot-maintenance contract before any product implementation can be
considered.

## Consequence for G0

The rejected fixed-lag bias proposal remains rejected. This oracle only removes
one infrastructure uncertainty: full snapshot/update/replay can reproduce a
zero-delay reference for isolated, sequential non-overlapping, and one tightly
defined overlapping/reordered two-event pattern. A future correction candidate
still requires a full 15-state lag covariance, process/preintegration
covariance, atomic nominal-state injection/reset treatment, causal persistence,
clean train/tune separation, a sealed holdout, and physical FCOne source and
arrival timestamps. None of those requirements is closed here.
