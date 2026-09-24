# Input integrity validation

## Purpose

This plan verifies the behavior of the hardware-neutral API when transport, timestamp, validity, or
numeric assumptions are violated. It complements estimation-accuracy datasets: ULog and EuRoC
answer whether valid real data can be processed, while this matrix answers whether invalid data can
corrupt state or be silently accepted.

## Acceptance principles

For each injected fault, record:

- attempted and accepted/rejected sample counts;
- exact returned status;
- whether estimator time advanced;
- quaternion, navigation state, bias, covariance, and health changes;
- first valid-sample recovery behavior;
- maximum consecutive faults tested;
- random seed and total samples for bulk campaigns.

Rejected measurements must not mutate estimator state, except that an excessive forward IMU gap may
re-anchor the integration timestamp so the next valid sample can resume without integrating across
missing time. This exception must remain explicit and tested.

## Coverage matrix

| Input class | Fault | Expected behavior | Evidence status |
| --- | --- | --- | --- |
| Required IMU | accelerometer validity clear | reject as missing; state/time unchanged | Passing deterministic and bulk checks |
| Required IMU | gyroscope validity clear | reject as missing; state/time unchanged | Passing deterministic and bulk checks |
| Required IMU | NaN/Inf on each of six axes | reject as missing; state/time unchanged | Passing exhaustive deterministic checks and bulk injection |
| IMU timestamp | duplicate | timestamp error; state/time unchanged | Passing deterministic and bulk checks |
| IMU timestamp | reversed | timestamp error; state/time unchanged | Passing deterministic and bulk checks |
| IMU timestamp | interval below minimum | timestamp error; state/time unchanged | Passing deterministic and bulk checks |
| IMU timestamp | interval above maximum | timestamp error; re-anchor without propagation | Passing deterministic and bulk checks |
| IMU transport | isolated dropped samples | bounded larger `dt`; propagate when within maximum | Passing randomized interval checks |
| IMU transport | burst loss exceeding maximum `dt` | reject gap, re-anchor, resume next sample | Passing burst lengths 1–100 |
| Optional mag | valid flag clear with finite/stale payload | ignore payload | Passing deterministic and bulk checks |
| Optional mag | valid flag set with NaN/Inf | ignore magnetometer only; IMU remains usable | Passing deterministic equivalence and bulk checks |
| Optional mag | magnitude outside gate | reject mag; continue IMU propagation | Passing gate and deterministic scenario tests |
| Aiding timestamp | duplicate/reordered/future/stale GNSS | reject without update | Passing timestamped API and 100-seed checks |
| Aiding timestamp | duplicate/reordered/future/stale heading | reject without update | Passing timestamped API and 100-seed checks |
| Aiding timestamp | duplicate/reordered/future/stale barometer | reject without update | Passing timestamped API and 100-seed checks |
| Aiding numeric | NaN/Inf vector or non-positive variance | reject without update | Passing deterministic and 100-seed checks |
| Stream robustness | 1–100 consecutive mixed faults | no NaN, no hidden state jump, deterministic recovery | Passing 1,020,000-attempt campaign |

## Planned data volume

The first hardware-independent campaign will contain:

- exhaustive deterministic API cases for every matrix row that is currently representable;
- each NaN and positive/negative infinity injected independently on all required IMU axes;
- burst lengths `1, 2, 5, 10, 20, 50, 100`;
- sample rates `50, 100, 200, 400, 1000 Hz` within configured limits;
- at least 100 random seeds and at least 1,000,000 total attempted IMU samples;
- retained per-fault counters and worst-case recovery/state-change metrics.

No acceptance limit will be relaxed only because a random seed fails. Each failure must be retained,
reproduced, classified, and either fixed or documented as an explicit unsupported condition.

## First completed campaign

The campaign is built as `aerakia_input_integrity_campaign` and is also run by the one-command host
regression. The reviewed 2026-07-18 run covered 100 fixed seeds and five sample rates:

- 1,020,000 IMU attempts, including 139,059 missing/non-finite rejections and 90,024 timestamp
  rejections;
- 18,800 burst-fault samples at lengths 1, 2, 5, 10, 20, 50, and 100;
- 20,355 forward-gap reanchors, 19,937 invalid optional-magnetometer cases, and 700 explicit
  recoveries;
- 600 accepted timestamped aiding calls plus 300 each duplicate, reordered, future, stale, and
  non-finite cases across GNSS, trusted heading, and barometer;
- zero state/covariance invariant failures and zero unhealthy outputs.

The machine-readable summary remains in the generated host-regression output. Commands, runtime,
environment, and individual stdout/stderr logs are retained by `run_host_regression.py`.
