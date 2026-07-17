# Development roadmap

This roadmap separates work that can be completed on a PC from work that requires the FCOne v2
hardware. The public algorithm repository remains the only editable source for Mahony, ESKF, and
hardware-independent validation code. FCOne repositories consume a reviewed commit through an
adapter; they do not maintain a second algorithm copy.

Public branch, history, and tagging rules are defined in the [release policy](release-policy.md).

## Current baseline

- Public `main` recovery point: `ef03c348e4d73d7e5461c9f4aec0b112760cffda`.
- The recovered private checkpoint has the same source tree as public `main`.
- Strict host build plus the C and Python test suites pass on the recovered local machine.
- Twenty-four private PX4 ULogs and the minimal EuRoC inputs are retained outside the public Git
  history and have verified SHA-256 manifests.
- Flight qualification, target timing, thermal behavior, and physical heading truth remain open.

## P0 — hardware-independent closure

| Work item | Acceptance evidence | Status |
| --- | --- | --- |
| One-command host regression | Configure, build, C tests, Python tests, deterministic scenarios, threshold checks, logs, and environment manifest complete from one command | Implemented by `validation/run_host_regression.py`; keep it passing |
| Monte Carlo consistency | Reviewed seed set covers IMU bias/noise, timestamp jitter, aiding loss, and recovery; aggregate NIS/NEES confidence bounds and failure seeds are reported | Constant three-axis bias, monotonic timestamp jitter, measurement noise, and five-second aiding loss implemented; thermal drift and transport faults pending |
| Trusted-heading behavior | Controlled independent heading covers cold-start yaw, normal updates, dropout, outliers, rejection, and recovery without treating GNSS course as body yaw | Deterministic synthetic gate implemented; independent physical heading remains P2 evidence |
| Bias convergence | Gyroscope and accelerometer bias error, convergence time, and steady-state uncertainty are reported separately; batch reference-bias correction is not counted as online convergence | Multi-axis synthetic gate implemented; thermal and physical characterization remain P2 evidence |
| Timing and malformed input | Duplicate, stale, out-of-order, delayed, missing, non-finite, and implausible samples have explicit deterministic behavior and tests | Implemented deterministic matrix plus 1,020,000-attempt, 100-seed campaign; retain as a host gate |
| Public dataset runner | Dataset manifest records source, hash, frame transform, time offset, command, code commit, and output summary; selected EuRoC runs reproduce with one command | Pending |
| FCOne-neutral contract tests | A mock publisher verifies timestamp, FRD/NED frames, SI units, validity flags, update freshness, dropout, and stale-aiding behavior without including private FCOne headers | Pending |

Run the complete current host gate with:

```bash
python validation/run_host_regression.py
```

Generated logs, reports, plots, and `run-manifest.json` stay under `build/` and are not committed.

Run the first reviewed 20-seed navigation consistency baseline after building the native runner:

```bash
python validation/run_monte_carlo.py \
  --runner build/host-regression/aerakia_validation_runner
```

This phase deliberately reports empirical P05/P95 ranges across seeds. It covers drawn constant
three-axis IMU biases and monotonic timestamp jitter. Transport gaps, reordering, malformed values,
burst loss, and aiding freshness are now covered by the separate input-integrity campaign; thermal
drift remains hardware-required evidence.

## P1 — FCOne v2 integration before hardware arrival

1. Freeze the FCOne v1 and BSP-validation source commits used by each migrated module.
2. Keep FCOne v2 as a private, independent build; remove build-time references to sibling v1
   directories as modules are migrated.
3. Pin this repository as a submodule or equivalent immutable dependency.
4. Implement a private `FCOneAerakiaAdapter` for calibrated sensor publications and estimator
   outputs. Board registers, CubeMX, HAL, RTOS, and product logic stay private.
5. Run the adapter against synthetic and recovered replay streams in shadow mode before it can
   affect a control path.

## P2 — hardware-required evidence

- Board bring-up and per-peripheral acceptance using the BSP-validation evidence as the reference.
- Static multi-orientation and thermal IMU characterization.
- Motor-off, throttle-sweep, and recovery magnetic-disturbance tests.
- Target-MCU execution time, stack, precision, watchdog, and long-run numerical checks.
- Rate-table, motion-capture, or dual-antenna GNSS heading truth.
- HIL and bounded flight-envelope expansion with logged abort criteria.

## Data still worth collecting

Additional ordinary ULogs are lower priority. The highest-value new evidence is:

1. Valid dual-antenna GNSS heading during static rotation and dynamic flight.
2. Independent yaw truth from motion capture or a rate table with synchronized IMU/magnetometer.
3. Controlled GNSS interruption and recovery with detailed aiding-source innovations.
4. Controlled motor-current magnetic-disturbance data at fixed vehicle attitude.

No physical dataset should be described as ground truth unless its independent measurement chain,
frame transform, and synchronization uncertainty are recorded.
