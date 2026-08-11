# Documentation Index

This directory records decisions, evidence, and known limits for the portable
algorithm layer. It is not a substitute for the private FCOne product safety
case or board-level verification.

## Start Here

- [Algorithm status](algorithm-status.md): current capability, quantitative
  evidence, and blockers.
- [Development roadmap](roadmap.md): ordered work before and after FCOne v2
  hardware is available.
- [FCOne v2 pre-hardware closure](fcone-v2-algorithm-closure.md): what may
  enter shadow integration, and what still blocks sole control authority.

## Repository Map

- [`../include`](../include) and [`../src`](../src): portable C99 public API
  and implementation.
- [`../tests`](../tests): deterministic unit and contract tests.
- [`../simulation`](../simulation): synthetic and public/private replay-input
  conversion tools.
- [`../validation`](../validation): campaign runners, host contracts, and
  compact reviewed evidence.
- [`../tools/validation-workstation`](../tools/validation-workstation): local
  viewer for committed, decimated evidence only.

The directories above now each contain a short `README.md` with their entry
points and retention boundary.  The repository stays intentionally flat within
`docs/` because its documents cross-reference one another heavily; this index
is the stable navigation layer rather than a disruptive mass rename.

## Architecture And Integration

- [Architecture](architecture.md) and [state augmentation policy](state-augmentation-policy.md)
  define the 16-component nominal / 15-dimensional error-state model and the
  conditions for adding states.
- [Integration guide](integration.md), [coordinate conventions](coordinate-conventions.md),
  and [estimator supervision](estimator-supervision.md) define the hardware-neutral
  API, time/frame contract, and ESKF/Mahony application roles.
- [Vehicle and repository flow](vehicle-target-and-repository-flow.md),
  [public boundary](public-boundary.md), and [release policy](release-policy.md)
  define the public/private split.

## Validation Evidence

- [Validation method](validation.md), [validation execution log](validation-execution-log.md),
  and [public datasets](public-datasets.md) define evidence grades, methods,
  inputs, results, and limits.
- [PX4-class validation plan](px4-class-validation-plan.md), [PX4 comparison](px4-ekf2-comparison.md),
  and [PX4 bias A/B protocol](px4-bias-ab-protocol.md) keep engineering
  comparison separate from independent truth.
- [Timing and precision validation](timing-and-precision-validation.md),
  [Cortex-M7 cross-compile preflight](cortex-m7-cross-compile.md),
  [input integrity validation](input-integrity-validation.md), and
  [golden replay contract](golden-dataset.md) record transport and numerical
  coverage.

## Focused Open Investigations

- [Bias observability](bias-observability.md) and
  [G0 correlated static-prior rejection](g0-correlated-static-prior.md): the
  retained cold-start horizontal accelerometer-bias boundary and rejected
  shortcut.
- [G0 v2 input contract](bias-observability-v2-input-contract.md): causal
  stationarity and interval delta semantics that precede the next estimator
  hypothesis.
- [G0 rate sensitivity](bias-observability-rate-sensitivity.md): controlled
  four-rate/noise study showing why the current analyzer cannot yet trigger a
  correction.
- [G0 gate characterization](g0-gate-characterization.md): stationary-noise
  null and zero-noise structural controls for measuring analyzer error rates.
- [G0 trajectory screen](g0-bias-observability-trajectory-screen.md): frozen
  maneuver geometry screen and its limits.
- [G0 score calibration](g0-score-calibration.md): disjoint stationary-null
  validation and noisy maneuver score campaign.
- [Fixed-lag proposal](fixed-lag-bias-proposal.md): rejected no-injection
- [Fixed-lag replay candidate](fixed-lag-replay-candidate.md): complete
  host-only correction/replay experiment, rejected for promotion after a
  216-trial paired development matrix
- [Full-state sensitivity screen](full-state-sensitivity-screen.md): full 15D
  nuisance/covariance diagnostic, rejected after a frozen 60-trial screen
- [Fixed-lag covariance composition](fixed-lag-covariance-composition.md):
  variable-rate full-state `Phi/Q` propagation prerequisite, verified without
  adding a smoother or delayed-fusion feature
- [Fixed-lag measurement cross covariance](fixed-lag-measurement-cross-covariance.md):
  double-precision P/V transaction passes; host float exposes an augmented-PSD
  limitation that blocks a naive embedded lag implementation
- [Barometer multi-lane handoff contract](barometer-multilane-handoff-contract.md):
  host-only complete active-to-shadow ESKF image transfer and fail-closed
  transaction evidence after a barometer source latch
- These records define the solver boundary and propagation/replay prerequisites
  before another estimator candidate.
- [Delayed GNSS replay oracle](delayed-gnss-repropagation-oracle.md): the
  host-only prerequisite for any future fixed-lag state correction.
- [Barometer source supervision](barometer-supervision-study.md) and
  [airspeed/barometer plan](airspeed-barometer-plan.md): vertical-aiding
  evidence, limits, and follow-up architecture.
- [Magnetic source supervision](magnetic-source-supervision.md) and
  [absolute-heading evidence](absolute-heading-evidence.md): why magnetic yaw
  is fail-safe by default and how trusted heading is qualified.

## Visual Evidence, History, And Retention

- [`../tools/validation-workstation`](../tools/validation-workstation): local,
  reproducible evidence viewer. It visualizes compact committed artifacts and
  never downloads or stores raw flight data.
- [Algorithm optimization showcase](algorithm-optimization-showcase/README.md):
  presentation source retained as a compact project artifact.
- [Validation workstation log](validation-explorer-development-log.md): viewer
  changes and source-data limits.
- [Validation execution log](validation-execution-log.md): append-only
  chronological experiment record.  Use focused documents for current policy;
  use this log when auditing why a decision changed.
- [Workspace artifact policy](workspace-artifact-policy.md): what belongs in
  Git, private archive, or disposable build output.

When a result changes a decision, update the focused document above and add a
compact, hashed summary under `validation/public/`. Do not commit raw flight
logs, dataset archives, build products, or high-rate replay CSVs.
