# Validation Map

This directory contains host-side executable validation contracts, campaign
runners, and compact public evidence.  It is deliberately separate from the
portable C library and from FCOne-specific integration code.

## Primary Entrypoints

- `run_suite.py`: deterministic synthetic host regression.
- `run_public_dataset_suite.py`: public-dataset conversion and replay suite;
  it requires separately obtained source data.
- `run_ulog_suite.py`: private PX4 ULog replay suite; it requires a private
  manifest and must not write raw data to Git.
- `run_monte_carlo.py`: seeded synthetic robustness and consistency campaign.
- `run_bias_observability_input_contract_v2.py`: v2 IMU interval/delta and
  causal-stationarity smoke.  Its excitation analyzer is research-only and
  cannot gate the flight estimator.
- `run_bias_observability_rate_sensitivity.py`: four-rate, three-noise-profile
  analyzer study; it records structural rank behavior without changing the
  estimator.
- `run_bias_observability_trajectory_screen.py`: zero-noise structural screen
  for frozen maneuver candidates; it does not establish noisy convergence.
- `run_bias_observability_gate_characterization.py`: stationary-noise null and
  zero-noise structural-control campaign for analyzer false-positive evidence.
- `run_bias_observability_null_split_validation.py`: disjoint-seed check of a
  development-only score threshold.
- `run_bias_observability_excitation_score_campaign.py`: noisy score coverage
  on two structural candidate maneuvers; not bias-convergence evidence.
- `fixed_lag_bias_proposal.py`: causal, prior-regularized joint tilt/bias
  proposal solver with no ESKF state injection. Its 144-replay campaign is
  rejected; use `run_fixed_lag_bias_proposal_campaign.py` to regenerate the
  compact evidence, and do not treat it as an estimator implementation. It
  defaults to one worker; `--jobs` is an explicitly validated workstation
  parallelism option.
- `fixed_lag_covariance_composition_oracle.c` and `eskf_lag_covariance.h`:
  validation-only variable-rate `Phi/Q` composition check for later smoother
  work. They do not provide delayed fusion or a product history buffer.
- `fixed_lag_measurement_cross_covariance_oracle.c`: 30x30 host-only P/V
  cross-covariance transaction check. It passes in the reviewed double build;
  float intentionally fails its PSD gate and is retained as a design warning.
- `barometer_multilane_handoff_contract.c`: host-only complete ESKF shadow-lane
  handoff contract after a causal barometer-source latch. It checks full image
  transfer and fail-closed timestamp/provenance/configuration/health cases; it
  is not a private FCOne controller policy.
- `run_delayed_gnss_repropagation_oracle.py`: host-only isolated, sequential
  non-overlapping, and tightly bounded two-event overlapping/reordered
  delayed-GNSS rewind/replay campaigns. The overlap gate requires the newer
  delivery to remain non-equivalent while the earlier source is pending, then
  requires final exact state/covariance equivalence to a zero-delay reference.
  It is not a product delayed-fusion API or a claim of generic OOSM support.
- `run_cortex_m7_cross_compile.py`: strict Cortex-M7 hard-float compile,
  internal-link, public-layout, and dependency preflight. It does not create
  an FCOne image or measure target timing, stack, Flash/RAM, or runtime health.
- `run_baro_outage_campaign.py` and `run_baro_outage_ab.py`: barometer source
  supervision experiments.
- `run_airspeed_wind_observability.py`: validation-only known-wind model and
  causal two-state wind-observability oracle. It intentionally does not call
  the production ESKF or add a TAS API/state; see
  [`../docs/airspeed-wind-observability.md`](../docs/airspeed-wind-observability.md).
- `run_airspeed_wind_observability_campaign.py`: two disjoint, frozen
  multi-seed replication windows for the same TAS/wind contract. It is a
  confirmation campaign, not a parameter-tuning loop or an estimator branch.
- `run_airspeed_wind_mismatch_monitor.py`: v2 opened-development and v4
  sealed-holdout residual-pattern monitor study for the retained vertical-wind
  failure; the pre-release v3 smoke is explicitly invalidated. The v4 holdout
  was executed once and failed `889/896` replications; its compact artifact is
  retained as evidence and must not be rerun for tuning. It has explicit
  full-window coverage, impulse/gap controls, and no production ESKF/API effect.
- `run_px4_bias_ab_m0.py`: optional PX4 host comparison; it is blocked until a
  matching PX4 source tree is supplied locally.

## Contracts And Analysis

- `validation_runner.c`: native C replay runner for the same library used by
  embedded integration.
- `fcone_adapter_contract.c` and `estimator_supervisor_contract.c`: neutral
  boundary tests.  They intentionally do not contain private FCOne drivers,
  sensor selection, or control policy.
- `analyze_bias_excitation_information.py`: causal, local observability
  diagnostic only.  Its limitations and current G0 status are documented in
  [`../docs/bias-observability.md`](../docs/bias-observability.md).
- `*.json`: versioned protocol, threshold, or manifest definitions.

## Evidence Retention

`public/` holds small, reviewed result summaries and manifests that support
documented claims.  It never holds raw flight logs, dataset archives, full
replay CSVs, build products, or plots.  See
[`../docs/workspace-artifact-policy.md`](../docs/workspace-artifact-policy.md)
for the retention and cleanup rules.
