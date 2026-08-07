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
- `run_baro_outage_campaign.py` and `run_baro_outage_ab.py`: barometer source
  supervision experiments.
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
