# Simulation And Replay Inputs

`simulation/tools/` is the input-conversion layer for the portable host
validation path.  It emits the hardware-neutral golden replay CSV contract
consumed by `validation/validation_runner.c`; it is not FCOne board code.

## Core Inputs

- `generate_synthetic_imu.py`: deterministic synthetic trajectories, fault
  injection, and the v2 interval/delta IMU contract.
- `convert_euroc_to_replay.py`, `convert_blackbird_to_replay.py`,
  `convert_urbannav_to_replay.py`, and `convert_insane*_to_replay.py`:
  public-dataset adapters.
- `convert_ulog_to_replay.py`: private PX4 ULog adapter.  Original ULogs and
  derived full replay CSVs are private archive material and must not enter Git.
- `analyze_golden_imu.py`, `analyze_yaw_drift.py`, and
  `compare_golden_runs.py`: offline inspection utilities.

The exact row schema, timing semantics, coordinate conventions, and retention
boundary are defined in [`../docs/golden-dataset.md`](../docs/golden-dataset.md),
[`../docs/coordinate-conventions.md`](../docs/coordinate-conventions.md), and
[`../docs/workspace-artifact-policy.md`](../docs/workspace-artifact-policy.md).
