# Test Map

This directory contains deterministic unit and contract tests.  It does not
contain replay inputs or campaign outputs.

## Run

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

The C tests are registered with CTest after configuring the top-level CMake
project:

```bash
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Release
cmake --build build/host --parallel
ctest --test-dir build/host --output-on-failure
```

## Groups

- `test_eskf*.c`, `test_mahony.c`, and `test_*supervisor.c`: numerical core,
  covariance, heading, magnetic, and barometer behavior.
- `test_public_api.c` and `test_version.c`: public API and release boundary.
- `test_synthetic_imu_input_contract.py`: the v2 interval/delta, timestamp,
  quaternion, and causal-stationarity input contract.
- `test_bias_excitation_information.py` and related `test_bias_*` files:
  analyzer-only observability diagnostics, provenance, and campaign resume
  behavior.  These tests must not be interpreted as flight qualification.
- `test_convert_*.py`, `test_calibrate_*.py`, and `test_python_tools.py`:
  replay-conversion and host-tool contracts.
- `test_*magnetometer*.py`, `test_baro_outage_ab.py`, and
  `test_aiding_delay_sensitivity.py`: deterministic validation protocol
  guards.

Full campaigns live in [`../validation`](../validation); their raw inputs and
temporary outputs belong under ignored `build/` directories, never here.
