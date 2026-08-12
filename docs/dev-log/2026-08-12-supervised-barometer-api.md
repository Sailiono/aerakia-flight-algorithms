# 2026-08-12 — Public supervised barometer transaction API

## Reason

The repository already had two independently tested pieces:

- `AerakiaBarometerSupervisor`, which classifies timestamp, jump, freeze, and
  recovery conditions; and
- `AerakiaEskf` timestamped barometer fusion.

The validation runner manually connected them. That left a production-facing
integration hazard: a caller could forget to capture the pre-update vertical
prediction, call the core before the supervisor, or commit a supervisor sample
that the ESKF rejected through its innovation gate.

## Change

Added the hardware-neutral API
`aerakia_eskf_update_supervised_barometer_observation()` and the
`AerakiaSupervisedBarometerObservation` contract. The function:

1. requires a processed IMU timestamp and clears per-call acceptance on a
   not-ready path;
2. captures the current positive-up height/vertical-velocity prediction;
3. evaluates the source supervisor with the physical sample timestamp and
   source identity/quality sequence;
4. refuses source faults before touching the ESKF state;
5. calls the existing timestamped ESKF barometer update exactly once; and
6. commits the supervisor only if the core reports `barometer_accepted`.

The existing 15-state measurement model and supervisor thresholds were not
changed. This API does not select redundant sources, convert pressure to
height, authorize a new datum, or perform a flight-control handoff.

## Verification

`tests/test_public_api.c` now covers the transaction boundary:

- positive-up barometer correction has the expected NED sign;
- invalid and duplicate source observations leave the ESKF state/covariance
  unchanged;
- a core NIS rejection does not advance the supervisor's fused baseline;
- persistent jump detection latches the source;
- recovery requires source-bound external authorization and remains
  probationary until the configured sample count is met.

Host command:

```text
cmake -S . -B build/baro-api -DAERAKIA_BUILD_VALIDATION=OFF
cmake --build build/baro-api -j2
ctest --test-dir build/baro-api --output-on-failure
```

Result: `17/17` CTest targets passed, including the existing barometer,
adapter, integrity, and multi-lane handoff contracts. This is a software
boundary result; no FCOne hardware or pressure-source qualification is
claimed.

The sanitizer build also compiled successfully and the new public API and
supervisor binaries passed with leak detection disabled. Full sanitizer CTest
was not usable in this Work execution environment because LeakSanitizer is
blocked by the host's ptrace policy; this is recorded as an environment limit,
not a passing sanitizer campaign.
