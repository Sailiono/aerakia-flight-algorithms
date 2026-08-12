# 2026-08-12 source-continuity break API

## Reason

FCOne's private GNSS and trusted-heading supervisors need a hardware-neutral
way to revoke controller-facing aiding validity when a receiver, generation,
or quality lifecycle changes. Reaching into `AerakiaEskf` internals from the
private repository would make the integration depend on undocumented state
layout, while reinitializing the entire filter would unnecessarily discard the
attitude, covariance, and learned IMU biases.

## Contract

- `aerakia_eskf_break_gps_source_continuity()` clears horizontal and vertical
  GNSS validity, pending recovery candidates, and recovery probation state.
  The paired GNSS API updates all three position/velocity axes, while the
  current filter does not retain per-source vertical provenance. Conservatively
  revoking both axes prevents stale GNSS evidence from surviving a receiver
  lifecycle change; later barometer or velocity observations remain free to
  qualify their own output again.
- `aerakia_eskf_break_heading_source_continuity()` clears trusted-heading
  validity.
- Both calls preserve the nominal state, covariance, learned biases, and
  source timestamp watermarks. A source transition cannot make an older sample
  appear new.
- The calls do not authorize a replacement source, reset navigation, select a
  receiver, or grant controller authority. Those policies remain private to
  the vehicle integration.

## Evidence

The public API test establishes accepted GNSS and heading, injects pending
recovery state, breaks both continuity classes, and verifies that output
validity/recovery evidence is cleared while state, covariance, biases, and
anti-replay timestamps remain byte-for-byte unchanged.
