# Public API migration to v0.3.0

Aerakia is still pre-1.0. Version 0.3.0 intentionally breaks the adapter API so independent
position/velocity validity and supervised navigation recovery cannot be bypassed accidentally.
Consumers can check the compile-time version through `<aerakia/version.h>`.

## Split horizontal validity

`AerakiaEskfConfig.maximum_horizontal_dead_reckoning_s` was removed. Configure the two output
qualifications independently:

```c
AerakiaEskfConfig config;
aerakia_eskf_default_config(&config);

config.maximum_horizontal_position_dead_reckoning_s = 5.0f;
config.maximum_horizontal_velocity_dead_reckoning_s = 5.0f;
```

Replace decisions based only on `horizontal_aiding_age_s` with the source-specific fields:

```c
if (!estimate.horizontal_position_valid) {
    invalidate_position_output();
}
if (!estimate.horizontal_velocity_valid) {
    invalidate_velocity_output();
}
if (!estimate.horizontal_navigation_valid) {
    leave_position_control();
}
```

`horizontal_position_aiding_age_s` and `horizontal_velocity_aiding_age_s` now advance
independently. The retained `horizontal_aiding_age_s` is their maximum, or infinity if either source
has never been accepted; it is a summary for telemetry, not a controller qualification.

## Source-stamped recovery

Ordinary accepted observations require no recovery stamp. A paired GNSS observation that may become
a recovery candidate must identify one physical source and one unchanged quality decision:

```c
AerakiaGpsObservation gps = {
    .timestamp_us = gps_sample_time_us,
    .position_ned_m = gps_position_ned_m,
    .velocity_ned_m_s = gps_velocity_ned_m_s,
    .position_variance_m2 = gps_position_variance_m2,
    .velocity_variance_m2_s2 = gps_velocity_variance_m2_s2,
    .source_id = FCONE_GNSS_PRIMARY,
    .source_generation = gps_generation,
    .quality_sequence = supervisor_quality_sequence,
};

AerakiaStatus status = aerakia_eskf_update_gps_observation(&filter, &gps);
```

Keep `source_id` stable for the physical input. Increment `source_generation` after receiver reset,
failover, reconfiguration, or replacement. Keep `quality_sequence` unchanged only while the exact
supervisor quality snapshot remains valid.

Repeated rejected observations can only prepare a bounded candidate. The private supervisor must
authorize that exact candidate before a re-anchor:

```c
AerakiaNavigationEstimate estimate;
aerakia_eskf_get_estimate(&filter, &estimate);

if (estimate.navigation_recovery_candidate_ready && upstream_gnss_quality_verified()) {
    AerakiaNavigationRecoveryAuthorization authorization = {
        .observation_timestamp_us = gps.timestamp_us,
        .source_quality_verified = true,
        .maximum_position_correction_m = 60.0f,
        .maximum_velocity_correction_m_s = 10.0f,
        .source_id = gps.source_id,
        .source_generation = gps.source_generation,
        .quality_sequence = gps.quality_sequence,
    };
    status = aerakia_eskf_authorize_navigation_recovery(&filter, &authorization);
}
```

The authorization fails closed with `AERAKIA_STATUS_RECOVERY_REJECTED` if its timestamp or source
stamp differs, its quality assertion is absent, the candidate is stale/inconsistent, or correction
bounds are exceeded. A successful re-anchor enters probation; continue publishing synchronized
same-source observations and wait for `horizontal_navigation_valid` before using navigation for
control.

## Compile-time compatibility check

```c
#include <aerakia/version.h>

#if !AERAKIA_VERSION_AT_LEAST(0, 3, 0)
# error "Aerakia adapter v0.3.0 or newer is required"
#endif
```

## Explicit core process noise

`AerakiaEskfConfig` now contains an `ESKF_Config process_noise` member. Integrations must initialize
the complete adapter configuration before overriding a named vehicle profile:

```c
AerakiaEskfConfig config;
aerakia_eskf_default_config(&config);
config.process_noise.sigma_acc_bias = reviewed_accel_bias_random_walk;
```

Do not zero-initialize the structure and set only legacy fields. Zero is a valid request to disable
a process-noise contribution, while any non-finite or negative member causes the complete four-term
profile to fall back atomically to the portable-core defaults.
