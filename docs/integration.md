# Integration guide

## Data ownership

The application owns every algorithm context. Aerakia does not allocate memory or start tasks.

```c
static AerakiaMahony attitude_filter;
static AerakiaEskf navigation_filter;

void estimator_init(void)
{
    AerakiaMahonyConfig attitude_config;
    AerakiaEskfConfig navigation_config;

    aerakia_mahony_default_config(&attitude_config);
    aerakia_mahony_init(&attitude_filter, &attitude_config);

    aerakia_eskf_default_config(&navigation_config);
    navigation_config.fuse_magnetometer = true;
    navigation_config.magnetic_reference_ned[0] = local_field_north;
    navigation_config.magnetic_reference_ned[1] = local_field_east;
    navigation_config.magnetic_reference_ned[2] = local_field_down;
    aerakia_eskf_init(&navigation_filter, &navigation_config, NULL, NULL);
}
```

The ESKF is the primary navigation estimator. Configure the Mahony instance with the reviewed
robust profile and run it as a separate attitude fallback/cross-monitor. Standard Mahony is a
validation baseline, not the intended product fallback. The FCOne application owns mode selection,
handover continuity, alarms, and per-output validity; see
[Estimator supervision](estimator-supervision.md). Mahony-only operation must invalidate position
and velocity rather than presenting a degraded attitude estimate as full navigation.

## Private driver adapter responsibilities

Before publishing `AerakiaImuSample`, the board/application layer must:

1. read the physical sensor and attach its physical sample timestamp;
2. apply sensor calibration and temperature compensation;
3. remap axes to body FRD;
4. convert acceleration to m/s², angular rate to rad/s, and magnetic field to µT;
5. set validity flags only for measurements that passed driver-level checks;
6. publish samples in monotonic timestamp order.

The hardware-free oracle in `validation/fcone_adapter_contract.c` demonstrates and tests the final
boundary using a mock FCOne publication: physical timestamps are preserved, FRD axes are not
silently remapped, `g`/degrees-per-second/gauss are converted to m/s²/rad/s/µT, and validity flags
remain independent. The private adapter may start from different raw units, but its output must pass
the same sentinel-value, missing-field, duplicate, gap, future-aiding, stale-aiding, and recovery
checks before integration is accepted.

The public algorithm layer does not know whether the source is SPI, I²C, CAN, a middleware topic, a file, or a PC simulator.

If the application already has a trusted startup attitude (for example, vision or a retained
alignment solution), call `aerakia_mahony_seed_attitude` before the first sample. The first update
then establishes the physical timestamp without replacing the seed. Otherwise Mahony initializes
tilt from accelerometer and yaw from a valid magnetometer, or zero yaw when no heading observation
exists. A trusted seed is an integration contract, not a substitute for validating its source.

## Scheduling

Call the estimator once per new IMU sample. The timestamp, not task wake-up time, defines the
integration interval. Duplicate, reversed, and below-minimum intervals return
`AERAKIA_STATUS_TIMESTAMP_ERROR` without advancing estimator time. A forward interval above the
configured maximum is also rejected, but re-anchors estimator time so the next fresh sample can
resume without integrating across the missing interval.

Lower-rate aiding measurements are explicit calls:

```c
AerakiaGpsObservation gps = {
    gps_sample_time_us, gps_position_ned_m, gps_velocity_ned_m_s,
    gps_position_variance_m2, gps_velocity_variance_m2_s2
};
AerakiaHeadingObservation heading = {
    heading_sample_time_us, heading_ned_rad, heading_variance_rad2
};
AerakiaBarometerObservation barometer = {
    barometer_sample_time_us, barometric_height_up_m, baro_variance_m2
};

gps_status = aerakia_eskf_update_gps_observation(&navigation_filter, &gps);
heading_status = aerakia_eskf_update_heading_observation(&navigation_filter, &heading);
barometer_status = aerakia_eskf_update_barometer_observation(&navigation_filter, &barometer);
```

If the receiver publishes position and velocity independently, use the source-specific contracts:

```c
AerakiaPositionObservation position = {
    gps_position_time_us, gps_position_ned_m, gps_position_variance_m2
};
AerakiaVelocityObservation velocity = {
    gps_velocity_time_us, gps_velocity_ned_m_s, gps_velocity_variance_m2_s2
};

aerakia_eskf_update_position_observation(&navigation_filter, &position);
aerakia_eskf_update_velocity_observation(&navigation_filter, &velocity);
```

Do not differentiate receiver positions inside the adapter and mark the result as measured Doppler
velocity. If only one source is valid, publish only that observation. The paired API remains the
preferred path when a receiver truly provides synchronized position and velocity.

Use the physical measurement time, not message-delivery or task-wakeup time. The timestamped APIs
reject observations before the first IMU, from the future, duplicate/reordered per source, or older
than `maximum_aiding_age_s`. The older untimestamped update functions remain source-compatible for
existing host applications, but cannot enforce freshness and must not be used by the FCOne adapter.

The paired GPS API gates position and velocity separately. Independent APIs also maintain separate
freshness timestamps and can re-anchor only their own state component after persistent rejection.
Position recovery preserves velocity; velocity recovery preserves position; both preserve attitude
and learned IMU biases. A syntactically and temporally valid observation consumes its source
timestamp even if its innovation is rejected, preventing the same physical sample from being
retried as if it were new.

An accepted position or velocity constraint also refreshes the estimate's horizontal-aiding age.
By default, `maximum_horizontal_dead_reckoning_s` is 5 seconds. After that time without an accepted
horizontal constraint, `horizontal_position_valid`, `horizontal_velocity_valid`, and
`horizontal_navigation_valid` become false even if `healthy` remains true. `healthy` means the
state and covariance are finite and numerically coherent; it must never be used as a substitute for
navigation observability. Rejected observations do not refresh validity. An accepted ZUPT refreshes
the velocity-drift constraint, but does not initialize a previously unknown position origin.

The FCOne adapter must propagate these validity fields to the private estimator supervisor. A
vehicle-specific policy may choose a shorter limit, but must not silently extend it without physical
evidence and a matching failsafe review.

Trusted heading is independent of magnetometer fusion. It can come from dual-antenna GNSS, vision, motion capture, or another upstream estimator, provided the application converts it to clockwise-from-North NED radians and supplies a defensible variance.

## Application-declared stationary alignment

Set `AERAKIA_SAMPLE_STATIONARY` only when the application has independent reason to know the vehicle is stationary (for example, pre-arm state plus actuator/landing checks). The adapter also checks gyro norm and acceleration magnitude. While the configured sample-count and duration requirements are accumulating, `aerakia_eskf_process_imu` returns `AERAKIA_STATUS_ALIGNING` and does not propagate navigation.

When `aerakia_eskf_init` receives no initial quaternion and `static_align_attitude` is enabled, the
completed static window first aligns roll/pitch from mean specific force, then aligns yaw from the
accepted mean magnetic field, and finally initializes IMU biases. The estimate reports separate
`static_tilt_alignment_complete` and `static_heading_alignment_complete` flags. Magnetic heading is
optional: tilt and bias alignment can finish without it, but yaw then retains its initial value.

After alignment, the same flag enables periodic zero-velocity updates. The adapter exposes alignment status, sample count, per-row ZUPT application, and total ZUPT count. If the flag is absent, existing streaming behavior is unchanged; the library never guesses stationarity from IMU data alone.

## Coordinate and magnetic reference configuration

The ESKF magnetic update compares horizontal heading only; magnetic inclination is deliberately excluded so it cannot inject roll/pitch error. Configure a valid local horizontal field direction before enabling magnetometer fusion. For a magnetic declination `D`, a sufficient heading reference is `[cos(D), sin(D), 0]` in NED. The FCOne application should obtain `D` from a reviewed geomagnetic model using the current GNSS location, refresh it after a material location change, or prefer a trusted heading observation. Omitting declination produces a stable magnetic-north/true-north yaw offset rather than estimator divergence. The magnitude anomaly gate still uses the measured field norm.

See [Coordinate conventions](coordinate-conventions.md) for the complete frame and sign contract.
