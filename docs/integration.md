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

## Private driver adapter responsibilities

Before publishing `AerakiaImuSample`, the board/application layer must:

1. read the physical sensor and attach its physical sample timestamp;
2. apply sensor calibration and temperature compensation;
3. remap axes to body FRD;
4. convert acceleration to m/s², angular rate to rad/s, and magnetic field to µT;
5. set validity flags only for measurements that passed driver-level checks;
6. publish samples in monotonic timestamp order.

The public algorithm layer does not know whether the source is SPI, I²C, CAN, a middleware topic, a file, or a PC simulator.

If the application already has a trusted startup attitude (for example, vision or a retained
alignment solution), call `aerakia_mahony_seed_attitude` before the first sample. The first update
then establishes the physical timestamp without replacing the seed. Otherwise Mahony initializes
tilt from accelerometer and yaw from a valid magnetometer, or zero yaw when no heading observation
exists. A trusted seed is an integration contract, not a substitute for validating its source.

## Scheduling

Call the estimator once per new IMU sample. The timestamp, not task wake-up time, defines the integration interval. Duplicate, reversed, or excessively delayed samples return `AERAKIA_STATUS_TIMESTAMP_ERROR`.

Lower-rate aiding measurements are explicit calls:

```c
aerakia_eskf_update_gps(&navigation_filter,
                        gps_position_ned_m, gps_velocity_ned_m_s,
                        gps_position_variance_m2, gps_velocity_variance_m2_s2);
aerakia_eskf_update_barometer(&navigation_filter, barometric_height_up_m, baro_variance_m2);
aerakia_eskf_update_heading(&navigation_filter, heading_ned_rad, heading_variance_rad2);
```

The paired GPS API gates position and velocity separately. If both are rejected for the configured consecutive limit, it re-anchors only position/velocity with covariance floors; attitude and learned IMU biases are preserved.

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
