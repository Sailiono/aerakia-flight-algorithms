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

## Scheduling

Call the estimator once per new IMU sample. The timestamp, not task wake-up time, defines the integration interval. Duplicate, reversed, or excessively delayed samples return `AERAKIA_STATUS_TIMESTAMP_ERROR`.

Lower-rate aiding measurements are explicit calls:

```c
aerakia_eskf_update_position(&navigation_filter, gps_position_ned_m, gps_variance_m2);
aerakia_eskf_update_barometer(&navigation_filter, barometric_height_up_m, baro_variance_m2);
aerakia_eskf_apply_zero_velocity(&navigation_filter, zupt_variance_m2_s2);
```

## Coordinate and magnetic reference configuration

The ESKF magnetic update compares the body measurement with a configured NED reference vector. Set the local field direction, including inclination, before enabling magnetometer fusion. Do not enable it with an arbitrary default reference.

See [Coordinate conventions](coordinate-conventions.md) for the complete frame and sign contract.
