#include <aerakia/eskf_adapter.h>
#include <aerakia/mahony.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static int near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static int near_double(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static void set_process_noise_field(ESKF_Config *config, int field, double value)
{
    if (field == 0) config->sigma_acc = value;
    else if (field == 1) config->sigma_gyr = value;
    else if (field == 2) config->sigma_acc_bias = value;
    else config->sigma_gyr_bias = value;
}

static void set_vector_axis(AerakiaVec3f *vector, int axis, float value)
{
    if (axis == 0) vector->x = value;
    else if (axis == 1) vector->y = value;
    else vector->z = value;
}

static int eskf_core_unchanged(const AerakiaEskf *filter, const AerakiaEskf *before)
{
    return memcmp(&filter->core.state, &before->core.state, sizeof(filter->core.state)) == 0
        && memcmp(filter->core.P, before->core.P, sizeof(filter->core.P)) == 0;
}

static int mahony_state_unchanged(const AerakiaMahony *filter, const AerakiaMahony *before)
{
    return memcmp(filter->quaternion_wxyz, before->quaternion_wxyz,
                  sizeof(filter->quaternion_wxyz)) == 0
        && memcmp(&filter->integral_feedback_rad_s, &before->integral_feedback_rad_s,
                  sizeof(filter->integral_feedback_rad_s)) == 0
        && filter->last_timestamp_us == before->last_timestamp_us
        && filter->healthy == before->healthy;
}

static AerakiaImuSample level_sample(uint64_t timestamp_us)
{
    AerakiaImuSample sample;
    memset(&sample, 0, sizeof(sample));
    sample.timestamp_us = timestamp_us;
    sample.acceleration_m_s2.z = -AERAKIA_GRAVITY_M_S2;
    sample.magnetic_field_ut.x = 22.0f;
    sample.magnetic_field_ut.z = 44.0f;
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID
        | AERAKIA_SAMPLE_GYRO_VALID
        | AERAKIA_SAMPLE_MAG_VALID;
    return sample;
}

static void euler_quaternion(float roll, float pitch, float yaw, float q[4])
{
    const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
    const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static AerakiaVec3f ned_to_body(const float q[4], AerakiaVec3f ned)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    AerakiaVec3f body;
    body.x = (1.0f - 2.0f * (y * y + z * z)) * ned.x
        + 2.0f * (x * y + w * z) * ned.y + 2.0f * (x * z - w * y) * ned.z;
    body.y = 2.0f * (x * y - w * z) * ned.x
        + (1.0f - 2.0f * (x * x + z * z)) * ned.y + 2.0f * (y * z + w * x) * ned.z;
    body.z = 2.0f * (x * z + w * y) * ned.x + 2.0f * (y * z - w * x) * ned.y
        + (1.0f - 2.0f * (x * x + y * y)) * ned.z;
    return body;
}

static void test_mahony_level_initialization(void)
{
    AerakiaMahony filter;
    AerakiaAttitudeEstimate estimate;
    AerakiaImuSample sample = level_sample(1000U);

    aerakia_mahony_init(&filter, NULL);
    check_true(
        aerakia_mahony_update(&filter, &sample, &estimate) == AERAKIA_STATUS_INITIALIZED,
        "Mahony initializes from first sample"
    );
    check_true(near(estimate.euler_rad.x, 0.0f, 1.0e-5f), "level roll initializes to zero");
    check_true(near(estimate.euler_rad.y, 0.0f, 1.0e-5f), "level pitch initializes to zero");
    check_true(near(estimate.euler_rad.z, 0.0f, 1.0e-5f), "north yaw initializes to zero");
}

static void test_mahony_yaw_integration(void)
{
    AerakiaMahony filter;
    AerakiaMahonyConfig config;
    AerakiaAttitudeEstimate estimate;
    AerakiaImuSample sample = level_sample(0U);
    int index;

    aerakia_mahony_default_config(&config);
    config.kp_magnetometer = 0.0f;
    config.ki = 0.0f;
    aerakia_mahony_init(&filter, &config);
    (void)aerakia_mahony_update(&filter, &sample, &estimate);

    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID;
    sample.angular_rate_rad_s.z = AERAKIA_PI_F / 2.0f;
    for (index = 1; index <= 100; ++index) {
        sample.timestamp_us = (uint64_t)index * 10000U;
        (void)aerakia_mahony_update(&filter, &sample, &estimate);
    }
    check_true(near(estimate.euler_rad.z, AERAKIA_PI_F / 2.0f, 2.0e-4f), "Mahony integrates 90-degree yaw");
}

static void test_mahony_trusted_attitude_seed(void)
{
    AerakiaMahony filter;
    AerakiaAttitudeEstimate estimate;
    AerakiaImuSample sample = level_sample(1000U);
    float q[4];

    euler_quaternion(0.0f, 0.0f, AERAKIA_PI_F / 4.0f, q);
    aerakia_mahony_init(&filter, NULL);
    check_true(
        aerakia_mahony_seed_attitude(&filter, q) == AERAKIA_STATUS_INITIALIZED,
        "Mahony accepts a trusted attitude seed"
    );
    check_true(
        aerakia_mahony_update(&filter, &sample, &estimate) == AERAKIA_STATUS_INITIALIZED,
        "first sample establishes the seeded Mahony timestamp"
    );
    check_true(
        near(estimate.euler_rad.z, AERAKIA_PI_F / 4.0f, 1.0e-5f),
        "first sample preserves trusted yaw"
    );
    q[0] = NAN;
    check_true(
        aerakia_mahony_seed_attitude(&filter, q) == AERAKIA_STATUS_INVALID_ARGUMENT,
        "Mahony rejects a non-finite attitude seed"
    );
}

static void test_mahony_adaptive_weight_and_timestamp(void)
{
    AerakiaMahony filter;
    AerakiaAttitudeEstimate estimate;
    AerakiaImuSample sample = level_sample(1000U);

    aerakia_mahony_init(&filter, NULL);
    (void)aerakia_mahony_update(&filter, &sample, &estimate);
    sample.timestamp_us = 11000U;
    sample.acceleration_m_s2.z = -2.0f * AERAKIA_GRAVITY_M_S2;
    check_true(aerakia_mahony_update(&filter, &sample, &estimate) == AERAKIA_STATUS_OK, "dynamic sample processes");
    check_true(near(estimate.accelerometer_weight, 0.05f, 1.0e-5f), "dynamic acceleration is down-weighted");
    check_true(aerakia_mahony_update(&filter, &sample, &estimate) == AERAKIA_STATUS_TIMESTAMP_ERROR, "duplicate timestamp rejected");
}

static void test_mahony_input_integrity(void)
{
    AerakiaMahony filter;
    AerakiaMahony before;
    AerakiaAttitudeEstimate estimate;
    AerakiaImuSample sample = level_sample(1000U);
    const float nonfinite[] = {NAN, INFINITY, -INFINITY};
    int axis;
    int value_index;

    aerakia_mahony_init(&filter, NULL);
    (void)aerakia_mahony_update(&filter, &sample, &estimate);
    sample.timestamp_us = 11000U;
    (void)aerakia_mahony_update(&filter, &sample, &estimate);

    for (axis = 0; axis < 6; ++axis) {
        for (value_index = 0; value_index < 3; ++value_index) {
            sample = level_sample(21000U);
            if (axis < 3) set_vector_axis(&sample.acceleration_m_s2, axis, nonfinite[value_index]);
            else set_vector_axis(&sample.angular_rate_rad_s, axis - 3, nonfinite[value_index]);
            before = filter;
            check_true(
                aerakia_mahony_update(&filter, &sample, &estimate)
                    == AERAKIA_STATUS_MISSING_MEASUREMENT,
                "Mahony rejects each non-finite required IMU axis"
            );
            check_true(mahony_state_unchanged(&filter, &before),
                       "Mahony rejected IMU value leaves state and time unchanged");
        }
    }

    sample = level_sample(11050U);
    before = filter;
    check_true(aerakia_mahony_update(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "Mahony rejects below-minimum dt");
    check_true(mahony_state_unchanged(&filter, &before),
               "Mahony below-minimum dt does not advance time");

    sample = level_sample(211000U);
    before = filter;
    check_true(aerakia_mahony_update(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "Mahony rejects a forward gap above maximum dt");
    check_true(memcmp(filter.quaternion_wxyz, before.quaternion_wxyz,
                      sizeof(filter.quaternion_wxyz)) == 0,
               "Mahony forward gap does not propagate attitude");
    check_true(filter.last_timestamp_us == sample.timestamp_us,
               "Mahony forward gap re-anchors time");
    sample.timestamp_us += 10000U;
    check_true(aerakia_mahony_update(&filter, &sample, &estimate) == AERAKIA_STATUS_OK,
               "Mahony resumes on the first valid sample after a gap");
}

static void test_eskf_input_integrity(void)
{
    AerakiaEskf filter;
    AerakiaEskf before;
    AerakiaEskf candidate;
    AerakiaEskf control;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000U);
    const float nonfinite[] = {NAN, INFINITY, -INFINITY};
    int axis;
    int value_index;

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.fuse_magnetometer = true;
    config.gate_magnetometer = false;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 11000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    for (axis = 0; axis < 6; ++axis) {
        for (value_index = 0; value_index < 3; ++value_index) {
            sample = level_sample(21000U);
            if (axis < 3) set_vector_axis(&sample.acceleration_m_s2, axis, nonfinite[value_index]);
            else set_vector_axis(&sample.angular_rate_rad_s, axis - 3, nonfinite[value_index]);
            before = filter;
            check_true(
                aerakia_eskf_process_imu(&filter, &sample, &estimate)
                    == AERAKIA_STATUS_MISSING_MEASUREMENT,
                "ESKF rejects each non-finite required IMU axis"
            );
            check_true(eskf_core_unchanged(&filter, &before),
                       "ESKF rejected IMU value leaves state and covariance unchanged");
            check_true(filter.last_timestamp_us == before.last_timestamp_us,
                       "ESKF rejected IMU value leaves time unchanged");
        }
    }

    sample = level_sample(21000U);
    sample.flags &= ~(uint32_t)AERAKIA_SAMPLE_ACCEL_VALID;
    before = filter;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "ESKF rejects a missing accelerometer validity flag");
    check_true(eskf_core_unchanged(&filter, &before),
               "missing accelerometer flag leaves ESKF core unchanged");
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "ESKF rejects a missing gyroscope validity flag");

    sample = level_sample(11000U);
    before = filter;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "ESKF rejects duplicate timestamp");
    check_true(eskf_core_unchanged(&filter, &before)
                   && filter.last_timestamp_us == before.last_timestamp_us,
               "duplicate timestamp leaves ESKF state and time unchanged");
    sample.timestamp_us = 10000U;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "ESKF rejects reversed timestamp");
    sample.timestamp_us = 11050U;
    before = filter;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "ESKF rejects below-minimum dt");
    check_true(eskf_core_unchanged(&filter, &before)
                   && filter.last_timestamp_us == before.last_timestamp_us,
               "ESKF below-minimum dt does not advance state or time");

    sample.timestamp_us = 211000U;
    before = filter;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "ESKF rejects a forward gap above maximum dt");
    check_true(eskf_core_unchanged(&filter, &before),
               "ESKF forward gap does not propagate state or covariance");
    check_true(filter.last_timestamp_us == sample.timestamp_us,
               "ESKF forward gap re-anchors time");
    sample.timestamp_us += 10000U;
    check_true(aerakia_eskf_process_imu(&filter, &sample, &estimate) == AERAKIA_STATUS_OK,
               "ESKF resumes on first valid sample after a gap");

    candidate = filter;
    control = filter;
    sample.timestamp_us += 10000U;
    sample.flags &= ~(uint32_t)AERAKIA_SAMPLE_MAG_VALID;
    check_true(aerakia_eskf_process_imu(&control, &sample, &estimate) == AERAKIA_STATUS_OK,
               "ESKF processes control sample without magnetometer");
    sample.flags |= AERAKIA_SAMPLE_MAG_VALID;
    sample.magnetic_field_ut.x = NAN;
    check_true(aerakia_eskf_process_imu(&candidate, &sample, &estimate) == AERAKIA_STATUS_OK,
               "non-finite optional magnetometer does not reject valid IMU");
    check_true(eskf_core_unchanged(&candidate, &control),
               "non-finite optional magnetometer is equivalent to no magnetometer update");
}

static void test_eskf_process_noise_profile(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaEskfConfig defaults;
    const double invalid_values[] = {NAN, INFINITY, -0.001};
    int field;
    int invalid_index;

    aerakia_eskf_default_config(&defaults);
    check_true(near_double(defaults.process_noise.sigma_acc, 0.1, 1.0e-15)
                   && near_double(defaults.process_noise.sigma_gyr, 0.01, 1.0e-15)
                   && near_double(defaults.process_noise.sigma_acc_bias, 0.001, 1.0e-15)
                   && near_double(defaults.process_noise.sigma_gyr_bias, 0.0001, 1.0e-15),
               "adapter process-noise defaults match the current portable core profile");

    aerakia_eskf_init(&filter, &defaults, NULL, NULL);
    check_true(memcmp(&filter.core.cfg, &defaults.process_noise,
                      sizeof(defaults.process_noise)) == 0,
               "default adapter process-noise profile reaches the ESKF core");

    config = defaults;
    config.process_noise.sigma_acc = 0.24;
    config.process_noise.sigma_gyr = 0.031;
    config.process_noise.sigma_acc_bias = 0.0042;
    config.process_noise.sigma_gyr_bias = 0.00073;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    check_true(memcmp(&filter.config.process_noise, &config.process_noise,
                      sizeof(config.process_noise)) == 0
                   && memcmp(&filter.core.cfg, &config.process_noise,
                             sizeof(config.process_noise)) == 0,
               "custom process-noise profile is retained and applied atomically");

    config = defaults;
    config.process_noise.sigma_acc_bias = 0.0;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    check_true(filter.core.cfg.sigma_acc_bias == 0.0,
               "zero is an explicit valid process-noise configuration");

    for (field = 0; field < 4; ++field) {
        for (invalid_index = 0; invalid_index < 3; ++invalid_index) {
            config = defaults;
            set_process_noise_field(
                &config.process_noise, field, invalid_values[invalid_index]
            );
            aerakia_eskf_init(&filter, &config, NULL, NULL);
            check_true(memcmp(&filter.config.process_noise, &defaults.process_noise,
                              sizeof(defaults.process_noise)) == 0
                           && memcmp(&filter.core.cfg, &defaults.process_noise,
                                     sizeof(defaults.process_noise)) == 0,
                       "non-finite or negative process-noise field falls back to the full default profile");
        }
    }
}

static void test_eskf_timestamped_aiding_integrity(void)
{
    AerakiaEskf filter;
    AerakiaEskf before;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000000U);
    AerakiaGpsObservation gps = {
        .timestamp_us = 1010000U,
        .position_ned_m = {0.0f, 0.0f, 0.0f},
        .velocity_ned_m_s = {0.0f, 0.0f, 0.0f},
        .position_variance_m2 = 1.0f,
        .velocity_variance_m2_s2 = 1.0f,
    };
    AerakiaHeadingObservation heading = {1010000U, 0.0f, 0.01f};
    AerakiaBarometerObservation barometer = {1010000U, 0.0f, 1.0f};

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.maximum_aiding_age_s = 0.05f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "timestamped aiding is rejected before the first IMU timestamp");
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 1010000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    check_true(aerakia_eskf_update_gps_observation(&filter, &gps) == AERAKIA_STATUS_OK,
               "fresh GNSS observation is processed");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading) == AERAKIA_STATUS_OK,
               "fresh trusted-heading observation is processed");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_OK,
               "fresh barometer observation is processed");

    before = filter;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate GNSS observation is rejected");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate trusted-heading observation is rejected");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate barometer observation is rejected");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.barometer_accepted,
               "rejected duplicate clears per-call barometer acceptance");
    check_true(eskf_core_unchanged(&filter, &before),
               "duplicate aiding observations leave state and covariance unchanged");

    gps.timestamp_us = 1009000U;
    heading.timestamp_us = 1009000U;
    barometer.timestamp_us = 1009000U;
    before = filter;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "reordered GNSS observation is rejected");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "reordered trusted-heading observation is rejected");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "reordered barometer observation is rejected");
    check_true(eskf_core_unchanged(&filter, &before),
               "reordered aiding observations leave state and covariance unchanged");

    gps.timestamp_us = 1010001U;
    heading.timestamp_us = 1010001U;
    barometer.timestamp_us = 1010001U;
    before = filter;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "future GNSS observation is rejected");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "future trusted-heading observation is rejected");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "future barometer observation is rejected");
    check_true(eskf_core_unchanged(&filter, &before),
               "future aiding observations leave state and covariance unchanged");

    sample.timestamp_us = 1100000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    gps.timestamp_us = 1020000U;
    heading.timestamp_us = 1020000U;
    barometer.timestamp_us = 1020000U;
    before = filter;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_STALE_MEASUREMENT,
               "stale GNSS observation is rejected");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_STALE_MEASUREMENT,
               "stale trusted-heading observation is rejected");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_STALE_MEASUREMENT,
               "stale barometer observation is rejected");
    check_true(eskf_core_unchanged(&filter, &before),
               "stale aiding observations leave state and covariance unchanged");

    gps.timestamp_us = 1100000U;
    gps.position_ned_m.x = NAN;
    heading.timestamp_us = 1100000U;
    heading.heading_ned_rad = INFINITY;
    barometer.timestamp_us = 1100000U;
    barometer.variance_m2 = NAN;
    before = filter;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "non-finite GNSS observation is rejected");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "non-finite trusted-heading observation is rejected");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "non-finite barometer observation is rejected");
    check_true(eskf_core_unchanged(&filter, &before),
               "non-finite aiding observations leave state and covariance unchanged");

    gps.position_ned_m.x = 0.0f;
    heading.heading_ned_rad = 0.0f;
    barometer.variance_m2 = 1.0f;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps) == AERAKIA_STATUS_OK,
               "GNSS recovers after rejected aiding faults");
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading) == AERAKIA_STATUS_OK,
               "trusted heading recovers after rejected aiding faults");
    check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                   == AERAKIA_STATUS_OK,
               "barometer recovers after rejected aiding faults");
}

static void test_eskf_independent_navigation_observations(void)
{
    AerakiaEskf filter;
    AerakiaEskf before;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000000U);
    AerakiaPositionObservation position = {
        1010000U, {1000.0f, 1000.0f, 1000.0f}, 1.0f
    };
    AerakiaVelocityObservation velocity = {
        1010000U, {100.0f, 100.0f, 100.0f}, 1.0f
    };
    AerakiaGpsObservation paired = {
        .timestamp_us = 1020000U,
        .position_ned_m = {1.0f, 2.0f, 3.0f},
        .velocity_ned_m_s = {0.1f, 0.2f, 0.3f},
        .position_variance_m2 = 1.0f,
        .velocity_variance_m2_s2 = 1.0f,
    };

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.navigation_recovery_rejection_limit = 1U;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 1010000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    check_true(aerakia_eskf_update_position_observation(&filter, &position)
                   == AERAKIA_STATUS_OK,
               "position-only observation is accepted without receiver velocity");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.navigation_recovered, "position-only aid cannot force a re-anchor");
    check_true(near(estimate.position_ned_m.x, 0.0f, 1.0e-4f),
               "rejected position-only aid preserves position");
    check_true(near(estimate.velocity_ned_m_s.x, 0.0f, 1.0e-4f),
               "position-only recovery preserves velocity");

    check_true(aerakia_eskf_update_velocity_observation(&filter, &velocity)
                   == AERAKIA_STATUS_OK,
               "velocity-only observation may share an epoch with position-only aid");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.navigation_recovered, "velocity-only aid cannot force a re-anchor");
    check_true(near(estimate.position_ned_m.x, 0.0f, 1.0e-4f),
               "rejected velocity-only aid preserves position");
    check_true(near(estimate.velocity_ned_m_s.x, 0.0f, 1.0e-4f),
               "rejected velocity-only aid preserves velocity");

    before = filter;
    check_true(aerakia_eskf_update_position_observation(&filter, &position)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate position-only observation is rejected");
    check_true(aerakia_eskf_update_velocity_observation(&filter, &velocity)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate velocity-only observation is rejected");
    check_true(eskf_core_unchanged(&filter, &before),
               "duplicate independent navigation aid leaves the filter unchanged");

    sample.timestamp_us = 1020000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(aerakia_eskf_update_gps_observation(&filter, &paired) == AERAKIA_STATUS_OK,
               "paired GNSS remains compatible after independent observations");
    check_true(filter.last_position_timestamp_us == paired.timestamp_us
                   && filter.last_velocity_timestamp_us == paired.timestamp_us,
               "paired GNSS advances both source-specific timestamps");

    config.navigation_recovery_rejection_limit = 2U;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    sample.timestamp_us = 2000000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 2010000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    position.timestamp_us = 2010000U;
    position.position_ned_m = (AerakiaVec3f){0.0f, 0.0f, 0.0f};
    velocity.timestamp_us = 2010000U;
    velocity.velocity_ned_m_s = (AerakiaVec3f){100.0f, 100.0f, 100.0f};
    (void)aerakia_eskf_update_velocity_observation(&filter, &velocity);
    (void)aerakia_eskf_update_position_observation(&filter, &position);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.navigation_recovered
                   && estimate.consecutive_velocity_rejections == 1U
                   && estimate.consecutive_position_rejections == 0U,
               "accepted position does not clear independent velocity rejection history");

    sample.timestamp_us = 2020000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    position.timestamp_us = 2020000U;
    velocity.timestamp_us = 2020000U;
    (void)aerakia_eskf_update_position_observation(&filter, &position);
    (void)aerakia_eskf_update_velocity_observation(&filter, &velocity);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.navigation_recovered
                   && estimate.consecutive_velocity_rejections == 2U,
               "standalone velocity rejection cannot bypass recovery supervision");
}

static void test_eskf_horizontal_navigation_validity_timeout(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(0U);
    AerakiaPositionObservation position = {
        10000U, {0.0f, 0.0f, 0.0f}, 1.0f
    };
    AerakiaVelocityObservation velocity = {
        10000U, {0.0f, 0.0f, 0.0f}, 1.0f
    };

    aerakia_eskf_default_config(&config);
    check_true(near(config.maximum_horizontal_position_dead_reckoning_s, 5.0f, 1.0e-6f)
                   && near(config.maximum_horizontal_velocity_dead_reckoning_s, 5.0f, 1.0e-6f),
               "default independent horizontal validity limits are five seconds");
    config.enable_static_alignment = false;
    config.navigation_recovery_rejection_limit = 0U;
    config.maximum_horizontal_position_dead_reckoning_s = 0.05f;
    config.maximum_horizontal_velocity_dead_reckoning_s = 0.05f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);

    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 10000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(!estimate.horizontal_navigation_valid
                   && !estimate.horizontal_position_valid
                   && !estimate.horizontal_velocity_valid
                   && isinf(estimate.horizontal_aiding_age_s)
                   && isinf(estimate.horizontal_position_aiding_age_s)
                   && isinf(estimate.horizontal_velocity_aiding_age_s),
               "navigation is invalid before any accepted horizontal constraint");

    check_true(aerakia_eskf_update_velocity_observation(&filter, &velocity)
                   == AERAKIA_STATUS_OK,
               "velocity-only aiding is accepted before a position origin exists");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.horizontal_navigation_valid
                   && !estimate.horizontal_position_valid
                   && estimate.horizontal_velocity_valid
                   && isinf(estimate.horizontal_position_aiding_age_s)
                   && near(estimate.horizontal_velocity_aiding_age_s, 0.0f, 1.0e-6f),
               "velocity-only aiding cannot invent a horizontal position origin");

    check_true(aerakia_eskf_update_position_observation(&filter, &position)
                   == AERAKIA_STATUS_OK,
               "accepted position initializes horizontal navigation validity");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.horizontal_navigation_valid
                   && estimate.horizontal_position_valid
                   && estimate.horizontal_velocity_valid
                   && near(estimate.horizontal_position_aiding_age_s, 0.0f, 1.0e-6f)
                   && near(estimate.horizontal_velocity_aiding_age_s, 0.0f, 1.0e-6f),
               "fresh independent position and velocity qualify navigation");

    sample.timestamp_us = 50000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(estimate.horizontal_navigation_valid
                   && near(estimate.horizontal_aiding_age_s, 0.04f, 1.0e-6f),
               "navigation remains valid inside the no-aiding interval");
    sample.timestamp_us = 70000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(!estimate.horizontal_navigation_valid
                   && !estimate.horizontal_position_valid
                   && !estimate.horizontal_velocity_valid
                   && near(estimate.horizontal_aiding_age_s, 0.06f, 1.0e-6f)
                   && estimate.healthy,
               "finite ESKF state is distinguished from stale navigation validity");

    position.timestamp_us = 70000U;
    position.position_ned_m.x = 1000.0f;
    position.variance_m2 = 0.01f;
    check_true(aerakia_eskf_update_position_observation(&filter, &position)
                   == AERAKIA_STATUS_OK,
               "fresh but inconsistent position reaches the innovation gate");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.position_accepted
                   && !estimate.horizontal_navigation_valid
                   && near(estimate.horizontal_aiding_age_s, 0.06f, 1.0e-6f),
               "rejected position cannot refresh horizontal validity");

    velocity.timestamp_us = 70000U;
    check_true(aerakia_eskf_update_velocity_observation(&filter, &velocity)
                   == AERAKIA_STATUS_OK,
               "accepted velocity can refresh an initialized horizontal solution");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.velocity_accepted && !estimate.horizontal_navigation_valid
                   && !estimate.horizontal_position_valid
                   && estimate.horizontal_velocity_valid
                   && near(estimate.horizontal_position_aiding_age_s, 0.06f, 1.0e-6f)
                   && near(estimate.horizontal_velocity_aiding_age_s, 0.0f, 1.0e-6f),
               "accepted velocity refreshes only velocity validity");

    sample.timestamp_us = 80000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    position.timestamp_us = 80000U;
    position.position_ned_m.x = 0.0f;
    position.variance_m2 = 1.0f;
    (void)aerakia_eskf_update_position_observation(&filter, &position);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.horizontal_navigation_valid
                   && estimate.horizontal_position_valid
                   && estimate.horizontal_velocity_valid,
               "fresh position restores navigation while velocity remains independently fresh");

    sample.timestamp_us = 130000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(!estimate.horizontal_navigation_valid && estimate.horizontal_position_valid
                   && !estimate.horizontal_velocity_valid,
               "horizontal velocity expires independently from position");
    aerakia_eskf_apply_zero_velocity(&filter, 0.01f);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.horizontal_navigation_valid
                   && near(estimate.horizontal_position_aiding_age_s, 0.05f, 1.0e-6f)
                   && near(estimate.horizontal_velocity_aiding_age_s, 0.0f, 1.0e-6f),
               "zero velocity refreshes velocity without refreshing position");

    sample.timestamp_us = 200000U;
    filter.config.maximum_horizontal_position_dead_reckoning_s = -1.0f;
    filter.config.maximum_horizontal_velocity_dead_reckoning_s = -1.0f;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(estimate.horizontal_navigation_valid
                   && near(estimate.horizontal_aiding_age_s, 0.12f, 1.0e-6f),
               "negative timeout explicitly disables age-based invalidation");
    filter.config.maximum_horizontal_position_dead_reckoning_s = 0.0f;
    filter.config.maximum_horizontal_velocity_dead_reckoning_s = 0.0f;
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.horizontal_navigation_valid,
               "zero timeout accepts only a same-timestamp horizontal constraint");
}

static void test_eskf_aiding_numeric_exhaustive(void)
{
    AerakiaEskf filter;
    AerakiaEskf before;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000000U);
    const float nonfinite[] = {NAN, INFINITY, -INFINITY};
    const float bad_variance[] = {NAN, INFINITY, -INFINITY, 0.0f};
    AerakiaGpsObservation gps = {
        .timestamp_us = 1010000U,
        .position_ned_m = {1.0f, 2.0f, 3.0f},
        .velocity_ned_m_s = {0.1f, 0.2f, 0.3f},
        .position_variance_m2 = 1.0f,
        .velocity_variance_m2_s2 = 1.0f,
    };
    AerakiaHeadingObservation heading = {1010000U, 0.1f, 0.01f};
    AerakiaBarometerObservation barometer = {1010000U, 2.0f, 1.0f};
    int axis;
    int value_index;

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 1010000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    for (axis = 0; axis < 6; ++axis) {
        for (value_index = 0; value_index < 3; ++value_index) {
            gps.position_ned_m = (AerakiaVec3f){1.0f, 2.0f, 3.0f};
            gps.velocity_ned_m_s = (AerakiaVec3f){0.1f, 0.2f, 0.3f};
            if (axis < 3) set_vector_axis(&gps.position_ned_m, axis, nonfinite[value_index]);
            else set_vector_axis(&gps.velocity_ned_m_s, axis - 3, nonfinite[value_index]);
            before = filter;
            check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                           == AERAKIA_STATUS_MISSING_MEASUREMENT,
                       "GNSS rejects NaN and signed infinity on every vector axis");
            check_true(eskf_core_unchanged(&filter, &before),
                       "invalid GNSS vector leaves state and covariance unchanged");
        }
    }
    gps.position_ned_m = (AerakiaVec3f){1.0f, 2.0f, 3.0f};
    gps.velocity_ned_m_s = (AerakiaVec3f){0.1f, 0.2f, 0.3f};
    for (axis = 0; axis < 2; ++axis) {
        for (value_index = 0; value_index < 4; ++value_index) {
            gps.position_variance_m2 = 1.0f;
            gps.velocity_variance_m2_s2 = 1.0f;
            if (axis == 0) gps.position_variance_m2 = bad_variance[value_index];
            else gps.velocity_variance_m2_s2 = bad_variance[value_index];
            before = filter;
            check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                           == AERAKIA_STATUS_MISSING_MEASUREMENT,
                       "GNSS rejects non-finite and non-positive variances");
            check_true(eskf_core_unchanged(&filter, &before),
                       "invalid GNSS variance leaves state and covariance unchanged");
        }
    }

    for (value_index = 0; value_index < 3; ++value_index) {
        heading.heading_ned_rad = nonfinite[value_index];
        heading.variance_rad2 = 0.01f;
        before = filter;
        check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                       == AERAKIA_STATUS_MISSING_MEASUREMENT,
                   "trusted heading rejects NaN and signed infinity");
        check_true(eskf_core_unchanged(&filter, &before),
                   "invalid trusted heading leaves state and covariance unchanged");
        barometer.height_up_m = nonfinite[value_index];
        barometer.variance_m2 = 1.0f;
        before = filter;
        check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                       == AERAKIA_STATUS_MISSING_MEASUREMENT,
                   "barometer rejects NaN and signed infinity");
        check_true(eskf_core_unchanged(&filter, &before),
                   "invalid barometer height leaves state and covariance unchanged");
    }
    heading.heading_ned_rad = 0.1f;
    barometer.height_up_m = 2.0f;
    for (value_index = 0; value_index < 4; ++value_index) {
        heading.variance_rad2 = bad_variance[value_index];
        before = filter;
        check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                       == AERAKIA_STATUS_MISSING_MEASUREMENT,
                   "trusted heading rejects non-finite and non-positive variance");
        check_true(eskf_core_unchanged(&filter, &before),
                   "invalid heading variance leaves state and covariance unchanged");
        barometer.variance_m2 = bad_variance[value_index];
        before = filter;
        check_true(aerakia_eskf_update_barometer_observation(&filter, &barometer)
                       == AERAKIA_STATUS_MISSING_MEASUREMENT,
                   "barometer rejects non-finite and non-positive variance");
        check_true(eskf_core_unchanged(&filter, &before),
                   "invalid barometer variance leaves state and covariance unchanged");
    }
}

static void test_eskf_adapter_stationary(void)
{
    AerakiaEskf filter;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(0U);
    int index;

    aerakia_eskf_init(&filter, NULL, NULL, NULL);
    check_true(
        aerakia_eskf_process_imu(&filter, &sample, &estimate) == AERAKIA_STATUS_INITIALIZED,
        "ESKF adapter initializes timestamp"
    );
    for (index = 1; index <= 100; ++index) {
        sample.timestamp_us = (uint64_t)index * 10000U;
        check_true(
            aerakia_eskf_process_imu(&filter, &sample, &estimate) == AERAKIA_STATUS_OK,
            "ESKF stationary sample processes"
        );
    }
    check_true(near(estimate.position_ned_m.z, 0.0f, 1.0e-6f), "ESKF stationary down position remains zero");
    check_true(near(estimate.velocity_ned_m_s.z, 0.0f, 1.0e-6f), "ESKF stationary down velocity remains zero");
    check_true(estimate.healthy, "ESKF adapter covariance remains healthy");
}

static void test_eskf_health_covers_state_and_covariance(void)
{
    AerakiaEskf filter;
    AerakiaNavigationEstimate estimate;

    aerakia_eskf_init(&filter, NULL, NULL, NULL);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.healthy, "fresh ESKF state and covariance are healthy");

    filter.core.state.p[0] = NAN;
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.healthy, "non-finite ESKF nominal state is unhealthy");

    filter.core.state.p[0] = 0.0;
    filter.core.state.q[0] = 2.0;
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.healthy, "non-unit ESKF quaternion is unhealthy");

    filter.core.state.q[0] = 1.0;
    filter.core.P[0][0] = NAN;
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.healthy, "non-finite ESKF covariance is unhealthy");
}

static void test_eskf_static_supervisor(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(0U);
    int index;

    aerakia_eskf_default_config(&config);
    config.static_alignment_duration_s = 0.08f;
    config.static_alignment_min_samples = 10U;
    config.zero_velocity_interval_s = 0.05f;
    sample.flags |= AERAKIA_SAMPLE_STATIONARY;
    sample.angular_rate_rad_s.z = 0.01f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    for (index = 0; index < 10; ++index) {
        AerakiaStatus status;
        sample.timestamp_us = (uint64_t)index * 10000U;
        status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
        check_true(
            status == AERAKIA_STATUS_ALIGNING || status == AERAKIA_STATUS_INITIALIZED,
            "stationary samples are held for static alignment"
        );
    }
    check_true(estimate.static_alignment_complete, "adapter completes static alignment");
    check_true(estimate.static_tilt_alignment_complete, "adapter completes tilt alignment");
    check_true(!estimate.static_heading_alignment_complete,
               "adapter reports missing magnetic heading alignment");
    aerakia_eskf_update_heading(&filter, 0.0f, 0.01f);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.heading_accepted, "trusted heading is accepted after tilt alignment");
    check_true(estimate.static_heading_alignment_complete,
               "trusted heading completes cold-start heading alignment");
    check_true(near(estimate.gyroscope_bias_rad_s.z, 0.01f, 1.0e-6f), "adapter estimates gyro bias");
    for (index = 10; index <= 20; ++index) {
        sample.timestamp_us = (uint64_t)index * 10000U;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    }
    check_true(estimate.zero_velocity_update_count >= 2U, "stationary supervisor applies periodic ZUPT");
}

static void test_eskf_navigation_recovery_and_heading(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000000U);
    AerakiaGpsObservation observation = {
        .timestamp_us = 1010000U,
        .position_ned_m = {50.0f, 0.0f, 0.0f},
        .velocity_ned_m_s = {10.0f, 0.0f, 0.0f},
        .position_variance_m2 = 1.0f,
        .velocity_variance_m2_s2 = 1.0f,
    };
    AerakiaNavigationRecoveryAuthorization authorization = {
        .observation_timestamp_us = 1010000U,
        .source_quality_verified = false,
        .maximum_position_correction_m = 60.0f,
        .maximum_velocity_correction_m_s = 15.0f,
    };
    double initial_q[4] = {cos(0.1), 0.0, 0.0, sin(0.1)};
    float yaw_before;
    int index;

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.navigation_recovery_rejection_limit = 3U;
    config.navigation_recovery_min_consistent_observations = 3U;
    config.navigation_recovery_probationary_acceptances = 3U;
    aerakia_eskf_init(&filter, &config, NULL, initial_q);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    observation.source_id = 7U;
    observation.source_generation = 2U;
    observation.quality_sequence = 11U;
    for (index = 0; index < 5; ++index) {
        sample.timestamp_us = index < 2
            ? 1100000U + (uint64_t)index * 100000U
            : 2000000U + (uint64_t)(index - 2) * 100000U;
        observation.timestamp_us = sample.timestamp_us;
        observation.position_ned_m.x = 50.0f + 0.10f * (float)index;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
        check_true(aerakia_eskf_update_gps_observation(&filter, &observation)
                       == AERAKIA_STATUS_OK,
                   "consistent rejected GNSS candidate is recorded");
        if (index == 2) {
            aerakia_eskf_get_estimate(&filter, &estimate);
            check_true(estimate.recovery_candidate_consistent_observations == 1U
                           && !estimate.navigation_recovery_candidate_ready,
                       "candidate gap above 0.3 seconds restarts the evidence window");
        }
    }
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.navigation_recovered
                   && estimate.navigation_recovery_candidate_ready
                   && estimate.recovery_candidate_consistent_observations == 3U
                   && near(estimate.recovery_candidate_duration_s, 0.2f, 1.0e-6f),
               "candidate requires count, continuity, and minimum physical duration");
    {
        AerakiaEskf changed_quality_filter = filter;
        AerakiaGpsObservation changed_quality = observation;
        AerakiaImuSample changed_quality_sample = sample;
        changed_quality_sample.timestamp_us += 100000U;
        changed_quality.timestamp_us = changed_quality_sample.timestamp_us;
        changed_quality.quality_sequence++;
        (void)aerakia_eskf_process_imu(
            &changed_quality_filter, &changed_quality_sample, NULL
        );
        (void)aerakia_eskf_update_gps_observation(
            &changed_quality_filter, &changed_quality
        );
        aerakia_eskf_get_estimate(&changed_quality_filter, &estimate);
        check_true(estimate.recovery_candidate_consistent_observations == 1U
                       && !estimate.navigation_recovery_candidate_ready,
                   "quality snapshot change restarts the candidate evidence window");
    }

    authorization.observation_timestamp_us = observation.timestamp_us;
    authorization.source_id = observation.source_id;
    authorization.source_generation = observation.source_generation;
    authorization.quality_sequence = observation.quality_sequence;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_RECOVERY_REJECTED,
               "recovery requires explicit physical source-quality attestation");
    authorization.source_quality_verified = true;
    authorization.source_id++;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_RECOVERY_REJECTED,
               "authorization is bound to the exact physical source");
    authorization.source_id = observation.source_id;
    authorization.source_generation++;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_RECOVERY_REJECTED,
               "authorization is bound to the exact source generation");
    authorization.source_generation = observation.source_generation;
    authorization.quality_sequence++;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_RECOVERY_REJECTED,
               "authorization is bound to the exact quality snapshot");
    authorization.quality_sequence = observation.quality_sequence;
    {
        AerakiaEskf stale_filter = filter;
        AerakiaImuSample stale_sample = sample;
        stale_sample.timestamp_us += 300000U;
        (void)aerakia_eskf_process_imu(&stale_filter, &stale_sample, NULL);
        check_true(aerakia_eskf_authorize_navigation_recovery(&stale_filter, &authorization)
                       == AERAKIA_STATUS_NOT_READY,
                   "authorization older than 0.25 seconds fails closed");
        aerakia_eskf_get_estimate(&stale_filter, &estimate);
        check_true(!estimate.navigation_recovery_candidate_ready,
                   "stale candidate is not advertised as ready");
    }
    authorization.maximum_position_correction_m = 10.0f;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_RECOVERY_REJECTED,
               "supervisor authorization cannot exceed the correction bound");
    authorization.maximum_position_correction_m = 60.0f;
    check_true(aerakia_eskf_authorize_navigation_recovery(&filter, &authorization)
                   == AERAKIA_STATUS_OK,
               "quality-verified consistent bounded recovery is applied once");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.navigation_recovered && estimate.navigation_recovery_probationary,
               "authorized reset enters probation rather than declaring navigation valid");
    check_true(estimate.navigation_recovery_count == 1U, "navigation recovery is counted");
    check_true(near(estimate.position_ned_m.x, 50.4f, 1.0e-4f),
               "authorized recovery uses the timestamp-bound candidate");
    check_true(!estimate.horizontal_navigation_valid,
               "probationary recovery does not qualify controller-facing navigation");

    sample.timestamp_us += 100000U;
    observation.timestamp_us = sample.timestamp_us;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    {
        int mismatch_index;
        const AerakiaEskf probation_before_mismatch = filter;
        for (mismatch_index = 0; mismatch_index < 3; ++mismatch_index) {
            AerakiaEskf attempted_filter = probation_before_mismatch;
            AerakiaGpsObservation mismatched_observation = observation;
            if (mismatch_index == 0) mismatched_observation.source_id++;
            else if (mismatch_index == 1) mismatched_observation.source_generation++;
            else mismatched_observation.quality_sequence++;

            check_true(aerakia_eskf_update_gps_observation(
                           &attempted_filter, &mismatched_observation)
                           == AERAKIA_STATUS_RECOVERY_REJECTED,
                       "mismatched probation source stamp is rejected before GPS fusion");
            check_true(eskf_core_unchanged(&attempted_filter, &probation_before_mismatch),
                       "mismatched probation source stamp leaves state and full covariance unchanged");
            check_true(attempted_filter.last_gps_timestamp_us
                           == probation_before_mismatch.last_gps_timestamp_us
                           && attempted_filter.last_position_timestamp_us
                               == probation_before_mismatch.last_position_timestamp_us
                           && attempted_filter.last_velocity_timestamp_us
                               == probation_before_mismatch.last_velocity_timestamp_us,
                       "mismatched probation source stamp does not consume GPS timestamps");
            check_true(memcmp(&attempted_filter, &probation_before_mismatch,
                              sizeof(attempted_filter)) == 0,
                       "mismatched probation source stamp leaves innovations and all bookkeeping unchanged");
        }
    }

    for (index = 0; index < 2; ++index) {
        sample.timestamp_us += 100000U;
        observation.timestamp_us = sample.timestamp_us;
        observation.position_ned_m.x += 0.10f;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
        (void)aerakia_eskf_update_gps_observation(&filter, &observation);
        aerakia_eskf_get_estimate(&filter, &estimate);
        check_true(estimate.navigation_recovery_probationary
                       && !estimate.horizontal_navigation_valid,
                   "rapid accepted pairs cannot bypass minimum probation duration");
    }

    sample.timestamp_us += 400000U;
    observation.timestamp_us = sample.timestamp_us;
    observation.position_ned_m.x += 0.40f;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    (void)aerakia_eskf_update_gps_observation(&filter, &observation);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.navigation_recovery_probationary
                   && estimate.navigation_recovery_probation_acceptances == 1U,
               "probation update gap above 0.3 seconds restarts the dwell");

    for (index = 0; index < 3; ++index) {
        sample.timestamp_us += 100000U;
        observation.timestamp_us = sample.timestamp_us;
        observation.position_ned_m.x += 0.10f;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
        (void)aerakia_eskf_update_gps_observation(&filter, &observation);
        aerakia_eskf_get_estimate(&filter, &estimate);
    }
    check_true(!estimate.navigation_recovery_probationary
                   && estimate.horizontal_navigation_valid,
               "configured accepted-update dwell exits recovery probation");

    yaw_before = fabsf(estimate.attitude.euler_rad.z);
    aerakia_eskf_update_heading(&filter, 0.0f, 0.01f);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.heading_accepted, "adapter accepts trusted heading");
    check_true(fabsf(estimate.attitude.euler_rad.z) < yaw_before, "adapter heading reduces yaw error");
}

static void test_eskf_cold_start_attitude_alignment(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample;
    float q[4];
    const float roll = 25.0f * AERAKIA_PI_F / 180.0f;
    const float pitch = -18.0f * AERAKIA_PI_F / 180.0f;
    const float yaw = 35.0f * AERAKIA_PI_F / 180.0f;
    const AerakiaVec3f acceleration_ned = {0.0f, 0.0f, -AERAKIA_GRAVITY_M_S2};
    const AerakiaVec3f magnetic_ned = {22.0f, 0.0f, 44.0f};
    int index;

    euler_quaternion(roll, pitch, yaw, q);
    memset(&sample, 0, sizeof(sample));
    sample.acceleration_m_s2 = ned_to_body(q, acceleration_ned);
    sample.magnetic_field_ut = ned_to_body(q, magnetic_ned);
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID
        | AERAKIA_SAMPLE_MAG_VALID | AERAKIA_SAMPLE_STATIONARY;
    aerakia_eskf_default_config(&config);
    config.fuse_magnetometer = true;
    config.gate_magnetometer = false;
    config.magnetic_reference_ned[0] = magnetic_ned.x;
    config.magnetic_reference_ned[1] = magnetic_ned.y;
    config.magnetic_reference_ned[2] = magnetic_ned.z;
    config.static_alignment_duration_s = 0.08f;
    config.static_alignment_min_samples = 10U;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    for (index = 0; index < 10; ++index) {
        sample.timestamp_us = (uint64_t)index * 10000U;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    }
    check_true(estimate.static_alignment_complete, "cold start completes static alignment");
    check_true(estimate.static_tilt_alignment_complete, "cold start aligns tilt");
    check_true(estimate.static_heading_alignment_complete, "cold start aligns heading");
    check_true(near(estimate.attitude.euler_rad.x, roll, 2.0e-5f),
               "cold start recovers roll");
    check_true(near(estimate.attitude.euler_rad.y, pitch, 2.0e-5f),
               "cold start recovers pitch");
    check_true(near(estimate.attitude.euler_rad.z, yaw, 2.0e-5f),
               "cold start recovers yaw");
}

static void test_eskf_heading_and_vertical_validity(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(0U);
    AerakiaHeadingObservation heading = {10000U, 0.0f, 0.01f};
    AerakiaBarometerObservation barometer = {10000U, 0.0f, 0.25f};
    AerakiaVelocityObservation velocity = {10000U, {0.0f, 0.0f, 0.0f}, 0.04f};

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.maximum_heading_dead_reckoning_s = 0.05f;
    config.maximum_vertical_position_dead_reckoning_s = 0.05f;
    config.maximum_vertical_velocity_dead_reckoning_s = 0.05f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    sample.timestamp_us = 10000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(!estimate.heading_valid && !estimate.vertical_position_valid
                   && !estimate.vertical_velocity_valid
                   && isinf(estimate.heading_aiding_age_s)
                   && isinf(estimate.vertical_position_aiding_age_s)
                   && isinf(estimate.vertical_velocity_aiding_age_s),
               "heading and vertical outputs start invalid without accepted aiding");

    (void)aerakia_eskf_update_heading_observation(&filter, &heading);
    (void)aerakia_eskf_update_barometer_observation(&filter, &barometer);
    (void)aerakia_eskf_update_velocity_observation(&filter, &velocity);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.heading_valid && estimate.vertical_position_valid
                   && estimate.vertical_velocity_valid && estimate.vertical_navigation_valid,
               "accepted heading, barometer, and velocity independently qualify outputs");
    check_true(estimate.barometer_accepted
                   && near(estimate.heading_aiding_age_s, 0.0f, 1.0e-6f)
                   && near(estimate.vertical_position_aiding_age_s, 0.0f, 1.0e-6f)
                   && near(estimate.vertical_velocity_aiding_age_s, 0.0f, 1.0e-6f),
               "continuous aiding ages report the physical observation epoch");

    aerakia_eskf_note_barometer_rejection(&filter);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(!estimate.barometer_accepted && estimate.vertical_position_valid,
               "prefilter rejection clears only per-attempt barometer acceptance");

    sample.timestamp_us = 70000U;
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(!estimate.heading_valid && !estimate.vertical_position_valid
                   && !estimate.vertical_velocity_valid && !estimate.vertical_navigation_valid,
               "heading and vertical validity expire independently after aiding loss");
    check_true(near(estimate.heading_aiding_age_s, 0.06f, 1.0e-6f)
                   && near(estimate.vertical_position_aiding_age_s, 0.06f, 1.0e-6f)
                   && near(estimate.vertical_velocity_aiding_age_s, 0.06f, 1.0e-6f),
               "expired outputs retain inspectable aiding ages");
}

int main(void)
{
    test_mahony_level_initialization();
    test_mahony_yaw_integration();
    test_mahony_trusted_attitude_seed();
    test_mahony_adaptive_weight_and_timestamp();
    test_mahony_input_integrity();
    test_eskf_process_noise_profile();
    test_eskf_input_integrity();
    test_eskf_timestamped_aiding_integrity();
    test_eskf_independent_navigation_observations();
    test_eskf_horizontal_navigation_validity_timeout();
    test_eskf_aiding_numeric_exhaustive();
    test_eskf_adapter_stationary();
    test_eskf_health_covers_state_and_covariance();
    test_eskf_static_supervisor();
    test_eskf_navigation_recovery_and_heading();
    test_eskf_cold_start_attitude_alignment();
    test_eskf_heading_and_vertical_validity();

    if (failures != 0) {
        fprintf(stderr, "%d public API assertion(s) failed\n", failures);
        return 1;
    }
    puts("Public API tests passed");
    return 0;
}
