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
    sample.flags &= ~AERAKIA_SAMPLE_ACCEL_VALID;
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
    sample.flags &= ~AERAKIA_SAMPLE_MAG_VALID;
    check_true(aerakia_eskf_process_imu(&control, &sample, &estimate) == AERAKIA_STATUS_OK,
               "ESKF processes control sample without magnetometer");
    sample.flags |= AERAKIA_SAMPLE_MAG_VALID;
    sample.magnetic_field_ut.x = NAN;
    check_true(aerakia_eskf_process_imu(&candidate, &sample, &estimate) == AERAKIA_STATUS_OK,
               "non-finite optional magnetometer does not reject valid IMU");
    check_true(eskf_core_unchanged(&candidate, &control),
               "non-finite optional magnetometer is equivalent to no magnetometer update");
}

static void test_eskf_timestamped_aiding_integrity(void)
{
    AerakiaEskf filter;
    AerakiaEskf before;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    AerakiaImuSample sample = level_sample(1000000U);
    AerakiaGpsObservation gps = {
        1010000U, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 1.0f, 1.0f
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
        1020000U, {1.0f, 2.0f, 3.0f}, {0.1f, 0.2f, 0.3f}, 1.0f, 1.0f
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
    check_true(estimate.navigation_recovered, "rejected position-only aid reanchors position");
    check_true(near(estimate.position_ned_m.x, 1000.0f, 1.0e-4f),
               "position-only recovery changes position");
    check_true(near(estimate.velocity_ned_m_s.x, 0.0f, 1.0e-4f),
               "position-only recovery preserves velocity");

    check_true(aerakia_eskf_update_velocity_observation(&filter, &velocity)
                   == AERAKIA_STATUS_OK,
               "velocity-only observation may share an epoch with position-only aid");
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.navigation_recovered, "rejected velocity-only aid reanchors velocity");
    check_true(near(estimate.position_ned_m.x, 1000.0f, 1.0e-4f),
               "velocity-only recovery preserves position");
    check_true(near(estimate.velocity_ned_m_s.x, 100.0f, 1.0e-4f),
               "velocity-only recovery changes velocity");

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
    check_true(estimate.navigation_recovered
                   && estimate.consecutive_velocity_rejections == 0U,
               "velocity recovery triggers despite interleaved accepted positions");
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
        1010000U, {1.0f, 2.0f, 3.0f}, {0.1f, 0.2f, 0.3f}, 1.0f, 1.0f
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
    AerakiaVec3f position = {1000.0f, 1000.0f, 1000.0f};
    AerakiaVec3f velocity = {100.0f, 100.0f, 100.0f};
    double initial_q[4] = {cos(0.1), 0.0, 0.0, sin(0.1)};
    float yaw_before;

    aerakia_eskf_default_config(&config);
    config.navigation_recovery_rejection_limit = 2U;
    aerakia_eskf_init(&filter, &config, NULL, initial_q);
    aerakia_eskf_update_gps(&filter, position, velocity, 1.0f, 1.0f);
    aerakia_eskf_update_gps(&filter, position, velocity, 1.0f, 1.0f);
    aerakia_eskf_get_estimate(&filter, &estimate);
    check_true(estimate.navigation_recovered, "paired GPS rejection triggers navigation recovery");
    check_true(estimate.navigation_recovery_count == 1U, "navigation recovery is counted");
    check_true(near(estimate.position_ned_m.x, 1000.0f, 1.0e-4f), "recovery reanchors position");

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

int main(void)
{
    test_mahony_level_initialization();
    test_mahony_yaw_integration();
    test_mahony_trusted_attitude_seed();
    test_mahony_adaptive_weight_and_timestamp();
    test_mahony_input_integrity();
    test_eskf_input_integrity();
    test_eskf_timestamped_aiding_integrity();
    test_eskf_independent_navigation_observations();
    test_eskf_aiding_numeric_exhaustive();
    test_eskf_adapter_stationary();
    test_eskf_health_covers_state_and_covariance();
    test_eskf_static_supervisor();
    test_eskf_navigation_recovery_and_heading();
    test_eskf_cold_start_attitude_alignment();

    if (failures != 0) {
        fprintf(stderr, "%d public API assertion(s) failed\n", failures);
        return 1;
    }
    puts("Public API tests passed");
    return 0;
}
