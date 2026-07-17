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
    test_eskf_adapter_stationary();
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
