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

int main(void)
{
    test_mahony_level_initialization();
    test_mahony_yaw_integration();
    test_mahony_adaptive_weight_and_timestamp();
    test_eskf_adapter_stationary();

    if (failures != 0) {
        fprintf(stderr, "%d public API assertion(s) failed\n", failures);
        return 1;
    }
    puts("Public API tests passed");
    return 0;
}
