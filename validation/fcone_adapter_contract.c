/**
 * @file fcone_adapter_contract.c
 * @brief Hardware-free oracle for the private FCOne-to-Aerakia message adapter.
 *
 * Board sensor axes and calibration remain private. This test starts after those transforms, at a
 * mock FCOne publication expressed in FRD engineering units, and verifies conversion into the
 * public SI contract plus timestamp/validity behavior in the real estimator adapter.
 */

#include <aerakia/eskf_adapter.h>
#include <aerakia/types.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint64_t physical_timestamp_us;
    AerakiaVec3f acceleration_frd_g;
    AerakiaVec3f angular_rate_frd_deg_s;
    AerakiaVec3f magnetic_field_frd_gauss;
    bool acceleration_valid;
    bool angular_rate_valid;
    bool magnetic_field_valid;
    bool stationary;
} MockFcOneEstimatorPublication;

typedef struct {
    uint64_t physical_timestamp_us;
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
    float position_variance_m2;
    float velocity_variance_m2_s2;
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
} MockFcOneNavigationPublication;

static int failures;

static void check_true(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures += 1;
    }
}

static bool close_float(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static AerakiaImuSample adapt_mock_publication(const MockFcOneEstimatorPublication *publication)
{
    AerakiaImuSample sample;
    const float degrees_to_radians = AERAKIA_PI_F / 180.0f;
    memset(&sample, 0, sizeof(sample));
    sample.timestamp_us = publication->physical_timestamp_us;
    sample.acceleration_m_s2.x = publication->acceleration_frd_g.x * AERAKIA_GRAVITY_M_S2;
    sample.acceleration_m_s2.y = publication->acceleration_frd_g.y * AERAKIA_GRAVITY_M_S2;
    sample.acceleration_m_s2.z = publication->acceleration_frd_g.z * AERAKIA_GRAVITY_M_S2;
    sample.angular_rate_rad_s.x = publication->angular_rate_frd_deg_s.x * degrees_to_radians;
    sample.angular_rate_rad_s.y = publication->angular_rate_frd_deg_s.y * degrees_to_radians;
    sample.angular_rate_rad_s.z = publication->angular_rate_frd_deg_s.z * degrees_to_radians;
    sample.magnetic_field_ut.x = publication->magnetic_field_frd_gauss.x * 100.0f;
    sample.magnetic_field_ut.y = publication->magnetic_field_frd_gauss.y * 100.0f;
    sample.magnetic_field_ut.z = publication->magnetic_field_frd_gauss.z * 100.0f;
    if (publication->acceleration_valid) sample.flags |= AERAKIA_SAMPLE_ACCEL_VALID;
    if (publication->angular_rate_valid) sample.flags |= AERAKIA_SAMPLE_GYRO_VALID;
    if (publication->magnetic_field_valid) sample.flags |= AERAKIA_SAMPLE_MAG_VALID;
    if (publication->stationary) sample.flags |= AERAKIA_SAMPLE_STATIONARY;
    return sample;
}

static AerakiaGpsObservation adapt_mock_navigation(
    const MockFcOneNavigationPublication *publication
)
{
    AerakiaGpsObservation observation = {0};
    observation.timestamp_us = publication->physical_timestamp_us;
    observation.position_ned_m = publication->position_ned_m;
    observation.velocity_ned_m_s = publication->velocity_ned_m_s;
    observation.position_variance_m2 = publication->position_variance_m2;
    observation.velocity_variance_m2_s2 = publication->velocity_variance_m2_s2;
    observation.source_id = publication->source_id;
    observation.source_generation = publication->source_generation;
    observation.quality_sequence = publication->quality_sequence;
    return observation;
}

static MockFcOneEstimatorPublication stationary_publication(uint64_t timestamp_us)
{
    MockFcOneEstimatorPublication publication;
    memset(&publication, 0, sizeof(publication));
    publication.physical_timestamp_us = timestamp_us;
    publication.acceleration_frd_g.z = -1.0f;
    publication.magnetic_field_frd_gauss.x = 0.22f;
    publication.magnetic_field_frd_gauss.z = 0.44f;
    publication.acceleration_valid = true;
    publication.angular_rate_valid = true;
    publication.magnetic_field_valid = true;
    publication.stationary = true;
    return publication;
}

static void test_unit_frame_flag_mapping(void)
{
    MockFcOneEstimatorPublication publication;
    AerakiaImuSample sample;
    memset(&publication, 0, sizeof(publication));
    publication.physical_timestamp_us = 1234567U;
    publication.acceleration_frd_g = (AerakiaVec3f){1.0f, -2.0f, 0.5f};
    publication.angular_rate_frd_deg_s = (AerakiaVec3f){180.0f, -90.0f, 45.0f};
    publication.magnetic_field_frd_gauss = (AerakiaVec3f){0.2f, -0.3f, 0.45f};
    publication.acceleration_valid = true;
    publication.angular_rate_valid = true;
    publication.stationary = true;

    sample = adapt_mock_publication(&publication);
    check_true(sample.timestamp_us == publication.physical_timestamp_us,
               "physical sensor timestamp is preserved");
    check_true(close_float(sample.acceleration_m_s2.x, AERAKIA_GRAVITY_M_S2, 1.0e-6f)
               && close_float(sample.acceleration_m_s2.y, -2.0f * AERAKIA_GRAVITY_M_S2, 1.0e-5f)
               && close_float(sample.acceleration_m_s2.z, 0.5f * AERAKIA_GRAVITY_M_S2, 1.0e-6f),
               "FRD acceleration converts from g to m/s^2 without axis changes");
    check_true(close_float(sample.angular_rate_rad_s.x, AERAKIA_PI_F, 1.0e-6f)
               && close_float(sample.angular_rate_rad_s.y, -0.5f * AERAKIA_PI_F, 1.0e-6f)
               && close_float(sample.angular_rate_rad_s.z, 0.25f * AERAKIA_PI_F, 1.0e-6f),
               "FRD angular rate converts degrees/s to radians/s");
    check_true(close_float(sample.magnetic_field_ut.x, 20.0f, 1.0e-5f)
               && close_float(sample.magnetic_field_ut.y, -30.0f, 1.0e-5f)
               && close_float(sample.magnetic_field_ut.z, 45.0f, 1.0e-5f),
               "FRD magnetic field converts gauss to microtesla");
    check_true((sample.flags & AERAKIA_SAMPLE_ACCEL_VALID) != 0U
               && (sample.flags & AERAKIA_SAMPLE_GYRO_VALID) != 0U
               && (sample.flags & AERAKIA_SAMPLE_STATIONARY) != 0U
               && (sample.flags & AERAKIA_SAMPLE_MAG_VALID) == 0U,
               "validity flags map independently");
}

static void test_real_adapter_rejection_and_recovery(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    MockFcOneEstimatorPublication publication;
    AerakiaImuSample sample;
    AerakiaStatus status;

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.minimum_dt_s = 0.001f;
    config.maximum_dt_s = 0.020f;
    config.maximum_aiding_age_s = 0.050f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);

    publication = stationary_publication(10000U);
    sample = adapt_mock_publication(&publication);
    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_INITIALIZED, "first FCOne sample anchors time");

    publication.physical_timestamp_us = 20000U;
    publication.angular_rate_valid = false;
    sample = adapt_mock_publication(&publication);
    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_MISSING_MEASUREMENT,
               "missing required gyroscope is rejected");
    check_true(filter.last_timestamp_us == 10000U,
               "missing sample cannot advance estimator time");

    publication.angular_rate_valid = true;
    sample = adapt_mock_publication(&publication);
    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_OK && filter.last_timestamp_us == 20000U,
               "corrected sample at same source timestamp is accepted");

    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate publication is rejected");

    publication.physical_timestamp_us = 100000U;
    sample = adapt_mock_publication(&publication);
    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_TIMESTAMP_ERROR && filter.last_timestamp_us == 100000U,
               "forward transport gap reanchors without integration");

    publication.physical_timestamp_us = 110000U;
    sample = adapt_mock_publication(&publication);
    status = aerakia_eskf_process_imu(&filter, &sample, &estimate);
    check_true(status == AERAKIA_STATUS_OK, "normal processing resumes after gap");
    check_true(estimate.healthy, "rejected transport samples do not corrupt estimator health");
}

static void test_timestamped_aiding_contract(void)
{
    AerakiaEskf filter;
    AerakiaEskfConfig config;
    AerakiaNavigationEstimate estimate;
    MockFcOneEstimatorPublication publication;
    AerakiaImuSample sample;
    AerakiaGpsObservation gps;
    AerakiaHeadingObservation heading;
    MockFcOneNavigationPublication navigation;

    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.maximum_aiding_age_s = 0.050f;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    publication = stationary_publication(100000U);
    sample = adapt_mock_publication(&publication);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
    publication.physical_timestamp_us = 110000U;
    sample = adapt_mock_publication(&publication);
    (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);

    memset(&navigation, 0, sizeof(navigation));
    navigation.physical_timestamp_us = 120000U;
    navigation.position_ned_m = (AerakiaVec3f){12.0f, -34.0f, 5.0f};
    navigation.velocity_ned_m_s = (AerakiaVec3f){1.0f, -2.0f, 0.5f};
    navigation.position_variance_m2 = 1.0f;
    navigation.velocity_variance_m2_s2 = 0.1f;
    navigation.source_id = 7U;
    navigation.source_generation = 3U;
    navigation.quality_sequence = 101U;
    gps = adapt_mock_navigation(&navigation);
    check_true(gps.position_ned_m.x == 12.0f && gps.position_ned_m.y == -34.0f
               && gps.position_ned_m.z == 5.0f && gps.velocity_ned_m_s.y == -2.0f,
               "NED position and velocity reach the public contract without axis changes");
    check_true(gps.source_id == navigation.source_id
               && gps.source_generation == navigation.source_generation
               && gps.quality_sequence == navigation.quality_sequence,
               "navigation source identity and quality sequence reach the recovery contract");
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "future GNSS publication is rejected");
    gps.timestamp_us = 50000U;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_STALE_MEASUREMENT,
               "stale GNSS publication is rejected");
    gps.timestamp_us = 90000U;
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps) == AERAKIA_STATUS_OK,
               "fresh delayed GNSS uses physical observation time");
    check_true(aerakia_eskf_update_gps_observation(&filter, &gps)
                   == AERAKIA_STATUS_TIMESTAMP_ERROR,
               "duplicate GNSS publication is rejected");

    memset(&heading, 0, sizeof(heading));
    heading.timestamp_us = 105000U;
    heading.heading_ned_rad = 0.2f;
    heading.variance_rad2 = 0.01f;
    check_true(aerakia_eskf_update_heading_observation(&filter, &heading)
                   == AERAKIA_STATUS_OK,
               "fresh trusted heading uses its own source timestamp");
}

int main(void)
{
    test_unit_frame_flag_mapping();
    test_real_adapter_rejection_and_recovery();
    test_timestamped_aiding_contract();
    if (failures != 0) {
        fprintf(stderr, "%d FCOne-neutral adapter checks failed\n", failures);
        return 1;
    }
    printf("FCOne-neutral adapter contract passed\n");
    return 0;
}
