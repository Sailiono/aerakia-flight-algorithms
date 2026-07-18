/**
 * @file input_integrity_campaign.c
 * @brief Deterministic million-sample public-API transport fault campaign.
 */

#include <aerakia/eskf_adapter.h>
#include <aerakia/mahony.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAMPAIGN_SEEDS 100U
#define SAMPLES_PER_SEED 10000U

typedef struct {
    uint64_t attempted;
    uint64_t accepted;
    uint64_t rejected_missing;
    uint64_t rejected_timestamp;
    uint64_t forward_gap_reanchors;
    uint64_t optional_mag_ignored;
    uint64_t burst_faults;
    uint64_t recoveries;
    uint64_t aiding_accepted;
    uint64_t aiding_duplicate_rejected;
    uint64_t aiding_reordered_rejected;
    uint64_t aiding_future_rejected;
    uint64_t aiding_stale_rejected;
    uint64_t aiding_numeric_rejected;
    uint64_t invariant_failures;
    uint64_t unhealthy_outputs;
} CampaignCounts;

static AerakiaImuSample level_sample(uint64_t timestamp_us);
static int eskf_core_unchanged(const AerakiaEskf *actual, const AerakiaEskf *before);
static int process_expected(
    AerakiaMahony *mahony,
    AerakiaEskf *eskf,
    const AerakiaImuSample *sample,
    AerakiaStatus expected,
    int expect_time_advance,
    CampaignCounts *counts
);

static uint32_t next_random(uint32_t *state)
{
    uint32_t value = *state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static void record_aiding_result(
    AerakiaStatus actual,
    AerakiaStatus expected,
    const AerakiaEskf *filter,
    const AerakiaEskf *before,
    CampaignCounts *counts,
    uint64_t *category
)
{
    (*category)++;
    if (actual != expected
        || (expected != AERAKIA_STATUS_OK && !eskf_core_unchanged(filter, before))) {
        counts->invariant_failures++;
    }
}

static void run_aiding_contract(
    AerakiaMahony *mahony,
    AerakiaEskf *eskf,
    uint64_t nominal_dt_us,
    CampaignCounts *counts
)
{
    AerakiaEskf before;
    AerakiaImuSample sample;
    const uint64_t fresh_timestamp = eskf->last_timestamp_us;
    AerakiaGpsObservation gps = {
        fresh_timestamp, {1.0f, -2.0f, 0.5f}, {0.1f, -0.2f, 0.0f}, 2.0f, 0.25f
    };
    AerakiaHeadingObservation heading = {fresh_timestamp, 0.1f, 0.02f};
    AerakiaBarometerObservation barometer = {fresh_timestamp, 0.5f, 1.5f};

    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );

    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_duplicate_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_duplicate_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_duplicate_rejected
    );

    gps.timestamp_us = fresh_timestamp - 1U;
    heading.timestamp_us = fresh_timestamp - 1U;
    barometer.timestamp_us = fresh_timestamp - 1U;
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_reordered_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_reordered_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer),
        AERAKIA_STATUS_TIMESTAMP_ERROR, eskf, &before, counts,
        &counts->aiding_reordered_rejected
    );

    gps.timestamp_us = fresh_timestamp + 1U;
    heading.timestamp_us = fresh_timestamp + 1U;
    barometer.timestamp_us = fresh_timestamp + 1U;
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_future_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading), AERAKIA_STATUS_TIMESTAMP_ERROR,
        eskf, &before, counts, &counts->aiding_future_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer),
        AERAKIA_STATUS_TIMESTAMP_ERROR, eskf, &before, counts,
        &counts->aiding_future_rejected
    );

    sample = level_sample(fresh_timestamp + 80000U);
    (void)process_expected(mahony, eskf, &sample, AERAKIA_STATUS_OK, 1, counts);
    gps.timestamp_us = fresh_timestamp + nominal_dt_us / 2U + 1U;
    heading.timestamp_us = gps.timestamp_us;
    barometer.timestamp_us = gps.timestamp_us;
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_STALE_MEASUREMENT,
        eskf, &before, counts, &counts->aiding_stale_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading),
        AERAKIA_STATUS_STALE_MEASUREMENT, eskf, &before, counts,
        &counts->aiding_stale_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer),
        AERAKIA_STATUS_STALE_MEASUREMENT, eskf, &before, counts,
        &counts->aiding_stale_rejected
    );

    gps.timestamp_us = eskf->last_timestamp_us;
    heading.timestamp_us = eskf->last_timestamp_us;
    barometer.timestamp_us = eskf->last_timestamp_us;
    gps.position_ned_m.x = NAN;
    heading.heading_ned_rad = INFINITY;
    barometer.variance_m2 = NAN;
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps),
        AERAKIA_STATUS_MISSING_MEASUREMENT, eskf, &before, counts,
        &counts->aiding_numeric_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading),
        AERAKIA_STATUS_MISSING_MEASUREMENT, eskf, &before, counts,
        &counts->aiding_numeric_rejected
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer),
        AERAKIA_STATUS_MISSING_MEASUREMENT, eskf, &before, counts,
        &counts->aiding_numeric_rejected
    );

    gps.position_ned_m.x = 1.0f;
    heading.heading_ned_rad = 0.1f;
    barometer.variance_m2 = 1.5f;
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_gps_observation(eskf, &gps), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_heading_observation(eskf, &heading), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );
    before = *eskf;
    record_aiding_result(
        aerakia_eskf_update_barometer_observation(eskf, &barometer), AERAKIA_STATUS_OK,
        eskf, &before, counts, &counts->aiding_accepted
    );
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

static int eskf_core_unchanged(const AerakiaEskf *actual, const AerakiaEskf *before)
{
    return memcmp(&actual->core.state, &before->core.state, sizeof(actual->core.state)) == 0
        && memcmp(actual->core.P, before->core.P, sizeof(actual->core.P)) == 0;
}

static int mahony_state_unchanged(const AerakiaMahony *actual, const AerakiaMahony *before)
{
    return memcmp(actual->quaternion_wxyz, before->quaternion_wxyz,
                  sizeof(actual->quaternion_wxyz)) == 0
        && memcmp(&actual->integral_feedback_rad_s, &before->integral_feedback_rad_s,
                  sizeof(actual->integral_feedback_rad_s)) == 0
        && actual->healthy == before->healthy;
}

static void record_health(
    const AerakiaAttitudeEstimate *mahony,
    const AerakiaNavigationEstimate *eskf,
    CampaignCounts *counts
)
{
    if (!mahony->healthy || !eskf->healthy) counts->unhealthy_outputs++;
}

static int process_expected(
    AerakiaMahony *mahony,
    AerakiaEskf *eskf,
    const AerakiaImuSample *sample,
    AerakiaStatus expected,
    int expect_time_advance,
    CampaignCounts *counts
)
{
    AerakiaMahony mahony_before = *mahony;
    AerakiaEskf eskf_before = *eskf;
    AerakiaAttitudeEstimate mahony_estimate;
    AerakiaNavigationEstimate eskf_estimate;
    const AerakiaStatus mahony_status = aerakia_mahony_update(
        mahony, sample, &mahony_estimate
    );
    const AerakiaStatus eskf_status = aerakia_eskf_process_imu(
        eskf, sample, &eskf_estimate
    );
    int valid = mahony_status == expected && eskf_status == expected;

    counts->attempted++;
    if (expected == AERAKIA_STATUS_OK) counts->accepted++;
    else if (expected == AERAKIA_STATUS_MISSING_MEASUREMENT) counts->rejected_missing++;
    else if (expected == AERAKIA_STATUS_TIMESTAMP_ERROR) counts->rejected_timestamp++;

    if (expected != AERAKIA_STATUS_OK) {
        valid = valid && mahony_state_unchanged(mahony, &mahony_before)
            && eskf_core_unchanged(eskf, &eskf_before);
        if (expect_time_advance) {
            valid = valid && mahony->last_timestamp_us == sample->timestamp_us
                && eskf->last_timestamp_us == sample->timestamp_us;
        } else {
            valid = valid && mahony->last_timestamp_us == mahony_before.last_timestamp_us
                && eskf->last_timestamp_us == eskf_before.last_timestamp_us;
        }
    } else {
        record_health(&mahony_estimate, &eskf_estimate, counts);
    }
    if (!valid) counts->invariant_failures++;
    return valid;
}

static void run_bursts(
    AerakiaMahony *mahony,
    AerakiaEskf *eskf,
    uint64_t nominal_dt_us,
    CampaignCounts *counts
)
{
    static const unsigned lengths[] = {1U, 2U, 5U, 10U, 20U, 50U, 100U};
    unsigned length_index;
    for (length_index = 0U; length_index < sizeof(lengths) / sizeof(lengths[0]); ++length_index) {
        const unsigned length = lengths[length_index];
        const uint64_t base_timestamp = eskf->last_timestamp_us;
        unsigned index;
        for (index = 0U; index < length; ++index) {
            AerakiaImuSample sample = level_sample(
                base_timestamp + (uint64_t)(index + 1U) * nominal_dt_us
            );
            sample.flags &= ~(uint32_t)AERAKIA_SAMPLE_GYRO_VALID;
            (void)process_expected(
                mahony, eskf, &sample, AERAKIA_STATUS_MISSING_MEASUREMENT, 0, counts
            );
            counts->burst_faults++;
        }
        {
            AerakiaImuSample recovery = level_sample(
                base_timestamp + (uint64_t)(length + 1U) * nominal_dt_us
            );
            const double recovery_dt_s =
                (double)(length + 1U) * (double)nominal_dt_us * 1.0e-6;
            if (recovery_dt_s > eskf->config.maximum_dt_s) {
                (void)process_expected(
                    mahony, eskf, &recovery, AERAKIA_STATUS_TIMESTAMP_ERROR, 1, counts
                );
                counts->forward_gap_reanchors++;
                recovery.timestamp_us += nominal_dt_us;
            }
            (void)process_expected(
                mahony, eskf, &recovery, AERAKIA_STATUS_OK, 1, counts
            );
            counts->recoveries++;
        }
    }
}

static void run_seed(unsigned seed, CampaignCounts *counts)
{
    static const unsigned rates_hz[] = {50U, 100U, 200U, 400U, 1000U};
    const unsigned rate_hz = rates_hz[seed % (sizeof(rates_hz) / sizeof(rates_hz[0]))];
    const uint64_t nominal_dt_us = 1000000U / rate_hz;
    uint32_t random_state = 0x9E3779B9U ^ (seed + 1U) * 0x85EBCA6BU;
    AerakiaMahony mahony;
    AerakiaEskf eskf;
    AerakiaEskfConfig eskf_config;
    AerakiaAttitudeEstimate mahony_estimate;
    AerakiaNavigationEstimate eskf_estimate;
    AerakiaImuSample sample = level_sample(1000U);
    unsigned index;

    aerakia_mahony_init(&mahony, NULL);
    aerakia_eskf_default_config(&eskf_config);
    eskf_config.enable_static_alignment = false;
    eskf_config.fuse_magnetometer = true;
    eskf_config.maximum_aiding_age_s = 0.05f;
    aerakia_eskf_init(&eskf, &eskf_config, NULL, NULL);
    (void)aerakia_mahony_update(&mahony, &sample, &mahony_estimate);
    (void)aerakia_eskf_process_imu(&eskf, &sample, &eskf_estimate);
    counts->attempted++;
    counts->accepted++;

    run_bursts(&mahony, &eskf, nominal_dt_us, counts);

    for (index = 0U; index < SAMPLES_PER_SEED; ++index) {
        const unsigned selector = next_random(&random_state) % 100U;
        const uint64_t last_timestamp = eskf.last_timestamp_us;
        sample = level_sample(last_timestamp + nominal_dt_us);

        if (selector < 75U) {
            (void)process_expected(&mahony, &eskf, &sample, AERAKIA_STATUS_OK, 1, counts);
        } else if (selector < 78U) {
            sample.flags &= ~(uint32_t)AERAKIA_SAMPLE_ACCEL_VALID;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_MISSING_MEASUREMENT, 0, counts
            );
        } else if (selector < 81U) {
            sample.flags &= ~(uint32_t)AERAKIA_SAMPLE_GYRO_VALID;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_MISSING_MEASUREMENT, 0, counts
            );
        } else if (selector < 84U) {
            sample.acceleration_m_s2.x = selector == 81U ? NAN
                : (selector == 82U ? INFINITY : -INFINITY);
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_MISSING_MEASUREMENT, 0, counts
            );
        } else if (selector < 87U) {
            sample.angular_rate_rad_s.z = selector == 84U ? NAN
                : (selector == 85U ? INFINITY : -INFINITY);
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_MISSING_MEASUREMENT, 0, counts
            );
        } else if (selector < 90U) {
            sample.timestamp_us = last_timestamp;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_TIMESTAMP_ERROR, 0, counts
            );
        } else if (selector < 92U) {
            sample.timestamp_us = last_timestamp > nominal_dt_us
                ? last_timestamp - nominal_dt_us : 0U;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_TIMESTAMP_ERROR, 0, counts
            );
        } else if (selector < 94U) {
            sample.timestamp_us = last_timestamp + 50U;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_TIMESTAMP_ERROR, 0, counts
            );
        } else if (selector < 96U) {
            sample.timestamp_us = last_timestamp + 200000U;
            (void)process_expected(
                &mahony, &eskf, &sample, AERAKIA_STATUS_TIMESTAMP_ERROR, 1, counts
            );
            counts->forward_gap_reanchors++;
        } else if (selector < 98U) {
            sample.magnetic_field_ut.y = selector == 96U ? NAN : INFINITY;
            (void)process_expected(&mahony, &eskf, &sample, AERAKIA_STATUS_OK, 1, counts);
            counts->optional_mag_ignored++;
        } else {
            const uint64_t maximum_safe_dt_us = 80000U;
            const uint64_t extra_intervals = maximum_safe_dt_us / nominal_dt_us;
            const uint64_t skipped = 1U + next_random(&random_state)
                % (extra_intervals > 1U ? extra_intervals - 1U : 1U);
            sample.timestamp_us = last_timestamp + skipped * nominal_dt_us;
            (void)process_expected(&mahony, &eskf, &sample, AERAKIA_STATUS_OK, 1, counts);
        }
    }
    run_aiding_contract(&mahony, &eskf, nominal_dt_us, counts);
}

int main(int argc, char *argv[])
{
    CampaignCounts counts;
    FILE *output = stdout;
    unsigned seed;
    memset(&counts, 0, sizeof(counts));
    if (argc > 2) {
        fputs("usage: aerakia_input_integrity_campaign [summary.json]\n", stderr);
        return 2;
    }
    if (argc == 2) {
        output = fopen(argv[1], "w");
        if (output == NULL) {
            perror("open summary");
            return 2;
        }
    }
    for (seed = 0U; seed < CAMPAIGN_SEEDS; ++seed) run_seed(seed, &counts);
    fprintf(
        output,
        "{\n"
        "  \"schema_version\": 1,\n"
        "  \"seeds\": %u,\n"
        "  \"samples_per_seed\": %u,\n"
        "  \"sample_rates_hz\": [50, 100, 200, 400, 1000],\n"
        "  \"attempted\": %llu,\n"
        "  \"accepted\": %llu,\n"
        "  \"rejected_missing\": %llu,\n"
        "  \"rejected_timestamp\": %llu,\n"
        "  \"forward_gap_reanchors\": %llu,\n"
        "  \"optional_mag_ignored\": %llu,\n"
        "  \"burst_faults\": %llu,\n"
        "  \"recoveries\": %llu,\n"
        "  \"aiding_accepted\": %llu,\n"
        "  \"aiding_duplicate_rejected\": %llu,\n"
        "  \"aiding_reordered_rejected\": %llu,\n"
        "  \"aiding_future_rejected\": %llu,\n"
        "  \"aiding_stale_rejected\": %llu,\n"
        "  \"aiding_numeric_rejected\": %llu,\n"
        "  \"invariant_failures\": %llu,\n"
        "  \"unhealthy_outputs\": %llu\n"
        "}\n",
        CAMPAIGN_SEEDS, SAMPLES_PER_SEED,
        (unsigned long long)counts.attempted,
        (unsigned long long)counts.accepted,
        (unsigned long long)counts.rejected_missing,
        (unsigned long long)counts.rejected_timestamp,
        (unsigned long long)counts.forward_gap_reanchors,
        (unsigned long long)counts.optional_mag_ignored,
        (unsigned long long)counts.burst_faults,
        (unsigned long long)counts.recoveries,
        (unsigned long long)counts.aiding_accepted,
        (unsigned long long)counts.aiding_duplicate_rejected,
        (unsigned long long)counts.aiding_reordered_rejected,
        (unsigned long long)counts.aiding_future_rejected,
        (unsigned long long)counts.aiding_stale_rejected,
        (unsigned long long)counts.aiding_numeric_rejected,
        (unsigned long long)counts.invariant_failures,
        (unsigned long long)counts.unhealthy_outputs
    );
    if (output != stdout) fclose(output);
    return counts.invariant_failures == 0U && counts.unhealthy_outputs == 0U ? 0 : 1;
}
