#include <aerakia/barometer_supervisor.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static AerakiaBarometerSupervisorObservation observation(
    double sample_s, double evaluation_s, float height_m, float predicted_m
)
{
    AerakiaBarometerSupervisorObservation value;
    memset(&value, 0, sizeof(value));
    value.sample_timestamp_us = (uint64_t)llround(sample_s * 1.0e6);
    value.evaluation_timestamp_us = (uint64_t)llround(evaluation_s * 1.0e6);
    value.height_up_m = height_m;
    value.variance_m2 = 0.16f;
    value.predicted_height_up_m = predicted_m;
    value.predicted_vertical_velocity_up_m_s = 0.0f;
    value.source_id = 7U;
    value.source_generation = 3U;
    value.quality_sequence = 9U;
    return value;
}

static AerakiaBarometerSupervisorDecision evaluate_and_commit(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *value,
    bool core_accepted
)
{
    AerakiaBarometerSupervisorDecision decision =
        aerakia_barometer_supervisor_evaluate(supervisor, value);
    if (decision.accepted) {
        aerakia_barometer_supervisor_commit(supervisor, core_accepted, &decision);
    }
    return decision;
}

static void test_stale_and_timestamp_rejection(void)
{
    AerakiaBarometerSupervisor supervisor;
    AerakiaBarometerSupervisorObservation value;
    AerakiaBarometerSupervisorDecision decision;
    aerakia_barometer_supervisor_init(&supervisor, NULL);
    value = observation(1.0, 1.0, 0.0f, 0.0f);
    check_true(evaluate_and_commit(&supervisor, &value, true).accepted,
               "first fresh observation is accepted");
    value = observation(1.1, 1.3, 0.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && (decision.fault_flags & AERAKIA_BARO_FAULT_STALE),
               "stale physical timestamp is rejected");
    value = observation(1.1, 1.1, 0.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && (decision.fault_flags & AERAKIA_BARO_FAULT_TIMESTAMP),
               "duplicate source timestamp is rejected even after a stale attempt");

    aerakia_barometer_supervisor_init(&supervisor, NULL);
    value = observation(0.0, 0.0, 0.0f, 0.0f);
    check_true(evaluate_and_commit(&supervisor, &value, true).accepted,
               "timestamp zero is a valid first sample");
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && (decision.fault_flags & AERAKIA_BARO_FAULT_TIMESTAMP),
               "duplicate timestamp zero is rejected");
}

static void test_matching_vertical_maneuvers_are_accepted(void)
{
    const float speeds[] = {0.5f, 2.0f, 5.0f, 10.0f, -0.5f, -2.0f, -5.0f, -10.0f};
    size_t speed_index;
    for (speed_index = 0U; speed_index < sizeof(speeds) / sizeof(speeds[0]); ++speed_index) {
        AerakiaBarometerSupervisor supervisor;
        AerakiaBarometerSupervisorDecision decision;
        int index;
        aerakia_barometer_supervisor_init(&supervisor, NULL);
        for (index = 0; index < 100; ++index) {
            const double time_s = 0.05 * index;
            const float height_m = speeds[speed_index] * (float)time_s;
            AerakiaBarometerSupervisorObservation value = observation(
                time_s, time_s, height_m, height_m
            );
            value.predicted_vertical_velocity_up_m_s = speeds[speed_index];
            decision = evaluate_and_commit(&supervisor, &value, true);
            check_true(decision.accepted, "matching climb/descent is not rejected as a source fault");
        }
    }
}

static void test_jump_latch_requires_authorized_recovery(void)
{
    AerakiaBarometerSupervisor supervisor;
    AerakiaBarometerSupervisorObservation value;
    AerakiaBarometerSupervisorDecision decision;
    AerakiaBarometerDatumRecoveryAuthorization authorization;
    int index;
    aerakia_barometer_supervisor_init(&supervisor, NULL);
    value = observation(1.0, 1.0, 0.0f, 0.0f);
    (void)evaluate_and_commit(&supervisor, &value, true);
    value = observation(1.05, 1.05, 3.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && !decision.fault_latched,
               "first large jump is rejected without an immediate latch");
    value = observation(1.10, 1.10, 3.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && decision.fault_latched,
               "second persistent jump latches the source fault");
    value = observation(1.11, 1.40, 0.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted && decision.fault_latched
                   && (decision.fault_flags & AERAKIA_BARO_FAULT_STALE),
               "stale samples do not hide an existing source latch");
    for (index = 1; index <= 11; ++index) {
        const double time_s = 1.10 + 0.05 * index;
        const float height_m = 0.02f * (float)index;
        value = observation(time_s, time_s, height_m, height_m);
        decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
        check_true(!decision.accepted && decision.fault_latched
                       && (decision.fault_flags
                           & AERAKIA_BARO_FAULT_RECOVERY_AUTH_REQUIRED),
                   "jump latch cannot recover without independent authorization");
    }

    memset(&authorization, 0, sizeof(authorization));
    authorization.authorization_timestamp_us = 1700000U;
    authorization.valid_until_timestamp_us = 3000000U;
    authorization.source_id = 7U;
    authorization.source_generation = 4U;
    authorization.quality_sequence = 9U;
    authorization.source_quality_verified = true;
    check_true(aerakia_barometer_supervisor_authorize_datum_recovery(
                   &supervisor, &authorization),
               "verified source-bound authorization is accepted");
    value = observation(1.70, 1.70, 0.0f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(!decision.accepted
                   && (decision.fault_flags & AERAKIA_BARO_FAULT_RECOVERY_AUTH_REQUIRED),
               "authorization for another source generation cannot recover the latch");

    authorization.source_generation = 3U;
    check_true(aerakia_barometer_supervisor_authorize_datum_recovery(
                   &supervisor, &authorization),
               "matching source generation can replace an unusable authorization");
    for (index = 1; index <= 11; ++index) {
        const double time_s = 1.70 + 0.05 * index;
        value = observation(time_s, time_s, 0.0f, 0.0f);
        decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
        if (decision.accepted) {
            aerakia_barometer_supervisor_commit(&supervisor, true, &decision);
        }
        if (index < 11) check_true(!decision.accepted, "authorized probation remains rejected");
    }
    check_true(decision.accepted && !decision.fault_latched,
               "authorized continuous baseline samples recover a jump latch");
}

static void test_core_rejection_preserves_fused_residual_baseline(void)
{
    AerakiaBarometerSupervisor supervisor;
    AerakiaBarometerSupervisorObservation value;
    AerakiaBarometerSupervisorDecision decision;
    aerakia_barometer_supervisor_init(&supervisor, NULL);
    value = observation(1.0, 1.0, 0.0f, 0.0f);
    (void)evaluate_and_commit(&supervisor, &value, true);

    value = observation(1.05, 1.05, 1.3f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(decision.accepted, "source guard can tentatively pass a core outlier");
    aerakia_barometer_supervisor_commit(&supervisor, false, &decision);
    check_true(!decision.accepted && fabsf(supervisor.last_accepted_height_up_m) < 1.0e-6f,
               "core rejection leaves the fused source reference unchanged");
    check_true(supervisor.last_seen_timestamp_us == 1050000U
                   && fabsf(supervisor.last_seen_height_up_m - 1.3f) < 1.0e-6f,
               "source classification history records a core-rejected observation");

    value = observation(1.10, 1.10, 1.3f, 0.0f);
    decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
    check_true(fabsf(decision.increment_residual_m - 1.3f) < 1.0e-5f,
               "next residual remains anchored to the last core-accepted sample");
    aerakia_barometer_supervisor_commit(&supervisor, false, &decision);
}

static void test_freeze_requires_predicted_motion(void)
{
    AerakiaBarometerSupervisor supervisor;
    AerakiaBarometerSupervisorObservation value;
    AerakiaBarometerSupervisorDecision decision;
    int index;
    aerakia_barometer_supervisor_init(&supervisor, NULL);
    for (index = 0; index <= 20; ++index) {
        const double time_s = 1.0 + 0.05 * index;
        value = observation(time_s, time_s, 0.0f, 0.0f);
        decision = evaluate_and_commit(&supervisor, &value, true);
        check_true(decision.accepted, "constant hover output is not called a freeze");
    }
    aerakia_barometer_supervisor_init(&supervisor, NULL);
    for (index = 0; index <= 20; ++index) {
        const double time_s = 1.0 + 0.05 * index;
        value = observation(time_s, time_s, 0.0f, 0.025f * (float)index);
        value.predicted_vertical_velocity_up_m_s = 0.5f;
        decision = aerakia_barometer_supervisor_evaluate(&supervisor, &value);
        if (decision.accepted) {
            aerakia_barometer_supervisor_commit(&supervisor, true, &decision);
        }
    }
    check_true(!decision.accepted && decision.fault_latched
                   && (decision.fault_flags & AERAKIA_BARO_FAULT_FREEZE),
               "unchanged height during predicted vertical motion latches freeze");
}

static void test_quantized_slow_vertical_motion_is_not_a_freeze(void)
{
    const float resolutions[] = {0.05f, 0.10f, 0.25f};
    const float speeds[] = {0.10f, 0.25f, 0.50f, -0.10f, -0.25f, -0.50f};
    size_t resolution_index;
    size_t speed_index;
    for (resolution_index = 0U;
         resolution_index < sizeof(resolutions) / sizeof(resolutions[0]);
         ++resolution_index) {
        for (speed_index = 0U; speed_index < sizeof(speeds) / sizeof(speeds[0]); ++speed_index) {
            AerakiaBarometerSupervisor supervisor;
            AerakiaBarometerSupervisorConfig config;
            AerakiaBarometerSupervisorDecision decision;
            int index;
            aerakia_barometer_supervisor_default_config(&config);
            config.measurement_quantization_m = resolutions[resolution_index];
            aerakia_barometer_supervisor_init(&supervisor, &config);
            for (index = 0; index < 400; ++index) {
                const double time_s = 0.05 * index;
                const float predicted_m = speeds[speed_index] * (float)time_s;
                const float height_m = roundf(predicted_m / resolutions[resolution_index])
                    * resolutions[resolution_index];
                AerakiaBarometerSupervisorObservation value = observation(
                    time_s, time_s, height_m, predicted_m
                );
                value.predicted_vertical_velocity_up_m_s = speeds[speed_index];
                decision = evaluate_and_commit(&supervisor, &value, true);
                check_true(decision.accepted && !decision.fault_latched,
                           "declared quantization does not false-latch slow vertical motion");
            }
        }
    }
}

int main(void)
{
    test_stale_and_timestamp_rejection();
    test_matching_vertical_maneuvers_are_accepted();
    test_jump_latch_requires_authorized_recovery();
    test_core_rejection_preserves_fused_residual_baseline();
    test_freeze_requires_predicted_motion();
    test_quantized_slow_vertical_motion_is_not_a_freeze();
    if (failures != 0) return EXIT_FAILURE;
    puts("barometer supervisor tests passed");
    return EXIT_SUCCESS;
}
