/**
 * @file estimator_supervisor_contract.c
 * @brief Executable application-level contract for ESKF/Mahony supervision.
 *
 * This is a host test oracle, not the FCOne flight-policy implementation. It fixes the required
 * mode, hysteresis, continuity, validity, and transition-log behavior before private integration.
 */

#include <aerakia/eskf_adapter.h>
#include <aerakia/types.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    SUPERVISOR_INITIALIZING = 0,
    SUPERVISOR_PRIMARY_ESKF,
    SUPERVISOR_DEGRADED_ATTITUDE_MAHONY,
    SUPERVISOR_ESTIMATE_INVALID
} SupervisorMode;

typedef enum {
    TRANSITION_NONE = 0,
    TRANSITION_ESKF_QUALIFIED,
    TRANSITION_ESKF_UNHEALTHY,
    TRANSITION_ESKF_UNOBSERVABLE,
    TRANSITION_ALL_ATTITUDE_INVALID,
    TRANSITION_ESKF_RECOVERED,
    TRANSITION_FALLBACK_TIMEOUT
} TransitionReason;

typedef struct {
    uint32_t eskf_failure_confirmation_samples;
    uint32_t eskf_recovery_confirmation_samples;
    float maximum_handover_angle_rad;
    uint64_t maximum_degraded_duration_us;
} SupervisorConfig;

typedef struct {
    uint64_t timestamp_us;
    AerakiaNavigationEstimate eskf;
    AerakiaAttitudeEstimate mahony;
    bool eskf_navigation_observable;
} SupervisorEvidence;

typedef struct {
    SupervisorMode mode;
    SupervisorConfig config;
    uint32_t eskf_failure_count;
    uint32_t eskf_recovery_count;
    uint32_t transition_count;
    uint64_t last_transition_timestamp_us;
    uint64_t degraded_started_timestamp_us;
    TransitionReason last_transition_reason;
    AerakiaAttitudeEstimate last_qualified_attitude;
    bool has_last_qualified_attitude;
} Supervisor;

typedef struct {
    SupervisorMode mode;
    AerakiaAttitudeEstimate attitude;
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
    bool attitude_valid;
    bool position_valid;
    bool velocity_valid;
    bool navigation_valid;
} QualifiedEstimate;

static int failures;

static void check_true(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures += 1;
    }
}

static float quaternion_separation_rad(const float left[4], const float right[4])
{
    float dot = fabsf(
        left[0] * right[0] + left[1] * right[1]
        + left[2] * right[2] + left[3] * right[3]
    );
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * acosf(dot);
}

static bool eskf_qualified(const SupervisorEvidence *evidence)
{
    return evidence->eskf.healthy && evidence->eskf.static_tilt_alignment_complete
        && evidence->eskf_navigation_observable;
}

static bool attitudes_continuous(
    const Supervisor *supervisor,
    const AerakiaAttitudeEstimate *candidate,
    const AerakiaAttitudeEstimate *reference
)
{
    return candidate->healthy && reference->healthy
        && quaternion_separation_rad(candidate->quaternion_wxyz, reference->quaternion_wxyz)
            <= supervisor->config.maximum_handover_angle_rad;
}

static bool fallback_continuous(
    const Supervisor *supervisor,
    const SupervisorEvidence *evidence
)
{
    return supervisor->has_last_qualified_attitude
        && attitudes_continuous(
            supervisor, &evidence->mahony, &supervisor->last_qualified_attitude
        );
}

static bool eskf_recovery_continuous(
    const Supervisor *supervisor,
    const SupervisorEvidence *evidence
)
{
    const AerakiaAttitudeEstimate *reference = supervisor->mode
        == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY
        ? &evidence->mahony : &supervisor->last_qualified_attitude;
    return supervisor->has_last_qualified_attitude
        && attitudes_continuous(supervisor, &evidence->eskf.attitude, reference);
}

static void transition(
    Supervisor *supervisor,
    SupervisorMode mode,
    TransitionReason reason,
    uint64_t timestamp_us
)
{
    if (supervisor->mode == mode) return;
    supervisor->mode = mode;
    supervisor->transition_count += 1U;
    supervisor->last_transition_timestamp_us = timestamp_us;
    supervisor->last_transition_reason = reason;
    supervisor->degraded_started_timestamp_us = mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY
        ? timestamp_us : 0U;
    supervisor->eskf_failure_count = 0U;
    supervisor->eskf_recovery_count = 0U;
}

static void supervisor_init(Supervisor *supervisor)
{
    memset(supervisor, 0, sizeof(*supervisor));
    supervisor->mode = SUPERVISOR_INITIALIZING;
    supervisor->config.eskf_failure_confirmation_samples = 3U;
    supervisor->config.eskf_recovery_confirmation_samples = 5U;
    supervisor->config.maximum_handover_angle_rad = 10.0f * AERAKIA_PI_F / 180.0f;
    supervisor->config.maximum_degraded_duration_us = 1000000U;
}

static QualifiedEstimate supervisor_update(
    Supervisor *supervisor,
    const SupervisorEvidence *evidence
)
{
    QualifiedEstimate output;
    const bool eskf_ok = eskf_qualified(evidence);
    const bool mahony_ok = evidence->mahony.healthy;
    const bool mahony_selectable = mahony_ok && fallback_continuous(supervisor, evidence);
    const bool recovery_continuous = eskf_ok
        && eskf_recovery_continuous(supervisor, evidence);
    memset(&output, 0, sizeof(output));

    if (supervisor->mode == SUPERVISOR_INITIALIZING) {
        supervisor->eskf_recovery_count = eskf_ok
            ? supervisor->eskf_recovery_count + 1U : 0U;
        if (supervisor->eskf_recovery_count
            >= supervisor->config.eskf_recovery_confirmation_samples) {
            transition(
                supervisor, SUPERVISOR_PRIMARY_ESKF, TRANSITION_ESKF_QUALIFIED,
                evidence->timestamp_us
            );
        }
    } else if (supervisor->mode == SUPERVISOR_PRIMARY_ESKF) {
        if (!evidence->eskf.healthy) {
            transition(
                supervisor,
                mahony_selectable ? SUPERVISOR_DEGRADED_ATTITUDE_MAHONY
                                  : SUPERVISOR_ESTIMATE_INVALID,
                mahony_selectable ? TRANSITION_ESKF_UNHEALTHY
                                  : TRANSITION_ALL_ATTITUDE_INVALID,
                evidence->timestamp_us
            );
        } else {
            supervisor->eskf_failure_count = eskf_ok
                ? 0U : supervisor->eskf_failure_count + 1U;
        }
        if (supervisor->mode == SUPERVISOR_PRIMARY_ESKF
            && supervisor->eskf_failure_count
            >= supervisor->config.eskf_failure_confirmation_samples) {
            transition(
                supervisor,
                mahony_selectable ? SUPERVISOR_DEGRADED_ATTITUDE_MAHONY
                                  : SUPERVISOR_ESTIMATE_INVALID,
                mahony_selectable ? TRANSITION_ESKF_UNOBSERVABLE
                                  : TRANSITION_ALL_ATTITUDE_INVALID,
                evidence->timestamp_us
            );
        }
    } else if (supervisor->mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY) {
        const bool fallback_timed_out = evidence->timestamp_us
                >= supervisor->degraded_started_timestamp_us
            && evidence->timestamp_us - supervisor->degraded_started_timestamp_us
                >= supervisor->config.maximum_degraded_duration_us;
        supervisor->eskf_recovery_count = recovery_continuous
            ? supervisor->eskf_recovery_count + 1U : 0U;
        if (!mahony_selectable) {
            transition(
                supervisor, SUPERVISOR_ESTIMATE_INVALID, TRANSITION_ALL_ATTITUDE_INVALID,
                evidence->timestamp_us
            );
        } else if (fallback_timed_out) {
            transition(
                supervisor, SUPERVISOR_ESTIMATE_INVALID, TRANSITION_FALLBACK_TIMEOUT,
                evidence->timestamp_us
            );
        } else if (supervisor->eskf_recovery_count
            >= supervisor->config.eskf_recovery_confirmation_samples) {
            transition(
                supervisor, SUPERVISOR_PRIMARY_ESKF, TRANSITION_ESKF_RECOVERED,
                evidence->timestamp_us
            );
        }
    } else {
        supervisor->eskf_recovery_count = recovery_continuous
            ? supervisor->eskf_recovery_count + 1U : 0U;
        if (supervisor->eskf_recovery_count
            >= supervisor->config.eskf_recovery_confirmation_samples) {
            transition(
                supervisor, SUPERVISOR_PRIMARY_ESKF, TRANSITION_ESKF_RECOVERED,
                evidence->timestamp_us
            );
        }
    }

    output.mode = supervisor->mode;
    if (supervisor->mode == SUPERVISOR_PRIMARY_ESKF) {
        output.attitude = evidence->eskf.attitude;
        output.position_ned_m = evidence->eskf.position_ned_m;
        output.velocity_ned_m_s = evidence->eskf.velocity_ned_m_s;
        output.attitude_valid = true;
        output.position_valid = true;
        output.velocity_valid = true;
        output.navigation_valid = true;
    } else if (supervisor->mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY) {
        output.attitude = evidence->mahony;
        output.attitude_valid = true;
    }
    if (output.attitude_valid) {
        supervisor->last_qualified_attitude = output.attitude;
        supervisor->has_last_qualified_attitude = true;
    }
    return output;
}

static SupervisorEvidence healthy_evidence(uint64_t timestamp_us)
{
    SupervisorEvidence evidence;
    memset(&evidence, 0, sizeof(evidence));
    evidence.timestamp_us = timestamp_us;
    evidence.eskf.healthy = true;
    evidence.eskf.attitude.healthy = true;
    evidence.eskf.static_tilt_alignment_complete = true;
    evidence.eskf_navigation_observable = true;
    evidence.eskf.attitude.quaternion_wxyz[0] = 1.0f;
    evidence.eskf.position_ned_m.x = 12.0f;
    evidence.eskf.velocity_ned_m_s.y = 3.0f;
    evidence.mahony.healthy = true;
    evidence.mahony.quaternion_wxyz[0] = 1.0f;
    return evidence;
}

static void test_primary_degraded_recovery_contract(void)
{
    Supervisor supervisor;
    SupervisorEvidence evidence;
    QualifiedEstimate output;
    uint32_t index;
    supervisor_init(&supervisor);

    evidence = healthy_evidence(0U);
    for (index = 0U; index < 4U; ++index) {
        evidence.timestamp_us += 10000U;
        output = supervisor_update(&supervisor, &evidence);
        check_true(!output.navigation_valid, "initialization never exposes navigation early");
    }
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_PRIMARY_ESKF, "ESKF enters primary after dwell");
    check_true(output.navigation_valid && output.position_valid && output.velocity_valid,
               "primary ESKF qualifies navigation fields");
    check_true(output.position_ned_m.x == 12.0f && output.velocity_ned_m_s.y == 3.0f,
               "primary output comes from ESKF");

    evidence.eskf_navigation_observable = false;
    for (index = 0U; index < 2U; ++index) {
        evidence.timestamp_us += 10000U;
        output = supervisor_update(&supervisor, &evidence);
        check_true(output.mode == SUPERVISOR_PRIMARY_ESKF,
                   "transient soft ESKF degradation does not switch immediately");
    }
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY,
               "confirmed ESKF observability loss selects robust Mahony");
    check_true(output.attitude_valid, "degraded mode retains attitude");
    check_true(!output.navigation_valid && !output.position_valid && !output.velocity_valid,
               "Mahony-only mode invalidates navigation fields");
    check_true(supervisor.last_transition_reason == TRANSITION_ESKF_UNOBSERVABLE,
               "degradation reason is logged");

    evidence.eskf_navigation_observable = true;
    evidence.eskf.attitude.quaternion_wxyz[0] = cosf(20.0f * AERAKIA_PI_F / 360.0f);
    evidence.eskf.attitude.quaternion_wxyz[3] = sinf(20.0f * AERAKIA_PI_F / 360.0f);
    for (index = 0U; index < 6U; ++index) {
        evidence.timestamp_us += 10000U;
        output = supervisor_update(&supervisor, &evidence);
    }
    check_true(output.mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY,
               "discontinuous ESKF attitude blocks automatic handback");

    evidence.eskf.attitude.quaternion_wxyz[0] = 1.0f;
    evidence.eskf.attitude.quaternion_wxyz[3] = 0.0f;
    for (index = 0U; index < 5U; ++index) {
        evidence.timestamp_us += 10000U;
        output = supervisor_update(&supervisor, &evidence);
    }
    check_true(output.mode == SUPERVISOR_PRIMARY_ESKF,
               "continuous healthy ESKF recovers after dwell");
    check_true(supervisor.last_transition_reason == TRANSITION_ESKF_RECOVERED,
               "recovery reason is logged");
    check_true(supervisor.transition_count == 3U, "all mode transitions are counted");
}

static void test_both_estimators_invalid_contract(void)
{
    Supervisor supervisor;
    SupervisorEvidence evidence = healthy_evidence(0U);
    QualifiedEstimate output;
    uint32_t index;
    supervisor_init(&supervisor);
    for (index = 0U; index < 5U; ++index) {
        evidence.timestamp_us += 10000U;
        (void)supervisor_update(&supervisor, &evidence);
    }
    evidence.eskf.healthy = false;
    evidence.eskf.attitude.healthy = false;
    evidence.mahony.healthy = false;
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_ESTIMATE_INVALID,
               "hard ESKF failure plus invalid Mahony immediately invalidates estimate");
    check_true(!output.attitude_valid && !output.navigation_valid,
               "invalid mode exposes no qualified control output");
    check_true(supervisor.last_transition_timestamp_us == evidence.timestamp_us,
               "invalid transition timestamp is retained");
}

static void test_discontinuous_fallback_is_rejected(void)
{
    Supervisor supervisor;
    SupervisorEvidence evidence = healthy_evidence(0U);
    QualifiedEstimate output;
    uint32_t index;
    supervisor_init(&supervisor);
    for (index = 0U; index < 5U; ++index) {
        evidence.timestamp_us += 10000U;
        (void)supervisor_update(&supervisor, &evidence);
    }
    evidence.mahony.quaternion_wxyz[0] = cosf(20.0f * AERAKIA_PI_F / 360.0f);
    evidence.mahony.quaternion_wxyz[3] = sinf(20.0f * AERAKIA_PI_F / 360.0f);
    evidence.eskf.healthy = false;
    evidence.eskf.attitude.healthy = false;
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_ESTIMATE_INVALID,
               "a discontinuous parallel Mahony state is never selected as fallback");
    check_true(!output.attitude_valid,
               "rejected fallback does not expose a discontinuous attitude");
}

static void test_fallback_has_a_finite_time_budget(void)
{
    Supervisor supervisor;
    SupervisorEvidence evidence = healthy_evidence(0U);
    QualifiedEstimate output;
    uint32_t index;
    supervisor_init(&supervisor);
    for (index = 0U; index < 5U; ++index) {
        evidence.timestamp_us += 10000U;
        (void)supervisor_update(&supervisor, &evidence);
    }
    evidence.eskf.healthy = false;
    evidence.eskf.attitude.healthy = false;
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY,
               "a continuous healthy Mahony state may enter bounded degraded mode");
    evidence.timestamp_us += supervisor.config.maximum_degraded_duration_us;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_ESTIMATE_INVALID,
               "Mahony-only degraded operation expires at its configured time budget");
    check_true(supervisor.last_transition_reason == TRANSITION_FALLBACK_TIMEOUT,
               "fallback timeout is retained as explicit transition evidence");
}

static void test_active_fallback_jump_is_immediately_invalid(void)
{
    Supervisor supervisor;
    SupervisorEvidence evidence = healthy_evidence(0U);
    QualifiedEstimate output;
    uint32_t index;
    supervisor_init(&supervisor);
    for (index = 0U; index < 5U; ++index) {
        evidence.timestamp_us += 10000U;
        (void)supervisor_update(&supervisor, &evidence);
    }
    evidence.eskf.healthy = false;
    evidence.eskf.attitude.healthy = false;
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_DEGRADED_ATTITUDE_MAHONY,
               "continuous fallback enters degraded mode before jump test");
    evidence.mahony.quaternion_wxyz[0] = cosf(20.0f * AERAKIA_PI_F / 360.0f);
    evidence.mahony.quaternion_wxyz[3] = sinf(20.0f * AERAKIA_PI_F / 360.0f);
    evidence.timestamp_us += 10000U;
    output = supervisor_update(&supervisor, &evidence);
    check_true(output.mode == SUPERVISOR_ESTIMATE_INVALID,
               "an active fallback continuity breach invalidates immediately");
    check_true(!output.attitude_valid,
               "a discontinuous active fallback is never published for one extra sample");
}

int main(void)
{
    test_primary_degraded_recovery_contract();
    test_both_estimators_invalid_contract();
    test_discontinuous_fallback_is_rejected();
    test_fallback_has_a_finite_time_budget();
    test_active_fallback_jump_is_immediately_invalid();
    if (failures != 0) {
        fprintf(stderr, "%d estimator-supervisor contract checks failed\n", failures);
        return 1;
    }
    printf("estimator-supervisor contract passed\n");
    return 0;
}
