/**
 * @file barometer_multilane_handoff_contract.c
 * @brief Host-only complete-state shadow-lane handoff contract.
 *
 * This is not FCOne flight policy.  It establishes the transaction that a
 * private supervisor must preserve when a barometer-free ESKF shadow lane
 * replaces a latched barometer lane: no component splice, no covariance blend,
 * and no automatic return.
 */

#include <aerakia/barometer_supervisor.h>
#include <aerakia/eskf_adapter.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HANDOFF_CONFIG_ID UINT64_C(0x4c414e455f563031)
#define BAROMETER_SOURCE_ID 17U
#define BAROMETER_SOURCE_GENERATION 4U
#define BAROMETER_QUALITY_SEQUENCE UINT64_C(91)
#define DEFAULT_IMU_DT_US UINT64_C(10000)
#define GRAVITY_M_S2 9.80665f

typedef struct {
    uint64_t common_event_hash;
    uint64_t configuration_id;
    uint64_t source_quality_sequence;
    uint32_t imu_events;
    uint32_t position_events;
    uint32_t velocity_events;
    uint32_t heading_events;
    uint32_t source_id;
    uint32_t source_generation;
} LaneProvenance;

typedef struct {
    AerakiaEskf filter;
    LaneProvenance provenance;
    bool barometer_enabled;
} EstimatorLane;

typedef enum {
    HANDOFF_OK = 0,
    HANDOFF_BLOCK_NOT_LATCHED,
    HANDOFF_BLOCK_ALREADY_SWITCHED,
    HANDOFF_BLOCK_RETURN_AUTHORIZATION_REQUIRED,
    HANDOFF_BLOCK_PROVENANCE,
    HANDOFF_BLOCK_CONFIGURATION,
    HANDOFF_BLOCK_SOURCE_IDENTITY,
    HANDOFF_BLOCK_TIMESTAMP,
    HANDOFF_BLOCK_SHADOW_UNHEALTHY,
    HANDOFF_BLOCK_ALIGNMENT,
    HANDOFF_BLOCK_NONFINITE
} HandoffResult;

typedef struct {
    bool fault_latched;
    bool return_to_barometer_lane;
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
} HandoffRequest;

typedef struct {
    bool switched;
    uint32_t switch_count;
    HandoffResult last_result;
    float attitude_reset_rad;
    float position_reset_norm_m;
    float velocity_reset_norm_m_s;
    float accelerometer_bias_reset_norm_m_s2;
    float gyroscope_bias_reset_norm_rad_s;
    double covariance_reset_max_abs;
} HandoffSession;

static int failures;
static HandoffSession successful_session;
static LaneProvenance successful_provenance;
static unsigned successful_rate_cases;
static uint64_t successful_last_interval_us;

static void check_true(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures += 1;
    }
}

static uint32_t float_bits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void hash_u64(uint64_t *hash, uint64_t value)
{
    unsigned byte;
    for (byte = 0U; byte < 8U; ++byte) {
        *hash ^= (value >> (byte * 8U)) & UINT64_C(0xff);
        *hash *= UINT64_C(1099511628211);
    }
}

static void hash_vec3(uint64_t *hash, AerakiaVec3f value)
{
    hash_u64(hash, float_bits(value.x));
    hash_u64(hash, float_bits(value.y));
    hash_u64(hash, float_bits(value.z));
}

static float vec3_norm(AerakiaVec3f value)
{
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

static float quaternion_difference_rad(const eskf_float_t left[4], const eskf_float_t right[4])
{
    double dot = fabs(
        (double)left[0] * right[0] + (double)left[1] * right[1]
        + (double)left[2] * right[2] + (double)left[3] * right[3]
    );
    if (dot > 1.0) dot = 1.0;
    return (float)(2.0 * acos(dot));
}

static unsigned steps_for_duration(uint64_t duration_us, uint64_t interval_us)
{
    return (unsigned)((duration_us + interval_us - 1U) / interval_us);
}

static void initialize_lane(EstimatorLane *lane, bool barometer_enabled)
{
    AerakiaEskfConfig config;
    const double initial_position[3] = {0.0, 0.0, 0.0};

    memset(lane, 0, sizeof(*lane));
    aerakia_eskf_default_config(&config);
    config.static_alignment_duration_s = 0.03f;
    config.static_alignment_min_samples = 4U;
    config.zero_velocity_interval_s = 0.0f;
    config.maximum_horizontal_position_dead_reckoning_s = -1.0f;
    config.maximum_horizontal_velocity_dead_reckoning_s = -1.0f;
    config.maximum_heading_dead_reckoning_s = -1.0f;
    config.maximum_vertical_position_dead_reckoning_s = -1.0f;
    config.maximum_vertical_velocity_dead_reckoning_s = -1.0f;
    aerakia_eskf_init(&lane->filter, &config, initial_position, NULL);
    lane->barometer_enabled = barometer_enabled;
    lane->provenance.common_event_hash = UINT64_C(1469598103934665603);
    lane->provenance.configuration_id = HANDOFF_CONFIG_ID;
    lane->provenance.source_quality_sequence = BAROMETER_QUALITY_SEQUENCE;
    lane->provenance.source_id = BAROMETER_SOURCE_ID;
    lane->provenance.source_generation = BAROMETER_SOURCE_GENERATION;
}

static void audit_imu(LaneProvenance *provenance, const AerakiaImuSample *sample)
{
    hash_u64(&provenance->common_event_hash, UINT64_C(1));
    hash_u64(&provenance->common_event_hash, sample->timestamp_us);
    hash_vec3(&provenance->common_event_hash, sample->acceleration_m_s2);
    hash_vec3(&provenance->common_event_hash, sample->angular_rate_rad_s);
    hash_vec3(&provenance->common_event_hash, sample->magnetic_field_ut);
    hash_u64(&provenance->common_event_hash, sample->flags);
    provenance->imu_events++;
}

static void audit_position(
    LaneProvenance *provenance,
    const AerakiaPositionObservation *observation
)
{
    hash_u64(&provenance->common_event_hash, UINT64_C(2));
    hash_u64(&provenance->common_event_hash, observation->timestamp_us);
    hash_vec3(&provenance->common_event_hash, observation->position_ned_m);
    hash_u64(&provenance->common_event_hash, float_bits(observation->variance_m2));
    provenance->position_events++;
}

static void audit_velocity(
    LaneProvenance *provenance,
    const AerakiaVelocityObservation *observation
)
{
    hash_u64(&provenance->common_event_hash, UINT64_C(3));
    hash_u64(&provenance->common_event_hash, observation->timestamp_us);
    hash_vec3(&provenance->common_event_hash, observation->velocity_ned_m_s);
    hash_u64(&provenance->common_event_hash, float_bits(observation->variance_m2_s2));
    provenance->velocity_events++;
}

static void audit_heading(
    LaneProvenance *provenance,
    const AerakiaHeadingObservation *observation
)
{
    hash_u64(&provenance->common_event_hash, UINT64_C(4));
    hash_u64(&provenance->common_event_hash, observation->timestamp_us);
    hash_u64(&provenance->common_event_hash, float_bits(observation->heading_ned_rad));
    hash_u64(&provenance->common_event_hash, float_bits(observation->variance_rad2));
    provenance->heading_events++;
}

static AerakiaImuSample make_imu(uint64_t timestamp_us, float down_acceleration_m_s2, bool stationary)
{
    AerakiaImuSample sample;
    memset(&sample, 0, sizeof(sample));
    sample.timestamp_us = timestamp_us;
    sample.acceleration_m_s2.z = -GRAVITY_M_S2 + down_acceleration_m_s2;
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID;
    if (stationary) sample.flags |= AERAKIA_SAMPLE_STATIONARY;
    return sample;
}

static void publish_common_imu(
    EstimatorLane *active,
    EstimatorLane *shadow,
    const AerakiaImuSample *sample
)
{
    AerakiaNavigationEstimate active_estimate;
    AerakiaNavigationEstimate shadow_estimate;
    const AerakiaStatus active_status = aerakia_eskf_process_imu(
        &active->filter, sample, &active_estimate
    );
    const AerakiaStatus shadow_status = aerakia_eskf_process_imu(
        &shadow->filter, sample, &shadow_estimate
    );
    check_true(active_status == shadow_status, "common IMU status is identical in both lanes");
    check_true(
        active_status == AERAKIA_STATUS_OK || active_status == AERAKIA_STATUS_ALIGNING
            || active_status == AERAKIA_STATUS_INITIALIZED,
        "common IMU event remains accepted by both lanes"
    );
    audit_imu(&active->provenance, sample);
    audit_imu(&shadow->provenance, sample);
}

static void publish_common_aiding(
    EstimatorLane *active,
    EstimatorLane *shadow,
    uint64_t timestamp_us,
    float elapsed_s
)
{
    const float down_acceleration = 4.0f;
    AerakiaPositionObservation position;
    AerakiaVelocityObservation velocity;
    AerakiaHeadingObservation heading;
    AerakiaStatus active_status;
    AerakiaStatus shadow_status;

    memset(&position, 0, sizeof(position));
    memset(&velocity, 0, sizeof(velocity));
    memset(&heading, 0, sizeof(heading));
    position.timestamp_us = timestamp_us;
    position.position_ned_m.z = 0.5f * down_acceleration * elapsed_s * elapsed_s;
    position.variance_m2 = 0.25f;
    velocity.timestamp_us = timestamp_us;
    velocity.velocity_ned_m_s.z = down_acceleration * elapsed_s;
    velocity.variance_m2_s2 = 0.04f;
    heading.timestamp_us = timestamp_us;
    heading.heading_ned_rad = 0.0f;
    heading.variance_rad2 = 0.25f;

    active_status = aerakia_eskf_update_position_observation(&active->filter, &position);
    shadow_status = aerakia_eskf_update_position_observation(&shadow->filter, &position);
    check_true(active_status == shadow_status, "common position status is identical in both lanes");
    check_true(active_status == AERAKIA_STATUS_OK, "common position observation is transport-valid");
    active_status = aerakia_eskf_update_velocity_observation(&active->filter, &velocity);
    shadow_status = aerakia_eskf_update_velocity_observation(&shadow->filter, &velocity);
    check_true(active_status == shadow_status, "common velocity status is identical in both lanes");
    check_true(active_status == AERAKIA_STATUS_OK, "common velocity observation is transport-valid");
    active_status = aerakia_eskf_update_heading_observation(&active->filter, &heading);
    shadow_status = aerakia_eskf_update_heading_observation(&shadow->filter, &heading);
    check_true(active_status == shadow_status, "common heading status is identical in both lanes");
    check_true(active_status == AERAKIA_STATUS_OK, "common heading observation is transport-valid");
    audit_position(&active->provenance, &position);
    audit_position(&shadow->provenance, &position);
    audit_velocity(&active->provenance, &velocity);
    audit_velocity(&shadow->provenance, &velocity);
    audit_heading(&active->provenance, &heading);
    audit_heading(&shadow->provenance, &heading);
}

static bool finite_filter_image(const AerakiaEskf *filter)
{
    const eskf_float_t *state = (const eskf_float_t *)&filter->core.state;
    const eskf_float_t *error = (const eskf_float_t *)&filter->core.error_state;
    unsigned index;
    unsigned row;
    unsigned column;

    for (index = 0U; index < ESKF_NOMINAL_STATE_DIM; ++index) {
        if (!isfinite((double)state[index])) return false;
    }
    for (index = 0U; index < ESKF_ERROR_STATE_DIM; ++index) {
        if (!isfinite((double)error[index])) return false;
    }
    for (row = 0U; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0U; column < ESKF_ERROR_STATE_DIM; ++column) {
            if (!isfinite((double)filter->core.P[row][column])) return false;
        }
    }
    return true;
}

static bool lane_is_healthy(const EstimatorLane *lane)
{
    AerakiaNavigationEstimate estimate;
    aerakia_eskf_get_estimate(&lane->filter, &estimate);
    return estimate.healthy;
}

static bool provenance_common_inputs_match(
    const LaneProvenance *active,
    const LaneProvenance *shadow
)
{
    return active->common_event_hash == shadow->common_event_hash
        && active->imu_events == shadow->imu_events
        && active->position_events == shadow->position_events
        && active->velocity_events == shadow->velocity_events
        && active->heading_events == shadow->heading_events;
}

static bool provenance_configuration_matches(
    const LaneProvenance *active,
    const LaneProvenance *shadow
)
{
    return active->configuration_id == shadow->configuration_id;
}

static bool provenance_source_matches(
    const LaneProvenance *active,
    const LaneProvenance *shadow,
    const HandoffRequest *request
)
{
    return active->source_id == shadow->source_id
        && active->source_generation == shadow->source_generation
        && active->source_quality_sequence == shadow->source_quality_sequence
        && active->source_id == request->source_id
        && active->source_generation == request->source_generation
        && active->source_quality_sequence == request->quality_sequence;
}

static void record_reset_delta(
    HandoffSession *session,
    const EstimatorLane *active,
    const EstimatorLane *shadow
)
{
    AerakiaVec3f position;
    AerakiaVec3f velocity;
    AerakiaVec3f accelerometer_bias;
    AerakiaVec3f gyroscope_bias;
    unsigned row;
    unsigned column;

    position.x = (float)(shadow->filter.core.state.p[0] - active->filter.core.state.p[0]);
    position.y = (float)(shadow->filter.core.state.p[1] - active->filter.core.state.p[1]);
    position.z = (float)(shadow->filter.core.state.p[2] - active->filter.core.state.p[2]);
    velocity.x = (float)(shadow->filter.core.state.v[0] - active->filter.core.state.v[0]);
    velocity.y = (float)(shadow->filter.core.state.v[1] - active->filter.core.state.v[1]);
    velocity.z = (float)(shadow->filter.core.state.v[2] - active->filter.core.state.v[2]);
    accelerometer_bias.x = (float)(
        shadow->filter.core.state.ab[0] - active->filter.core.state.ab[0]
    );
    accelerometer_bias.y = (float)(
        shadow->filter.core.state.ab[1] - active->filter.core.state.ab[1]
    );
    accelerometer_bias.z = (float)(
        shadow->filter.core.state.ab[2] - active->filter.core.state.ab[2]
    );
    gyroscope_bias.x = (float)(shadow->filter.core.state.gb[0] - active->filter.core.state.gb[0]);
    gyroscope_bias.y = (float)(shadow->filter.core.state.gb[1] - active->filter.core.state.gb[1]);
    gyroscope_bias.z = (float)(shadow->filter.core.state.gb[2] - active->filter.core.state.gb[2]);
    session->attitude_reset_rad = quaternion_difference_rad(
        active->filter.core.state.q, shadow->filter.core.state.q
    );
    session->position_reset_norm_m = vec3_norm(position);
    session->velocity_reset_norm_m_s = vec3_norm(velocity);
    session->accelerometer_bias_reset_norm_m_s2 = vec3_norm(accelerometer_bias);
    session->gyroscope_bias_reset_norm_rad_s = vec3_norm(gyroscope_bias);
    session->covariance_reset_max_abs = 0.0;
    for (row = 0U; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0U; column < ESKF_ERROR_STATE_DIM; ++column) {
            const double delta = fabs((double)(
                shadow->filter.core.P[row][column] - active->filter.core.P[row][column]
            ));
            if (delta > session->covariance_reset_max_abs) {
                session->covariance_reset_max_abs = delta;
            }
        }
    }
}

static HandoffResult try_complete_handoff(
    HandoffSession *session,
    EstimatorLane *output,
    const EstimatorLane *active,
    const EstimatorLane *shadow,
    const HandoffRequest *request
)
{
    HandoffResult result;

    if (session->switched) {
        result = HANDOFF_BLOCK_ALREADY_SWITCHED;
    } else if (request->return_to_barometer_lane) {
        result = HANDOFF_BLOCK_RETURN_AUTHORIZATION_REQUIRED;
    } else if (!request->fault_latched) {
        result = HANDOFF_BLOCK_NOT_LATCHED;
    } else if (!provenance_common_inputs_match(&active->provenance, &shadow->provenance)) {
        result = HANDOFF_BLOCK_PROVENANCE;
    } else if (!provenance_configuration_matches(&active->provenance, &shadow->provenance)) {
        result = HANDOFF_BLOCK_CONFIGURATION;
    } else if (!provenance_source_matches(
        &active->provenance, &shadow->provenance, request
    )) {
        result = HANDOFF_BLOCK_SOURCE_IDENTITY;
    } else if (active->filter.last_timestamp_us != shadow->filter.last_timestamp_us) {
        result = HANDOFF_BLOCK_TIMESTAMP;
    } else if (!finite_filter_image(&shadow->filter)) {
        result = HANDOFF_BLOCK_NONFINITE;
    } else if (!lane_is_healthy(shadow)) {
        result = HANDOFF_BLOCK_SHADOW_UNHEALTHY;
    } else if (!active->filter.static_alignment_complete
        || !shadow->filter.static_alignment_complete) {
        result = HANDOFF_BLOCK_ALIGNMENT;
    } else {
        record_reset_delta(session, active, shadow);
        /* AerakiaEskf currently has no pointers or external ownership.  This
         * bytewise transfer intentionally proves an all-or-nothing host image,
         * not an embedded concurrency primitive. */
        memcpy(output, shadow, sizeof(*output));
        session->switched = true;
        session->switch_count++;
        result = HANDOFF_OK;
    }
    session->last_result = result;
    return result;
}

static void update_active_barometer(
    EstimatorLane *active,
    AerakiaBarometerSupervisor *supervisor,
    uint64_t timestamp_us,
    float height_up_m,
    AerakiaBarometerSupervisorDecision *last_decision
)
{
    AerakiaBarometerSupervisorObservation supervisor_observation;
    AerakiaBarometerSupervisorDecision decision;
    AerakiaBarometerObservation observation;
    AerakiaStatus status;

    memset(&supervisor_observation, 0, sizeof(supervisor_observation));
    memset(&observation, 0, sizeof(observation));
    supervisor_observation.sample_timestamp_us = timestamp_us;
    supervisor_observation.evaluation_timestamp_us = timestamp_us;
    supervisor_observation.height_up_m = height_up_m;
    supervisor_observation.variance_m2 = 0.25f;
    supervisor_observation.predicted_height_up_m = (float)(-active->filter.core.state.p[2]);
    supervisor_observation.predicted_vertical_velocity_up_m_s =
        (float)(-active->filter.core.state.v[2]);
    supervisor_observation.source_id = BAROMETER_SOURCE_ID;
    supervisor_observation.source_generation = BAROMETER_SOURCE_GENERATION;
    supervisor_observation.quality_sequence = BAROMETER_QUALITY_SEQUENCE;
    decision = aerakia_barometer_supervisor_evaluate(supervisor, &supervisor_observation);
    if (decision.accepted) {
        observation.timestamp_us = timestamp_us;
        observation.height_up_m = height_up_m;
        observation.variance_m2 = 0.25f;
        status = aerakia_eskf_update_barometer_observation(&active->filter, &observation);
        check_true(status == AERAKIA_STATUS_OK, "accepted barometer observation is transport-valid");
        aerakia_barometer_supervisor_commit(
            supervisor, active->filter.barometer_accepted, &decision
        );
    } else {
        aerakia_eskf_note_barometer_rejection(&active->filter);
    }
    *last_decision = decision;
}

static bool prepare_latched_lanes(
    EstimatorLane *active,
    EstimatorLane *shadow,
    HandoffRequest *request,
    uint64_t imu_interval_us
)
{
    AerakiaBarometerSupervisorConfig supervisor_config;
    AerakiaBarometerSupervisor supervisor;
    uint64_t timestamp_us = 0U;
    float frozen_height_up_m = 0.0f;
    bool freeze_started = false;
    const unsigned static_steps = steps_for_duration(40000U, imu_interval_us);
    const unsigned motion_steps = steps_for_duration(800000U, imu_interval_us);
    const unsigned barometer_period_steps = steps_for_duration(50000U, imu_interval_us);
    const unsigned freeze_start_step = 4U * barometer_period_steps;
    unsigned index;

    initialize_lane(active, true);
    initialize_lane(shadow, false);
    aerakia_barometer_supervisor_default_config(&supervisor_config);
    supervisor_config.jump_minimum_threshold_m = 100.0f;
    supervisor_config.jump_sigma_multiplier = 100.0f;
    supervisor_config.freeze_consecutive_samples = 3U;
    supervisor_config.freeze_repeat_tolerance_m = 0.001f;
    supervisor_config.freeze_minimum_vertical_speed_m_s = 0.10f;
    aerakia_barometer_supervisor_init(&supervisor, &supervisor_config);

    for (index = 0U; index < static_steps; ++index) {
        AerakiaImuSample sample;
        timestamp_us += imu_interval_us;
        sample = make_imu(timestamp_us, 0.0f, true);
        publish_common_imu(active, shadow, &sample);
    }
    check_true(active->filter.static_alignment_complete, "active lane completes static alignment");
    check_true(shadow->filter.static_alignment_complete, "shadow lane completes static alignment");

    for (index = 0U; index < motion_steps; ++index) {
        AerakiaImuSample sample;
        const float elapsed_s = (float)((double)(index + 1U) * (double)imu_interval_us * 1.0e-6);
        AerakiaBarometerSupervisorDecision decision;
        timestamp_us += imu_interval_us;
        sample = make_imu(timestamp_us, 4.0f, false);
        publish_common_imu(active, shadow, &sample);
        if (index == 0U || index == barometer_period_steps) {
            publish_common_aiding(active, shadow, timestamp_us, elapsed_s);
        }
        if ((index % barometer_period_steps) != 0U) continue;
        if (index >= freeze_start_step && !freeze_started) {
            frozen_height_up_m = (float)(-active->filter.core.state.p[2] + 0.25);
            freeze_started = true;
        }
        update_active_barometer(
            active,
            &supervisor,
            timestamp_us,
            freeze_started ? frozen_height_up_m : (float)(-active->filter.core.state.p[2]),
            &decision
        );
        if (decision.fault_latched) {
            memset(request, 0, sizeof(*request));
            request->fault_latched = true;
            request->source_id = BAROMETER_SOURCE_ID;
            request->source_generation = BAROMETER_SOURCE_GENERATION;
            request->quality_sequence = BAROMETER_QUALITY_SEQUENCE;
            return true;
        }
    }
    return false;
}

static void check_complete_shadow_image(
    const EstimatorLane *output,
    const EstimatorLane *shadow,
    const char *message
)
{
    check_true(
        memcmp(output, shadow, sizeof(*output)) == 0,
        message
    );
}

static void check_output_unchanged(
    const EstimatorLane *before,
    const EstimatorLane *after,
    const char *message
)
{
    check_true(memcmp(before, after, sizeof(*before)) == 0, message);
}

static void test_successful_full_state_handoff(void)
{
    static const uint64_t intervals_us[] = {10000U, 5000U, 2500U};
    unsigned rate_index;

    for (rate_index = 0U; rate_index < sizeof(intervals_us) / sizeof(intervals_us[0]); ++rate_index) {
        EstimatorLane active;
        EstimatorLane shadow;
        EstimatorLane output;
        EstimatorLane active_before;
        HandoffRequest request;
        HandoffSession session;
        AerakiaImuSample next_sample;
        const uint64_t imu_interval_us = intervals_us[rate_index];

        memset(&session, 0, sizeof(session));
        check_true(prepare_latched_lanes(&active, &shadow, &request, imu_interval_us),
            "supervised frozen barometer latches in the deterministic stream");
        check_true(active.provenance.common_event_hash == shadow.provenance.common_event_hash,
            "active and shadow consume an identical common event stream");
        check_true(active.provenance.imu_events > 0U && active.provenance.position_events > 0U
            && active.provenance.velocity_events > 0U && active.provenance.heading_events > 0U,
            "common stream includes IMU, position, velocity, and heading events");
        check_true(active.barometer_enabled && !shadow.barometer_enabled,
            "only the active lane receives barometer aiding");
        check_true(fabs((double)(
            active.filter.core.P[ESKF_IDX_DP + 2][ESKF_IDX_DP + 2]
            - shadow.filter.core.P[ESKF_IDX_DP + 2][ESKF_IDX_DP + 2]
        )) > 1.0e-12, "barometer lane differs from its shadow before the transaction");

        memcpy(&output, &active, sizeof(output));
        memcpy(&active_before, &active, sizeof(active_before));
        check_true(
            try_complete_handoff(&session, &output, &active, &shadow, &request) == HANDOFF_OK,
            "qualified latch performs one complete shadow-lane transaction"
        );
        check_true(session.switched && session.switch_count == 1U,
            "successful handoff records exactly one switch");
        check_true(session.covariance_reset_max_abs > 1.0e-12,
            "handoff records a nonzero covariance reset delta");
        check_true(session.position_reset_norm_m > 1.0e-5f,
            "latched barometer lane has a nonzero position reset to the shadow lane");
        check_true(isfinite(session.attitude_reset_rad)
            && isfinite(session.position_reset_norm_m)
            && isfinite(session.velocity_reset_norm_m_s)
            && isfinite(session.accelerometer_bias_reset_norm_m_s2)
            && isfinite(session.gyroscope_bias_reset_norm_rad_s),
            "handoff records finite reset deltas for every nominal-state group");
        check_complete_shadow_image(
            &output, &shadow,
            "handoff output is the complete shadow lane, not a vertical-component splice"
        );
        check_complete_shadow_image(
            &active, &active_before,
            "handoff leaves the latched active lane unchanged for diagnostics"
        );

        next_sample = make_imu(output.filter.last_timestamp_us + imu_interval_us, 4.0f, false);
        publish_common_imu(&output, &shadow, &next_sample);
        check_complete_shadow_image(
            &output, &shadow,
            "post-handoff common IMU propagation remains equivalent to the shadow lane"
        );
        check_true(
            try_complete_handoff(&session, &output, &active, &shadow, &request)
                == HANDOFF_BLOCK_ALREADY_SWITCHED,
            "a completed handoff cannot silently switch a second time"
        );
        check_complete_shadow_image(
            &output, &shadow,
            "duplicate switch attempt leaves the selected lane unchanged"
        );
        successful_session = session;
        successful_provenance = output.provenance;
        successful_rate_cases++;
        successful_last_interval_us = imu_interval_us;
    }
}

static void expect_blocked(
    HandoffResult expected,
    const EstimatorLane *active,
    const EstimatorLane *shadow,
    const HandoffRequest *request,
    const char *message
)
{
    EstimatorLane output;
    EstimatorLane before;
    HandoffSession session;

    memcpy(&output, active, sizeof(output));
    memcpy(&before, &output, sizeof(before));
    memset(&session, 0, sizeof(session));
    check_true(
        try_complete_handoff(&session, &output, active, shadow, request) == expected,
        message
    );
    check_true(!session.switched && session.switch_count == 0U,
        "blocked transaction never marks a lane as switched");
    check_output_unchanged(&before, &output,
        "blocked transaction leaves the preexisting output lane untouched");
}

static void test_fail_closed_preconditions(void)
{
    EstimatorLane active;
    EstimatorLane shadow;
    EstimatorLane altered_shadow;
    HandoffRequest request;

    check_true(prepare_latched_lanes(&active, &shadow, &request, DEFAULT_IMU_DT_US),
        "failure matrix obtains a qualified physical source latch");

    request.fault_latched = false;
    expect_blocked(HANDOFF_BLOCK_NOT_LATCHED, &active, &shadow, &request,
        "handoff without a supervisor latch fails closed");
    request.fault_latched = true;
    request.return_to_barometer_lane = true;
    expect_blocked(HANDOFF_BLOCK_RETURN_AUTHORIZATION_REQUIRED, &active, &shadow, &request,
        "automatic return to the contaminated lane requires external authorization");
    request.return_to_barometer_lane = false;

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.filter.last_timestamp_us -= 1U;
    expect_blocked(HANDOFF_BLOCK_TIMESTAMP, &active, &altered_shadow, &request,
        "shadow timestamp skew blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.provenance.common_event_hash ^= UINT64_C(1);
    expect_blocked(HANDOFF_BLOCK_PROVENANCE, &active, &altered_shadow, &request,
        "different common input provenance blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.provenance.configuration_id++;
    expect_blocked(HANDOFF_BLOCK_CONFIGURATION, &active, &altered_shadow, &request,
        "different lane configuration identity blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.provenance.source_generation++;
    expect_blocked(HANDOFF_BLOCK_SOURCE_IDENTITY, &active, &altered_shadow, &request,
        "different source generation blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.filter.static_alignment_complete = false;
    expect_blocked(HANDOFF_BLOCK_ALIGNMENT, &active, &altered_shadow, &request,
        "incomplete shadow alignment blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.filter.core.state.q[0] = 2.0;
    expect_blocked(HANDOFF_BLOCK_SHADOW_UNHEALTHY, &active, &altered_shadow, &request,
        "finite but non-normalized shadow attitude blocks handoff");

    memcpy(&altered_shadow, &shadow, sizeof(altered_shadow));
    altered_shadow.filter.core.P[0][0] = NAN;
    expect_blocked(HANDOFF_BLOCK_NONFINITE, &active, &altered_shadow, &request,
        "non-finite shadow covariance blocks handoff");
}

int main(void)
{
    test_successful_full_state_handoff();
    test_fail_closed_preconditions();
    if (failures != 0) {
        fprintf(stderr, "%d barometer multi-lane handoff checks failed\n", failures);
        return 1;
    }
    printf(
        "barometer multi-lane handoff contract passed: %u rates (last %.1f Hz), %u IMU, %u P, %u V, %u heading events; "
        "reset attitude %.9g rad, position %.9g m, velocity %.9g m/s, covariance %.9g\n",
        successful_rate_cases, 1.0e6 / (double)successful_last_interval_us,
        successful_provenance.imu_events, successful_provenance.position_events,
        successful_provenance.velocity_events, successful_provenance.heading_events,
        successful_session.attitude_reset_rad,
        successful_session.position_reset_norm_m,
        successful_session.velocity_reset_norm_m_s,
        successful_session.covariance_reset_max_abs
    );
    return 0;
}
