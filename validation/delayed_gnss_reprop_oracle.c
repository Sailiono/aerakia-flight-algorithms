#define _CRT_SECURE_NO_WARNINGS
/**
 * @file delayed_gnss_reprop_oracle.c
 * @brief Host-only isolated delayed-GNSS rewind/repropagation oracle.
 *
 * This is a validation experiment, not a flight feature.  It keeps a bounded
 * ring of complete AerakiaEskf snapshots and exact IMU/P-V events, rewinds an
 * isolated delayed GNSS observation to its source timestamp, and replays the
 * intervening stream. The sequential mode repeats that transaction only after
 * the first delivery completes. The separate overlap mode rebuilds from the
 * earliest affected snapshot using only events delivered so far, then compares
 * only the fully delivered lane with an otherwise identical zero-delay run.
 *
 * Only the timestamped P/V contract is exercised.  The experiment deliberately
 * excludes magnetic, heading, barometer, supervisor, and multi-IMU behavior.
 */

#include <aerakia/eskf_adapter.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ORACLE_HISTORY_CAPACITY 64U
#define ORACLE_GPS_INTERVAL_US 100000U
#define ORACLE_OVERLAP_GPS_INTERVAL_US 20000U
#define ORACLE_OVERLAP_MIN_DELAY_MS 50U
#define ORACLE_FIRST_SOURCE_TIMESTAMP_US 3000000U
#define ORACLE_SECOND_SOURCE_TIMESTAMP_US 4000000U
#define ORACLE_DURATION_US 6000000U
#define ORACLE_MAX_DELAY_MS 150U

typedef struct {
    uint64_t sequence;
    AerakiaImuSample sample;
    AerakiaGpsObservation gps;
    bool gps_update;
    bool valid;
    AerakiaEskf before_gps;
} HistoryEntry;

typedef struct {
    double max_state_difference;
    double max_covariance_difference;
    bool metadata_match;
} FilterDifference;

typedef struct {
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
} TruthState;

typedef enum {
    ORACLE_SCENARIO_SINGLE = 0,
    ORACLE_SCENARIO_SEQUENTIAL = 1,
    ORACLE_SCENARIO_OVERLAP = 2
} OracleScenario;

typedef struct {
    uint64_t source_sequence;
    uint64_t delivery_sequence;
    bool source_accepted;
    bool delivered;
    bool repropagated;
    unsigned pending_earlier_events_at_delivery;
    bool zero_delay_equivalence_required_after_delivery;
    double pre_delivery_max_state_difference;
    double post_delivery_max_state_difference;
    double post_delivery_max_covariance_difference;
    bool post_delivery_metadata_match;
} DelayedEvent;

static void usage(const char *program)
{
    fprintf(
        stderr,
        "Usage: %s [--rate-hz HZ] [--delay-ms MS] "
        "[--scenario single|sequential|overlap] [--out PATH]\n",
        program
    );
}

static bool parse_unsigned(const char *text, unsigned *value)
{
    char *end = NULL;
    unsigned long parsed;
    if (text == NULL || value == NULL || text[0] == '\0') return false;
    parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed > UINT32_MAX) return false;
    *value = (unsigned)parsed;
    return true;
}

static bool parse_scenario(const char *text, OracleScenario *scenario)
{
    if (text == NULL || scenario == NULL) return false;
    if (strcmp(text, "single") == 0) {
        *scenario = ORACLE_SCENARIO_SINGLE;
        return true;
    }
    if (strcmp(text, "sequential") == 0) {
        *scenario = ORACLE_SCENARIO_SEQUENTIAL;
        return true;
    }
    if (strcmp(text, "overlap") == 0) {
        *scenario = ORACLE_SCENARIO_OVERLAP;
        return true;
    }
    return false;
}

static const char *scenario_name(OracleScenario scenario)
{
    if (scenario == ORACLE_SCENARIO_SEQUENTIAL) {
        return "sequential_two_non_overlapping";
    }
    if (scenario == ORACLE_SCENARIO_OVERLAP) {
        return "overlapping_reordered_two_event";
    }
    return "single_isolated";
}

static double state_component_difference(const ESKF_NominalState *first,
                                         const ESKF_NominalState *second)
{
    double maximum = 0.0;
    int axis;
    if (first == NULL || second == NULL) return INFINITY;
    for (axis = 0; axis < 3; ++axis) {
        maximum = fmax(maximum, fabs(first->p[axis] - second->p[axis]));
        maximum = fmax(maximum, fabs(first->v[axis] - second->v[axis]));
        maximum = fmax(maximum, fabs(first->ab[axis] - second->ab[axis]));
        maximum = fmax(maximum, fabs(first->gb[axis] - second->gb[axis]));
    }
    for (axis = 0; axis < 4; ++axis) {
        maximum = fmax(maximum, fabs(first->q[axis] - second->q[axis]));
    }
    return maximum;
}

static FilterDifference compare_filters(const AerakiaEskf *first,
                                        const AerakiaEskf *second)
{
    FilterDifference difference;
    int row;
    int column;
    difference.max_state_difference = INFINITY;
    difference.max_covariance_difference = INFINITY;
    difference.metadata_match = false;
    if (first == NULL || second == NULL) return difference;
    difference.max_state_difference = state_component_difference(
        &first->core.state, &second->core.state
    );
    difference.max_covariance_difference = 0.0;
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            difference.max_covariance_difference = fmax(
                difference.max_covariance_difference,
                fabs(first->core.P[row][column] - second->core.P[row][column])
            );
        }
    }
    difference.metadata_match = first->last_timestamp_us == second->last_timestamp_us
        && first->last_gps_timestamp_us == second->last_gps_timestamp_us
        && first->last_position_timestamp_us == second->last_position_timestamp_us
        && first->last_velocity_timestamp_us == second->last_velocity_timestamp_us
        && first->has_timestamp == second->has_timestamp
        && first->has_gps_timestamp == second->has_gps_timestamp
        && first->has_position_timestamp == second->has_position_timestamp
        && first->has_velocity_timestamp == second->has_velocity_timestamp
        && first->rejected_samples == second->rejected_samples
        && first->position_accepted == second->position_accepted
        && first->velocity_accepted == second->velocity_accepted
        && first->navigation_recovered == second->navigation_recovered
        && first->navigation_recovery_count == second->navigation_recovery_count
        && first->consecutive_navigation_rejections
            == second->consecutive_navigation_rejections
        && first->consecutive_position_rejections == second->consecutive_position_rejections
        && first->consecutive_velocity_rejections == second->consecutive_velocity_rejections
        && first->navigation_recovery_probationary
            == second->navigation_recovery_probationary
        && first->horizontal_position_initialized == second->horizontal_position_initialized;
    return difference;
}

static bool state_is_finite(const ESKF_NominalState *state)
{
    int axis;
    if (state == NULL) return false;
    for (axis = 0; axis < 3; ++axis) {
        if (!isfinite(state->p[axis]) || !isfinite(state->v[axis])
            || !isfinite(state->ab[axis]) || !isfinite(state->gb[axis])) {
            return false;
        }
    }
    for (axis = 0; axis < 4; ++axis) {
        if (!isfinite(state->q[axis])) return false;
    }
    return true;
}

static bool covariance_is_symmetric_psd(eskf_float_t covariance[15][15])
{
    double lower[15][15] = {{0.0}};
    int row;
    int column;
    int index;
    if (covariance == NULL) return false;
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column <= row; ++column) {
            double sum = covariance[row][column];
            if (!isfinite(sum)
                || fabs(covariance[row][column] - covariance[column][row]) > 1.0e-9) {
                return false;
            }
            for (index = 0; index < column; ++index) {
                sum -= lower[row][index] * lower[column][index];
            }
            if (row == column) {
                if (sum < -1.0e-9) return false;
                lower[row][column] = sqrt(sum > 0.0 ? sum : 0.0);
            } else if (lower[column][column] > 1.0e-12) {
                lower[row][column] = sum / lower[column][column];
            } else if (fabs(sum) > 1.0e-8) {
                return false;
            }
        }
    }
    return true;
}

static void quaternion_from_euler(double roll, double pitch, double yaw, double q[4])
{
    const double cr = cos(0.5 * roll);
    const double sr = sin(0.5 * roll);
    const double cp = cos(0.5 * pitch);
    const double sp = sin(0.5 * pitch);
    const double cy = cos(0.5 * yaw);
    const double sy = sin(0.5 * yaw);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void true_acceleration(double time_s, AerakiaVec3f *acceleration)
{
    if (acceleration == NULL) return;
    acceleration->x = (float)(0.45 * sin(0.85 * time_s) + 0.13 * cos(0.23 * time_s));
    acceleration->y = (float)(-0.35 * cos(0.61 * time_s) + 0.08 * sin(1.15 * time_s));
    acceleration->z = (float)(0.08 * sin(0.37 * time_s));
}

static AerakiaImuSample make_sample(uint64_t timestamp_us)
{
    AerakiaImuSample sample;
    AerakiaVec3f acceleration;
    memset(&sample, 0, sizeof(sample));
    true_acceleration((double)timestamp_us * 1.0e-6, &acceleration);
    sample.timestamp_us = timestamp_us;
    sample.acceleration_m_s2 = acceleration;
    sample.acceleration_m_s2.z -= AERAKIA_GRAVITY_M_S2;
    sample.angular_rate_rad_s.x = 0.0f;
    sample.angular_rate_rad_s.y = 0.0f;
    sample.angular_rate_rad_s.z = 0.0f;
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID;
    return sample;
}

static void advance_truth(TruthState *truth, const AerakiaVec3f acceleration, double dt_s)
{
    if (truth == NULL) return;
    truth->position_ned_m.x += truth->velocity_ned_m_s.x * (float)dt_s
        + 0.5f * acceleration.x * (float)(dt_s * dt_s);
    truth->position_ned_m.y += truth->velocity_ned_m_s.y * (float)dt_s
        + 0.5f * acceleration.y * (float)(dt_s * dt_s);
    truth->position_ned_m.z += truth->velocity_ned_m_s.z * (float)dt_s
        + 0.5f * acceleration.z * (float)(dt_s * dt_s);
    truth->velocity_ned_m_s.x += acceleration.x * (float)dt_s;
    truth->velocity_ned_m_s.y += acceleration.y * (float)dt_s;
    truth->velocity_ned_m_s.z += acceleration.z * (float)dt_s;
}

static AerakiaGpsObservation make_gps_observation(
    uint64_t timestamp_us, const TruthState *truth, uint64_t quality_sequence
)
{
    AerakiaGpsObservation observation;
    memset(&observation, 0, sizeof(observation));
    observation.timestamp_us = timestamp_us;
    observation.position_ned_m = truth->position_ned_m;
    observation.velocity_ned_m_s = truth->velocity_ned_m_s;
    observation.position_variance_m2 = 9.0f;
    observation.velocity_variance_m2_s2 = 1.0f;
    observation.source_id = 1U;
    observation.source_generation = 1U;
    observation.quality_sequence = quality_sequence;
    return observation;
}

static bool history_store(HistoryEntry history[ORACLE_HISTORY_CAPACITY],
                          uint64_t sequence,
                          const AerakiaImuSample *sample,
                          const AerakiaGpsObservation *gps,
                          bool gps_update,
                          const AerakiaEskf *before_gps)
{
    HistoryEntry *entry;
    if (history == NULL || sample == NULL || gps == NULL || before_gps == NULL) return false;
    entry = &history[sequence % ORACLE_HISTORY_CAPACITY];
    entry->sequence = sequence;
    entry->sample = *sample;
    entry->gps = *gps;
    entry->gps_update = gps_update;
    entry->before_gps = *before_gps;
    entry->valid = true;
    return true;
}

static HistoryEntry *history_find(HistoryEntry history[ORACLE_HISTORY_CAPACITY],
                                  uint64_t sequence)
{
    HistoryEntry *entry;
    if (history == NULL) return NULL;
    entry = &history[sequence % ORACLE_HISTORY_CAPACITY];
    if (!entry->valid || entry->sequence != sequence) return NULL;
    return entry;
}

static int event_index_for_source(const DelayedEvent *events,
                                  unsigned event_count,
                                  uint64_t sequence)
{
    unsigned index;
    if (events == NULL) return -1;
    for (index = 0U; index < event_count; ++index) {
        if (events[index].source_sequence == sequence) return (int)index;
    }
    return -1;
}

static int event_index_for_delivery(const DelayedEvent *events,
                                    unsigned event_count,
                                    uint64_t sequence)
{
    unsigned index;
    if (events == NULL) return -1;
    for (index = 0U; index < event_count; ++index) {
        if (events[index].delivery_sequence == sequence) return (int)index;
    }
    return -1;
}

static bool process_sample(AerakiaEskf *filter, const AerakiaImuSample *sample)
{
    AerakiaNavigationEstimate estimate;
    AerakiaStatus status;
    memset(&estimate, 0, sizeof(estimate));
    status = aerakia_eskf_process_imu(filter, sample, &estimate);
    return status == AERAKIA_STATUS_INITIALIZED || status == AERAKIA_STATUS_ALIGNING
        || status == AERAKIA_STATUS_OK;
}

static bool apply_gps(AerakiaEskf *filter, const AerakiaGpsObservation *observation)
{
    return aerakia_eskf_update_gps_observation(filter, observation) == AERAKIA_STATUS_OK
        && filter->position_accepted && filter->velocity_accepted;
}

static bool repropagate_from_source(
    AerakiaEskf *filter,
    HistoryEntry history[ORACLE_HISTORY_CAPACITY],
    uint64_t source_sequence,
    uint64_t delivery_sequence
)
{
    HistoryEntry *source_entry;
    AerakiaEskf replay;
    uint64_t sequence;
    if (filter == NULL || source_sequence >= delivery_sequence) return false;
    source_entry = history_find(history, source_sequence);
    if (source_entry == NULL || !source_entry->gps_update) return false;
    replay = source_entry->before_gps;
    if (!apply_gps(&replay, &source_entry->gps)) return false;
    for (sequence = source_sequence + 1U; sequence <= delivery_sequence; ++sequence) {
        HistoryEntry *entry = history_find(history, sequence);
        if (entry == NULL || !process_sample(&replay, &entry->sample)) return false;
        if (entry->gps_update && !apply_gps(&replay, &entry->gps)) return false;
    }
    *filter = replay;
    return true;
}

/*
 * Rebuild a delayed lane from the earliest affected full snapshot.  Unlike the
 * isolated oracle path above, this applies only source events that have already
 * arrived.  That prevents a newer pending event from being fused during an
 * earlier event's replay, which is the essential overlap/reordering boundary.
 */
static bool rebuild_with_delivered_events(
    AerakiaEskf *filter,
    HistoryEntry history[ORACLE_HISTORY_CAPACITY],
    const DelayedEvent *events,
    unsigned event_count,
    uint64_t anchor_sequence,
    uint64_t delivery_sequence
)
{
    HistoryEntry *anchor;
    AerakiaEskf replay;
    uint64_t sequence;

    if (filter == NULL || history == NULL || events == NULL || event_count == 0U
        || anchor_sequence >= delivery_sequence) {
        return false;
    }
    anchor = history_find(history, anchor_sequence);
    if (anchor == NULL || !anchor->gps_update) return false;
    replay = anchor->before_gps;

    for (sequence = anchor_sequence; sequence <= delivery_sequence; ++sequence) {
        HistoryEntry *entry = history_find(history, sequence);
        int source_event;
        bool apply_observation;
        if (entry == NULL) return false;
        if (sequence > anchor_sequence && !process_sample(&replay, &entry->sample)) {
            return false;
        }
        if (!entry->gps_update) continue;
        source_event = event_index_for_source(events, event_count, sequence);
        apply_observation = source_event < 0 || events[(unsigned)source_event].delivered;
        if (apply_observation && !apply_gps(&replay, &entry->gps)) return false;
    }
    *filter = replay;
    return true;
}

static bool initialize_filter(AerakiaEskf *filter)
{
    AerakiaEskfConfig config;
    double initial_position[3] = {2.0, -1.0, 0.5};
    double initial_quaternion[4];
    if (filter == NULL) return false;
    quaternion_from_euler(0.035, -0.045, 0.01, initial_quaternion);
    aerakia_eskf_default_config(&config);
    config.enable_static_alignment = false;
    config.fuse_magnetometer = false;
    config.maximum_aiding_age_s = 0.5f;
    aerakia_eskf_init(filter, &config, initial_position, initial_quaternion);
    filter->core.state.v[0] = 0.10;
    filter->core.state.v[1] = -0.05;
    filter->core.state.v[2] = 0.02;
    filter->core.state.ab[0] = 0.02;
    filter->core.state.ab[1] = -0.015;
    filter->core.state.ab[2] = 0.01;
    filter->core.state.gb[0] = 0.0004;
    filter->core.state.gb[1] = -0.0003;
    filter->core.state.gb[2] = 0.0002;
    return true;
}

static void write_event_json(FILE *output,
                             const DelayedEvent *event,
                             uint64_t dt_us,
                             unsigned event_index,
                             bool trailing_comma)
{
    if (output == NULL || event == NULL) return;
    fprintf(
        output,
        "    {\n"
        "      \"event_index\": %u,\n"
        "      \"source_timestamp_us\": %llu,\n"
        "      \"delivery_timestamp_us\": %llu,\n"
        "      \"source_gps_accepted\": %s,\n"
        "      \"repropagated\": %s,\n"
        "      \"pre_delivery_max_state_difference\": %.17g,\n"
        "      \"post_delivery_max_state_difference\": %.17g,\n"
        "      \"post_delivery_max_covariance_difference\": %.17g,\n"
        "      \"post_delivery_metadata_match\": %s\n"
        "    }%s\n",
        event_index,
        (unsigned long long)(event->source_sequence * dt_us),
        (unsigned long long)(event->delivery_sequence * dt_us),
        event->source_accepted ? "true" : "false",
        event->repropagated ? "true" : "false",
        event->pre_delivery_max_state_difference,
        event->post_delivery_max_state_difference,
        event->post_delivery_max_covariance_difference,
        event->post_delivery_metadata_match ? "true" : "false",
        trailing_comma ? "," : ""
    );
}

static void write_overlapping_event_json(FILE *output,
                                         const DelayedEvent *event,
                                         uint64_t dt_us,
                                         unsigned event_index,
                                         bool trailing_comma)
{
    if (output == NULL || event == NULL) return;
    fprintf(
        output,
        "    {\n"
        "      \"event_index\": %u,\n"
        "      \"source_timestamp_us\": %llu,\n"
        "      \"delivery_timestamp_us\": %llu,\n"
        "      \"source_gps_accepted\": %s,\n"
        "      \"repropagated\": %s,\n"
        "      \"pending_earlier_event_count_at_delivery\": %u,\n"
        "      \"zero_delay_equivalence_required_after_delivery\": %s,\n"
        "      \"pre_delivery_max_state_difference\": %.17g,\n"
        "      \"post_delivery_max_state_difference\": %.17g,\n"
        "      \"post_delivery_max_covariance_difference\": %.17g,\n"
        "      \"post_delivery_metadata_match\": %s\n"
        "    }%s\n",
        event_index,
        (unsigned long long)(event->source_sequence * dt_us),
        (unsigned long long)(event->delivery_sequence * dt_us),
        event->source_accepted ? "true" : "false",
        event->repropagated ? "true" : "false",
        event->pending_earlier_events_at_delivery,
        event->zero_delay_equivalence_required_after_delivery ? "true" : "false",
        event->pre_delivery_max_state_difference,
        event->post_delivery_max_state_difference,
        event->post_delivery_max_covariance_difference,
        event->post_delivery_metadata_match ? "true" : "false",
        trailing_comma ? "," : ""
    );
}

static unsigned pending_earlier_event_count(const DelayedEvent *events,
                                            unsigned event_count,
                                            unsigned event_index)
{
    unsigned index;
    unsigned pending = 0U;
    if (events == NULL || event_index >= event_count) return UINT32_MAX;
    for (index = 0U; index < event_index; ++index) {
        if (!events[index].delivered) pending++;
    }
    return pending;
}

static bool run_overlapping_case(unsigned rate_hz, unsigned delay_ms, FILE *output)
{
    AerakiaEskf baseline;
    AerakiaEskf delayed;
    HistoryEntry history[ORACLE_HISTORY_CAPACITY];
    TruthState truth;
    DelayedEvent events[2];
    uint64_t dt_us;
    uint64_t delay_us;
    uint64_t delay_samples;
    uint64_t duration_samples;
    uint64_t gps_interval_samples;
    uint64_t sequence;
    double final_state_difference = 0.0;
    double final_covariance_difference = 0.0;
    bool final_metadata_match = true;
    bool healthy = true;
    bool covariance_psd = true;
    bool final_delivery_seen = false;
    const double state_tolerance =
#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
        2.0e-5;
#else
        1.0e-12;
#endif
    const double covariance_tolerance =
#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
        2.0e-5;
#else
        1.0e-12;
#endif

    if (rate_hz == 0U || 1000000U % rate_hz != 0U
        || delay_ms < ORACLE_OVERLAP_MIN_DELAY_MS || delay_ms > ORACLE_MAX_DELAY_MS) {
        return false;
    }
    dt_us = 1000000U / rate_hz;
    delay_us = (uint64_t)delay_ms * 1000U;
    if (delay_us % dt_us != 0U || ORACLE_OVERLAP_GPS_INTERVAL_US % dt_us != 0U) {
        return false;
    }
    delay_samples = delay_us / dt_us;
    gps_interval_samples = ORACLE_OVERLAP_GPS_INTERVAL_US / dt_us;
    if (delay_samples + 1U > ORACLE_HISTORY_CAPACITY
        || gps_interval_samples == 0U || delay_samples <= gps_interval_samples + 1U) {
        return false;
    }
    duration_samples = ORACLE_DURATION_US / dt_us;
    memset(&history, 0, sizeof(history));
    memset(&truth, 0, sizeof(truth));
    memset(&events, 0, sizeof(events));
    events[0].source_sequence = ORACLE_FIRST_SOURCE_TIMESTAMP_US / dt_us;
    events[0].delivery_sequence = events[0].source_sequence + delay_samples;
    events[1].source_sequence = events[0].source_sequence + gps_interval_samples;
    events[1].delivery_sequence = events[1].source_sequence + 1U;
    if (events[0].source_sequence == 0U
        || events[0].source_sequence % gps_interval_samples != 0U
        || events[1].source_sequence % gps_interval_samples != 0U
        || events[1].delivery_sequence >= events[0].delivery_sequence
        || events[0].delivery_sequence >= duration_samples) {
        return false;
    }
    if (!initialize_filter(&baseline) || !initialize_filter(&delayed)) return false;

    for (sequence = 0U; sequence < duration_samples; ++sequence) {
        const uint64_t timestamp_us = sequence * dt_us;
        const AerakiaImuSample sample = make_sample(timestamp_us);
        const bool gps_update = sequence > 0U && sequence % gps_interval_samples == 0U;
        int source_event;
        int delivery_event;
        AerakiaGpsObservation gps;
        FilterDifference difference;
        unsigned event_index;

        if (sequence > 0U) {
            AerakiaVec3f acceleration;
            true_acceleration((double)timestamp_us * 1.0e-6, &acceleration);
            advance_truth(&truth, acceleration, (double)dt_us * 1.0e-6);
        }
        gps = make_gps_observation(timestamp_us, &truth, sequence / gps_interval_samples + 1U);
        if (!process_sample(&baseline, &sample) || !process_sample(&delayed, &sample)) {
            return false;
        }
        if (!history_store(history, sequence, &sample, &gps, gps_update, &delayed)) {
            return false;
        }
        source_event = event_index_for_source(events, 2U, sequence);
        delivery_event = event_index_for_delivery(events, 2U, sequence);
        if (gps_update) {
            if (!apply_gps(&baseline, &gps)) return false;
            if (source_event >= 0) {
                events[(unsigned)source_event].source_accepted = true;
            }
        }
        if (gps_update && source_event < 0 && !apply_gps(&delayed, &gps)) return false;

        difference = compare_filters(&baseline, &delayed);
        for (event_index = 0U; event_index < 2U; ++event_index) {
            DelayedEvent *event = &events[event_index];
            if (sequence > event->source_sequence && sequence <= event->delivery_sequence) {
                event->pre_delivery_max_state_difference = fmax(
                    event->pre_delivery_max_state_difference, difference.max_state_difference
                );
            }
        }
        if (delivery_event >= 0) {
            DelayedEvent *event = &events[(unsigned)delivery_event];
            event->delivered = true;
            event->pending_earlier_events_at_delivery = pending_earlier_event_count(
                events, 2U, (unsigned)delivery_event
            );
            event->zero_delay_equivalence_required_after_delivery =
                event->pending_earlier_events_at_delivery == 0U;
            if (!rebuild_with_delivered_events(
                    &delayed, history, events, 2U, events[0].source_sequence, sequence
                )) {
                return false;
            }
            event->repropagated = true;
            difference = compare_filters(&baseline, &delayed);
            event->post_delivery_max_state_difference = difference.max_state_difference;
            event->post_delivery_max_covariance_difference = difference.max_covariance_difference;
            event->post_delivery_metadata_match = difference.metadata_match;
            if (event->zero_delay_equivalence_required_after_delivery) {
                final_delivery_seen = true;
            }
        }

        healthy = healthy && state_is_finite(&baseline.core.state)
            && state_is_finite(&delayed.core.state)
            && covariance_is_symmetric_psd(baseline.core.P)
            && covariance_is_symmetric_psd(delayed.core.P);
        covariance_psd = covariance_psd && covariance_is_symmetric_psd(baseline.core.P)
            && covariance_is_symmetric_psd(delayed.core.P);
        if (final_delivery_seen) {
            difference = compare_filters(&baseline, &delayed);
            final_state_difference = fmax(final_state_difference, difference.max_state_difference);
            final_covariance_difference = fmax(
                final_covariance_difference, difference.max_covariance_difference
            );
            final_metadata_match = final_metadata_match && difference.metadata_match;
        }
    }

    {
        const DelayedEvent *older = &events[0];
        const DelayedEvent *newer = &events[1];
        const bool overlap = newer->source_sequence < older->delivery_sequence;
        const bool reverse_delivery = newer->delivery_sequence < older->delivery_sequence;
        const bool pass = healthy && covariance_psd && overlap && reverse_delivery
            && older->source_accepted && newer->source_accepted
            && older->repropagated && newer->repropagated
            && older->pre_delivery_max_state_difference > state_tolerance
            && newer->pre_delivery_max_state_difference > state_tolerance
            && newer->pending_earlier_events_at_delivery == 1U
            && !newer->zero_delay_equivalence_required_after_delivery
            && newer->post_delivery_max_state_difference > state_tolerance
            && older->pending_earlier_events_at_delivery == 0U
            && older->zero_delay_equivalence_required_after_delivery
            && older->post_delivery_max_state_difference <= state_tolerance
            && older->post_delivery_max_covariance_difference <= covariance_tolerance
            && older->post_delivery_metadata_match
            && final_delivery_seen && final_state_difference <= state_tolerance
            && final_covariance_difference <= covariance_tolerance && final_metadata_match;

        if (output != NULL) {
            fprintf(
                output,
                "{\n"
                "  \"schema_version\": 1,\n"
                "  \"status\": \"host_only_research_oracle_not_flight_feature\",\n"
                "  \"scenario\": \"overlapping_reordered_two_event\",\n"
                "  \"input_contract\": \"synthetic_exact_timestamp_pv_only\",\n"
                "  \"rate_hz\": %u,\n"
                "  \"delay_ms\": %u,\n"
                "  \"gps_interval_us\": %u,\n"
                "  \"history_capacity_samples\": %u,\n"
                "  \"delay_samples\": %llu,\n"
                "  \"overlapping_delivery_windows\": %s,\n"
                "  \"delivery_order_reversed\": %s,\n"
                "  \"healthy\": %s,\n"
                "  \"covariance_psd\": %s,\n"
                "  \"state_tolerance\": %.17g,\n"
                "  \"covariance_tolerance\": %.17g,\n"
                "  \"event_count\": 2,\n"
                "  \"events\": [\n",
                rate_hz, delay_ms, ORACLE_OVERLAP_GPS_INTERVAL_US,
                ORACLE_HISTORY_CAPACITY, (unsigned long long)delay_samples,
                overlap ? "true" : "false", reverse_delivery ? "true" : "false",
                healthy ? "true" : "false", covariance_psd ? "true" : "false",
                state_tolerance, covariance_tolerance
            );
            write_overlapping_event_json(output, older, dt_us, 1U, true);
            write_overlapping_event_json(output, newer, dt_us, 2U, false);
            fprintf(
                output,
                "  ],\n"
                "  \"final_max_state_difference\": %.17g,\n"
                "  \"final_max_covariance_difference\": %.17g,\n"
                "  \"final_metadata_match\": %s,\n"
                "  \"pass\": %s,\n"
                "  \"limitations\": [\n"
                "    \"Exactly two GNSS P/V events overlap, and the newer source is delivered first.\",\n"
                "    \"The host oracle rebuilds from the earliest retained full snapshot using only events delivered so far.\",\n"
                "    \"This is not a production OOSM buffer, controller policy, or target-resource measurement.\",\n"
                "    \"The experiment does not validate delayed heading, barometer, multi-IMU, interpolation, or physical source-arrival timing.\",\n"
                "    \"Truth creates the synthetic P/V observation only and is not read by replay logic.\"\n"
                "  ]\n"
                "}\n",
                final_state_difference, final_covariance_difference,
                final_metadata_match ? "true" : "false", pass ? "true" : "false"
            );
        }
        return pass;
    }
}

static bool run_case(unsigned rate_hz,
                     unsigned delay_ms,
                     OracleScenario scenario,
                     FILE *output)
{
    AerakiaEskf baseline;
    AerakiaEskf delayed;
    HistoryEntry history[ORACLE_HISTORY_CAPACITY];
    TruthState truth;
    DelayedEvent events[2];
    uint64_t dt_us;
    uint64_t delay_us;
    uint64_t delay_samples;
    uint64_t duration_samples;
    uint64_t gps_interval_samples;
    unsigned event_count;
    double final_state_difference = 0.0;
    double final_covariance_difference = 0.0;
    bool final_metadata_match = true;
    bool healthy = true;
    bool covariance_psd = true;
    bool sequential_non_overlapping = false;
    uint64_t sequence;
    unsigned event_index;
    const double state_tolerance =
#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
        2.0e-5;
#else
        1.0e-12;
#endif
    const double covariance_tolerance =
#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
        2.0e-5;
#else
        1.0e-12;
#endif

    if (rate_hz == 0U || 1000000U % rate_hz != 0U || delay_ms > ORACLE_MAX_DELAY_MS) {
        return false;
    }
    dt_us = 1000000U / rate_hz;
    delay_us = (uint64_t)delay_ms * 1000U;
    if (delay_us == 0U || delay_us % dt_us != 0U) return false;
    delay_samples = delay_us / dt_us;
    if (delay_samples == 0U || delay_samples + 1U > ORACLE_HISTORY_CAPACITY) return false;
    if (ORACLE_GPS_INTERVAL_US % dt_us != 0U) return false;
    duration_samples = ORACLE_DURATION_US / dt_us;
    gps_interval_samples = ORACLE_GPS_INTERVAL_US / dt_us;
    event_count = scenario == ORACLE_SCENARIO_SEQUENTIAL ? 2U : 1U;
    memset(&history, 0, sizeof(history));
    memset(&truth, 0, sizeof(truth));
    memset(&events, 0, sizeof(events));
    events[0].source_sequence = ORACLE_FIRST_SOURCE_TIMESTAMP_US / dt_us;
    events[0].delivery_sequence = events[0].source_sequence + delay_samples;
    events[0].post_delivery_metadata_match = true;
    if (event_count == 2U) {
        events[1].source_sequence = ORACLE_SECOND_SOURCE_TIMESTAMP_US / dt_us;
        events[1].delivery_sequence = events[1].source_sequence + delay_samples;
        events[1].post_delivery_metadata_match = true;
        sequential_non_overlapping = events[1].source_sequence > events[0].delivery_sequence;
    }
    for (event_index = 0U; event_index < event_count; ++event_index) {
        if (events[event_index].source_sequence == 0U
            || events[event_index].source_sequence % gps_interval_samples != 0U
            || events[event_index].delivery_sequence >= duration_samples) {
            return false;
        }
    }
    if (event_count == 2U && !sequential_non_overlapping) return false;
    if (!initialize_filter(&baseline) || !initialize_filter(&delayed)) return false;

    for (sequence = 0U; sequence < duration_samples; ++sequence) {
        const uint64_t timestamp_us = sequence * dt_us;
        const AerakiaImuSample sample = make_sample(timestamp_us);
        const bool gps_update = sequence > 0U && sequence % gps_interval_samples == 0U;
        int source_event;
        int delivery_event;
        AerakiaGpsObservation gps;
        FilterDifference difference;
        if (sequence > 0U) {
            AerakiaVec3f acceleration;
            true_acceleration((double)timestamp_us * 1.0e-6, &acceleration);
            advance_truth(&truth, acceleration, (double)dt_us * 1.0e-6);
        }
        gps = make_gps_observation(timestamp_us, &truth, sequence / gps_interval_samples + 1U);
        if (!process_sample(&baseline, &sample) || !process_sample(&delayed, &sample)) {
            return false;
        }
        if (!history_store(history, sequence, &sample, &gps, gps_update, &delayed)) {
            return false;
        }
        source_event = event_index_for_source(events, event_count, sequence);
        delivery_event = event_index_for_delivery(events, event_count, sequence);
        if (gps_update) {
            if (!apply_gps(&baseline, &gps)) return false;
            if (source_event >= 0) {
                events[(unsigned)source_event].source_accepted = true;
            }
        }
        if (gps_update && source_event < 0 && delivery_event < 0) {
            if (!apply_gps(&delayed, &gps)) return false;
        }
        if (delivery_event >= 0) {
            DelayedEvent *event = &events[(unsigned)delivery_event];
            if (!repropagate_from_source(
                    &delayed, history, event->source_sequence, event->delivery_sequence
                )) {
                return false;
            }
            event->repropagated = true;
        }
        healthy = healthy && state_is_finite(&baseline.core.state)
            && state_is_finite(&delayed.core.state)
            && covariance_is_symmetric_psd(baseline.core.P)
            && covariance_is_symmetric_psd(delayed.core.P);
        covariance_psd = covariance_psd && covariance_is_symmetric_psd(baseline.core.P)
            && covariance_is_symmetric_psd(delayed.core.P);
        difference = compare_filters(&baseline, &delayed);
        for (event_index = 0U; event_index < event_count; ++event_index) {
            DelayedEvent *event = &events[event_index];
            const uint64_t post_stop_sequence = event_index + 1U < event_count
                ? events[event_index + 1U].source_sequence
                : duration_samples;
            if (sequence > event->source_sequence && sequence < event->delivery_sequence) {
                event->pre_delivery_max_state_difference = fmax(
                    event->pre_delivery_max_state_difference,
                    difference.max_state_difference
                );
            }
            if (sequence >= event->delivery_sequence && sequence < post_stop_sequence) {
                event->post_delivery_max_state_difference = fmax(
                    event->post_delivery_max_state_difference,
                    difference.max_state_difference
                );
                event->post_delivery_max_covariance_difference = fmax(
                    event->post_delivery_max_covariance_difference,
                    difference.max_covariance_difference
                );
                event->post_delivery_metadata_match = event->post_delivery_metadata_match
                    && difference.metadata_match;
            }
        }
        if (sequence >= events[event_count - 1U].delivery_sequence) {
            final_state_difference = fmax(final_state_difference, difference.max_state_difference);
            final_covariance_difference = fmax(
                final_covariance_difference, difference.max_covariance_difference
            );
            final_metadata_match = final_metadata_match && difference.metadata_match;
        }
    }

    {
        bool pass = healthy && covariance_psd && final_state_difference <= state_tolerance
            && final_covariance_difference <= covariance_tolerance && final_metadata_match;
        for (event_index = 0U; event_index < event_count; ++event_index) {
            const DelayedEvent *event = &events[event_index];
            pass = pass && event->source_accepted && event->repropagated
                && event->pre_delivery_max_state_difference > state_tolerance
                && event->post_delivery_max_state_difference <= state_tolerance
                && event->post_delivery_max_covariance_difference <= covariance_tolerance
                && event->post_delivery_metadata_match;
        }
        if (event_count == 2U) pass = pass && sequential_non_overlapping;
        if (output != NULL && event_count == 1U) {
            const DelayedEvent *event = &events[0];
            fprintf(
                output,
                "{\n"
                "  \"schema_version\": 1,\n"
                "  \"status\": \"host_only_research_oracle_not_flight_feature\",\n"
                "  \"input_contract\": \"synthetic_exact_timestamp_pv_only\",\n"
                "  \"rate_hz\": %u,\n"
                "  \"delay_ms\": %u,\n"
                "  \"history_capacity_samples\": %u,\n"
                "  \"delay_samples\": %llu,\n"
                "  \"source_timestamp_us\": %llu,\n"
                "  \"delivery_timestamp_us\": %llu,\n"
                "  \"source_gps_accepted\": %s,\n"
                "  \"repropagated\": %s,\n"
                "  \"healthy\": %s,\n"
                "  \"covariance_psd\": %s,\n"
                "  \"pre_delivery_max_state_difference\": %.17g,\n"
                "  \"post_delivery_max_state_difference\": %.17g,\n"
                "  \"post_delivery_max_covariance_difference\": %.17g,\n"
                "  \"post_delivery_metadata_match\": %s,\n"
                "  \"state_tolerance\": %.17g,\n"
                "  \"covariance_tolerance\": %.17g,\n"
                "  \"pass\": %s,\n"
                "  \"limitations\": [\n"
                "    \"Only one isolated delayed GNSS P/V event is replayed.\",\n"
                "    \"History is a host validation ring; no production rewind API is added.\",\n"
                "    \"The experiment does not validate delayed heading, barometer, multi-IMU, or supervisor policy.\",\n"
                "    \"Truth is used only to generate the synthetic P/V observation and is not read by the replay logic.\"\n"
                "  ]\n"
                "}\n",
                rate_hz, delay_ms, ORACLE_HISTORY_CAPACITY,
                (unsigned long long)delay_samples,
                (unsigned long long)(event->source_sequence * dt_us),
                (unsigned long long)(event->delivery_sequence * dt_us),
                event->source_accepted ? "true" : "false",
                event->repropagated ? "true" : "false",
                healthy ? "true" : "false",
                covariance_psd ? "true" : "false",
                event->pre_delivery_max_state_difference,
                event->post_delivery_max_state_difference,
                event->post_delivery_max_covariance_difference,
                event->post_delivery_metadata_match ? "true" : "false",
                state_tolerance,
                covariance_tolerance,
                pass ? "true" : "false"
            );
        } else if (output != NULL) {
            fprintf(
                output,
                "{\n"
                "  \"schema_version\": 1,\n"
                "  \"status\": \"host_only_research_oracle_not_flight_feature\",\n"
                "  \"scenario\": \"%s\",\n"
                "  \"input_contract\": \"synthetic_exact_timestamp_pv_only\",\n"
                "  \"rate_hz\": %u,\n"
                "  \"delay_ms\": %u,\n"
                "  \"history_capacity_samples\": %u,\n"
                "  \"delay_samples\": %llu,\n"
                "  \"sequential_non_overlapping\": %s,\n"
                "  \"healthy\": %s,\n"
                "  \"covariance_psd\": %s,\n"
                "  \"state_tolerance\": %.17g,\n"
                "  \"covariance_tolerance\": %.17g,\n"
                "  \"event_count\": %u,\n"
                "  \"events\": [\n",
                scenario_name(scenario), rate_hz, delay_ms, ORACLE_HISTORY_CAPACITY,
                (unsigned long long)delay_samples,
                sequential_non_overlapping ? "true" : "false",
                healthy ? "true" : "false",
                covariance_psd ? "true" : "false",
                state_tolerance,
                covariance_tolerance,
                event_count
            );
            for (event_index = 0U; event_index < event_count; ++event_index) {
                write_event_json(
                    output, &events[event_index], dt_us, event_index + 1U,
                    event_index + 1U < event_count
                );
            }
            fprintf(
                output,
                "  ],\n"
                "  \"final_max_state_difference\": %.17g,\n"
                "  \"final_max_covariance_difference\": %.17g,\n"
                "  \"final_metadata_match\": %s,\n"
                "  \"pass\": %s,\n"
                "  \"limitations\": [\n"
                "    \"Exactly two sequential, non-overlapping delayed GNSS P/V events are replayed.\",\n"
                "    \"The second source timestamp is after the first delivery timestamp.\",\n"
                "    \"This is not a generic overlapping out-of-sequence measurement implementation.\",\n"
                "    \"History is a host validation ring; no production rewind API is added.\",\n"
                "    \"The experiment does not validate delayed heading, barometer, multi-IMU, or supervisor policy.\",\n"
                "    \"Truth is used only to generate the synthetic P/V observation and is not read by the replay logic.\"\n"
                "  ]\n"
                "}\n",
                final_state_difference,
                final_covariance_difference,
                final_metadata_match ? "true" : "false",
                pass ? "true" : "false"
            );
        }
        return pass;
    }
}

int main(int argc, char **argv)
{
    unsigned rate_hz = 100U;
    unsigned delay_ms = 50U;
    OracleScenario scenario = ORACLE_SCENARIO_SINGLE;
    const char *output_path = NULL;
    FILE *output = stdout;
    int argument;
    bool pass;
    for (argument = 1; argument < argc; ++argument) {
        if (strcmp(argv[argument], "--rate-hz") == 0 && argument + 1 < argc) {
            if (!parse_unsigned(argv[++argument], &rate_hz)) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[argument], "--delay-ms") == 0 && argument + 1 < argc) {
            if (!parse_unsigned(argv[++argument], &delay_ms)) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[argument], "--scenario") == 0 && argument + 1 < argc) {
            if (!parse_scenario(argv[++argument], &scenario)) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[argument], "--out") == 0 && argument + 1 < argc) {
            output_path = argv[++argument];
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (output_path != NULL) {
        output = fopen(output_path, "w");
        if (output == NULL) {
            perror(output_path);
            return 2;
        }
    }
    pass = scenario == ORACLE_SCENARIO_OVERLAP
        ? run_overlapping_case(rate_hz, delay_ms, output)
        : run_case(rate_hz, delay_ms, scenario, output);
    if (output_path != NULL) fclose(output);
    return pass ? 0 : 1;
}
