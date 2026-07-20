/**
 * @file barometer_supervisor.h
 * @brief Causal source-quality guard for relative barometric height aiding.
 */

#ifndef AERAKIA_BAROMETER_SUPERVISOR_H
#define AERAKIA_BAROMETER_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AERAKIA_BARO_FAULT_NONE = 0U,
    AERAKIA_BARO_FAULT_INVALID = 1U << 0,
    AERAKIA_BARO_FAULT_TIMESTAMP = 1U << 1,
    AERAKIA_BARO_FAULT_STALE = 1U << 2,
    AERAKIA_BARO_FAULT_JUMP = 1U << 3,
    AERAKIA_BARO_FAULT_FREEZE = 1U << 4,
    AERAKIA_BARO_FAULT_LATCHED = 1U << 5,
    AERAKIA_BARO_FAULT_PROBATION = 1U << 6,
    AERAKIA_BARO_FAULT_RECOVERY_AUTH_REQUIRED = 1U << 7,
    AERAKIA_BARO_FAULT_COMMIT_PENDING = 1U << 8
} AerakiaBarometerFault;

typedef struct {
    /** Independent source-age gate; the ESKF adapter retains its own final timestamp gate. */
    float maximum_sample_age_s;
    float jump_minimum_threshold_m;
    float jump_sigma_multiplier;
    float prediction_delta_variance_m2;
    uint32_t jump_latch_count;
    float freeze_window_s;
    float freeze_height_span_m;
    float freeze_prediction_displacement_m;
    float freeze_repeat_tolerance_m;
    float freeze_minimum_vertical_speed_m_s;
    float measurement_quantization_m;
    uint32_t freeze_consecutive_samples;
    uint32_t recovery_min_samples;
    float recovery_min_duration_s;
    float recovery_max_gap_s;
} AerakiaBarometerSupervisorConfig;

typedef struct {
    uint64_t sample_timestamp_us;
    uint64_t evaluation_timestamp_us;
    float height_up_m;
    float variance_m2;
    /** ESKF height immediately before this barometer update, also positive-up. */
    float predicted_height_up_m;
    float predicted_vertical_velocity_up_m_s;
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
} AerakiaBarometerSupervisorObservation;

typedef struct {
    uint64_t authorization_timestamp_us;
    uint64_t valid_until_timestamp_us;
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
    bool source_quality_verified;
} AerakiaBarometerDatumRecoveryAuthorization;

typedef struct {
    bool accepted;
    bool fault_latched;
    bool recovery_probationary;
    uint32_t fault_flags;
    uint32_t consecutive_jump_count;
    uint32_t consecutive_freeze_sample_count;
    uint32_t recovery_sample_count;
    float sample_age_s;
    float increment_residual_m;
    float jump_threshold_m;
    float freeze_duration_s;
} AerakiaBarometerSupervisorDecision;

typedef struct {
    AerakiaBarometerSupervisorConfig config;
    uint64_t last_seen_timestamp_us;
    uint64_t last_accepted_timestamp_us;
    uint64_t freeze_anchor_timestamp_us;
    uint64_t recovery_start_timestamp_us;
    uint64_t recovery_last_timestamp_us;
    float last_accepted_height_up_m;
    float last_accepted_predicted_height_up_m;
    float last_accepted_variance_m2;
    float freeze_anchor_height_up_m;
    float freeze_anchor_predicted_height_up_m;
    float last_seen_height_up_m;
    uint32_t consecutive_jump_count;
    uint32_t consecutive_freeze_sample_count;
    uint32_t recovery_sample_count;
    uint32_t latched_fault_flags;
    AerakiaBarometerSupervisorObservation pending_observation;
    AerakiaBarometerDatumRecoveryAuthorization datum_recovery_authorization;
    bool has_seen_timestamp;
    bool initialized;
    bool fault_latched;
    bool pending_observation_available;
    bool pending_initialization;
    bool pending_recovery;
    bool datum_recovery_authorized;
} AerakiaBarometerSupervisor;

void aerakia_barometer_supervisor_default_config(
    AerakiaBarometerSupervisorConfig *config
);

void aerakia_barometer_supervisor_init(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorConfig *config
);

AerakiaBarometerSupervisorDecision aerakia_barometer_supervisor_evaluate(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *observation
);

void aerakia_barometer_supervisor_commit(
    AerakiaBarometerSupervisor *supervisor,
    bool core_accepted,
    AerakiaBarometerSupervisorDecision *decision
);

bool aerakia_barometer_supervisor_authorize_datum_recovery(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerDatumRecoveryAuthorization *authorization
);

#endif
