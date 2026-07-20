/**
 * @file barometer_supervisor.c
 * @brief Causal source-quality guard for relative barometric height aiding.
 */

#include <aerakia/barometer_supervisor.h>

#include <math.h>
#include <stddef.h>
#include <string.h>

static float maximumf(float first, float second)
{
    return first > second ? first : second;
}

static float elapsed_s(uint64_t current_us, uint64_t previous_us)
{
    return current_us >= previous_us
        ? (float)((double)(current_us - previous_us) * 1.0e-6)
        : INFINITY;
}

void aerakia_barometer_supervisor_default_config(
    AerakiaBarometerSupervisorConfig *config
)
{
    if (config == NULL) return;
    config->maximum_sample_age_s = 0.15f;
    config->jump_minimum_threshold_m = 0.50f;
    config->jump_sigma_multiplier = 3.0f;
    config->prediction_delta_variance_m2 = 0.04f;
    config->jump_latch_count = 2U;
    config->freeze_window_s = 0.75f;
    config->freeze_height_span_m = 0.05f;
    config->freeze_prediction_displacement_m = 0.30f;
    config->freeze_repeat_tolerance_m = 0.001f;
    config->freeze_minimum_vertical_speed_m_s = 0.10f;
    config->measurement_quantization_m = 0.0f;
    config->freeze_consecutive_samples = 3U;
    config->recovery_min_samples = 10U;
    config->recovery_min_duration_s = 0.50f;
    config->recovery_max_gap_s = 0.10f;
}

static void sanitize_config(AerakiaBarometerSupervisorConfig *config)
{
    AerakiaBarometerSupervisorConfig defaults;
    aerakia_barometer_supervisor_default_config(&defaults);
    if (!isfinite(config->maximum_sample_age_s) || config->maximum_sample_age_s < 0.0f) {
        config->maximum_sample_age_s = defaults.maximum_sample_age_s;
    }
    if (!isfinite(config->jump_minimum_threshold_m)
        || config->jump_minimum_threshold_m <= 0.0f) {
        config->jump_minimum_threshold_m = defaults.jump_minimum_threshold_m;
    }
    if (!isfinite(config->jump_sigma_multiplier) || config->jump_sigma_multiplier <= 0.0f) {
        config->jump_sigma_multiplier = defaults.jump_sigma_multiplier;
    }
    if (!isfinite(config->prediction_delta_variance_m2)
        || config->prediction_delta_variance_m2 < 0.0f) {
        config->prediction_delta_variance_m2 = defaults.prediction_delta_variance_m2;
    }
    if (config->jump_latch_count < 1U) config->jump_latch_count = defaults.jump_latch_count;
    if (!isfinite(config->freeze_window_s) || config->freeze_window_s <= 0.0f) {
        config->freeze_window_s = defaults.freeze_window_s;
    }
    if (!isfinite(config->freeze_height_span_m) || config->freeze_height_span_m <= 0.0f) {
        config->freeze_height_span_m = defaults.freeze_height_span_m;
    }
    if (!isfinite(config->freeze_prediction_displacement_m)
        || config->freeze_prediction_displacement_m <= 0.0f) {
        config->freeze_prediction_displacement_m = defaults.freeze_prediction_displacement_m;
    }
    if (!isfinite(config->freeze_repeat_tolerance_m)
        || config->freeze_repeat_tolerance_m < 0.0f) {
        config->freeze_repeat_tolerance_m = defaults.freeze_repeat_tolerance_m;
    }
    if (!isfinite(config->freeze_minimum_vertical_speed_m_s)
        || config->freeze_minimum_vertical_speed_m_s < 0.0f) {
        config->freeze_minimum_vertical_speed_m_s = defaults.freeze_minimum_vertical_speed_m_s;
    }
    if (!isfinite(config->measurement_quantization_m)
        || config->measurement_quantization_m < 0.0f) {
        config->measurement_quantization_m = defaults.measurement_quantization_m;
    }
    if (config->freeze_consecutive_samples < 1U) {
        config->freeze_consecutive_samples = defaults.freeze_consecutive_samples;
    }
    if (config->recovery_min_samples < 1U) {
        config->recovery_min_samples = defaults.recovery_min_samples;
    }
    if (!isfinite(config->recovery_min_duration_s) || config->recovery_min_duration_s < 0.0f) {
        config->recovery_min_duration_s = defaults.recovery_min_duration_s;
    }
    if (!isfinite(config->recovery_max_gap_s) || config->recovery_max_gap_s <= 0.0f) {
        config->recovery_max_gap_s = defaults.recovery_max_gap_s;
    }
}

void aerakia_barometer_supervisor_init(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorConfig *config
)
{
    AerakiaBarometerSupervisorConfig defaults;
    if (supervisor == NULL) return;
    aerakia_barometer_supervisor_default_config(&defaults);
    memset(supervisor, 0, sizeof(*supervisor));
    supervisor->config = config != NULL ? *config : defaults;
    sanitize_config(&supervisor->config);
}

static void reset_recovery(AerakiaBarometerSupervisor *supervisor)
{
    supervisor->recovery_sample_count = 0U;
    supervisor->recovery_start_timestamp_us = 0U;
    supervisor->recovery_last_timestamp_us = 0U;
}

static void accept_observation(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *observation
)
{
    supervisor->last_accepted_timestamp_us = observation->sample_timestamp_us;
    supervisor->last_accepted_height_up_m = observation->height_up_m;
    supervisor->last_accepted_predicted_height_up_m = observation->predicted_height_up_m;
    supervisor->last_accepted_variance_m2 = observation->variance_m2;
}

static void stage_observation(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *observation,
    bool initialization,
    bool recovery
)
{
    supervisor->pending_observation = *observation;
    supervisor->pending_observation_available = true;
    supervisor->pending_initialization = initialization;
    supervisor->pending_recovery = recovery;
}

static bool datum_recovery_matches(
    const AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *observation
)
{
    const AerakiaBarometerDatumRecoveryAuthorization *authorization =
        &supervisor->datum_recovery_authorization;
    return supervisor->datum_recovery_authorized
        && authorization->source_quality_verified
        && observation->sample_timestamp_us >= authorization->authorization_timestamp_us
        && observation->sample_timestamp_us <= authorization->valid_until_timestamp_us
        && observation->source_id == authorization->source_id
        && observation->source_generation == authorization->source_generation
        && observation->quality_sequence == authorization->quality_sequence;
}

bool aerakia_barometer_supervisor_authorize_datum_recovery(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerDatumRecoveryAuthorization *authorization
)
{
    if (supervisor == NULL || authorization == NULL
        || !authorization->source_quality_verified
        || !supervisor->fault_latched
        || (supervisor->latched_fault_flags & AERAKIA_BARO_FAULT_JUMP) == 0U
        || authorization->authorization_timestamp_us < supervisor->last_seen_timestamp_us
        || authorization->valid_until_timestamp_us < authorization->authorization_timestamp_us) {
        return false;
    }
    supervisor->datum_recovery_authorization = *authorization;
    supervisor->datum_recovery_authorized = true;
    return true;
}

void aerakia_barometer_supervisor_commit(
    AerakiaBarometerSupervisor *supervisor,
    bool core_accepted,
    AerakiaBarometerSupervisorDecision *decision
)
{
    if (supervisor == NULL || !supervisor->pending_observation_available) return;
    if (core_accepted) {
        accept_observation(supervisor, &supervisor->pending_observation);
        if (supervisor->pending_initialization) {
            supervisor->initialized = true;
            supervisor->freeze_anchor_timestamp_us =
                supervisor->pending_observation.sample_timestamp_us;
            supervisor->freeze_anchor_height_up_m = supervisor->pending_observation.height_up_m;
            supervisor->freeze_anchor_predicted_height_up_m =
                supervisor->pending_observation.predicted_height_up_m;
            supervisor->last_seen_height_up_m = supervisor->pending_observation.height_up_m;
        }
        if (supervisor->pending_recovery) {
            supervisor->fault_latched = false;
            supervisor->latched_fault_flags = AERAKIA_BARO_FAULT_NONE;
            supervisor->consecutive_jump_count = 0U;
            reset_recovery(supervisor);
            supervisor->datum_recovery_authorized = false;
        }
    } else if (supervisor->pending_recovery) {
        reset_recovery(supervisor);
    }
    if (decision != NULL) {
        decision->accepted = core_accepted;
        decision->fault_latched = supervisor->fault_latched;
        decision->recovery_probationary = supervisor->fault_latched
            && supervisor->recovery_sample_count > 0U;
        decision->recovery_sample_count = supervisor->recovery_sample_count;
        if (supervisor->fault_latched) {
            decision->fault_flags = supervisor->latched_fault_flags
                | AERAKIA_BARO_FAULT_LATCHED;
        } else if (core_accepted && supervisor->pending_recovery) {
            decision->fault_flags = AERAKIA_BARO_FAULT_NONE;
        }
    }
    supervisor->pending_observation_available = false;
    supervisor->pending_initialization = false;
    supervisor->pending_recovery = false;
}

AerakiaBarometerSupervisorDecision aerakia_barometer_supervisor_evaluate(
    AerakiaBarometerSupervisor *supervisor,
    const AerakiaBarometerSupervisorObservation *observation
)
{
    AerakiaBarometerSupervisorDecision decision;
    float residual;
    float threshold;
    float freeze_duration;
    bool jump;
    bool freeze;
    memset(&decision, 0, sizeof(decision));
    decision.sample_age_s = INFINITY;
    decision.increment_residual_m = NAN;
    decision.jump_threshold_m = NAN;
    if (supervisor != NULL && supervisor->fault_latched) {
        decision.fault_latched = true;
        decision.recovery_probationary = supervisor->recovery_sample_count > 0U;
        decision.fault_flags = supervisor->latched_fault_flags | AERAKIA_BARO_FAULT_LATCHED;
    }

    if (supervisor != NULL && supervisor->pending_observation_available) {
        decision.fault_flags |= AERAKIA_BARO_FAULT_COMMIT_PENDING;
        return decision;
    }

    if (supervisor == NULL || observation == NULL
        || !isfinite(observation->height_up_m)
        || !isfinite(observation->variance_m2) || observation->variance_m2 <= 0.0f
        || !isfinite(observation->predicted_height_up_m)
        || !isfinite(observation->predicted_vertical_velocity_up_m_s)) {
        decision.fault_flags |= AERAKIA_BARO_FAULT_INVALID;
        return decision;
    }
    if (observation->sample_timestamp_us > observation->evaluation_timestamp_us
        || (supervisor->has_seen_timestamp
            && observation->sample_timestamp_us <= supervisor->last_seen_timestamp_us)) {
        decision.fault_flags |= AERAKIA_BARO_FAULT_TIMESTAMP;
        return decision;
    }
    supervisor->last_seen_timestamp_us = observation->sample_timestamp_us;
    supervisor->has_seen_timestamp = true;
    decision.sample_age_s = elapsed_s(
        observation->evaluation_timestamp_us, observation->sample_timestamp_us
    );
    if (decision.sample_age_s > supervisor->config.maximum_sample_age_s) {
        decision.fault_flags |= AERAKIA_BARO_FAULT_STALE;
        return decision;
    }

    if (!supervisor->initialized) {
        stage_observation(supervisor, observation, true, false);
        decision.accepted = true;
        return decision;
    }

    residual = (observation->height_up_m - supervisor->last_accepted_height_up_m)
        - (observation->predicted_height_up_m
            - supervisor->last_accepted_predicted_height_up_m);
    threshold = maximumf(
        supervisor->config.jump_minimum_threshold_m,
        supervisor->config.jump_sigma_multiplier * sqrtf(
            observation->variance_m2 + supervisor->last_accepted_variance_m2
                + supervisor->config.prediction_delta_variance_m2
        )
    );
    decision.increment_residual_m = residual;
    decision.jump_threshold_m = threshold;
    jump = fabsf(residual) > threshold;

    if (supervisor->config.measurement_quantization_m
            <= supervisor->config.freeze_repeat_tolerance_m
        && fabsf(observation->height_up_m - supervisor->last_seen_height_up_m)
            <= supervisor->config.freeze_repeat_tolerance_m
        && fabsf(observation->predicted_vertical_velocity_up_m_s)
            >= supervisor->config.freeze_minimum_vertical_speed_m_s) {
        supervisor->consecutive_freeze_sample_count++;
    } else {
        supervisor->consecutive_freeze_sample_count = 0U;
    }
    supervisor->last_seen_height_up_m = observation->height_up_m;

    if (fabsf(observation->height_up_m - supervisor->freeze_anchor_height_up_m)
        > maximumf(supervisor->config.freeze_height_span_m,
            0.75f * supervisor->config.measurement_quantization_m)) {
        supervisor->freeze_anchor_timestamp_us = observation->sample_timestamp_us;
        supervisor->freeze_anchor_height_up_m = observation->height_up_m;
        supervisor->freeze_anchor_predicted_height_up_m = observation->predicted_height_up_m;
    }
    freeze_duration = elapsed_s(
        observation->sample_timestamp_us, supervisor->freeze_anchor_timestamp_us
    );
    decision.freeze_duration_s = freeze_duration;
    freeze = supervisor->consecutive_freeze_sample_count
            >= supervisor->config.freeze_consecutive_samples
        || (freeze_duration >= supervisor->config.freeze_window_s
            && fabsf(observation->predicted_height_up_m
                - supervisor->freeze_anchor_predicted_height_up_m)
                >= maximumf(supervisor->config.freeze_prediction_displacement_m,
                    1.25f * supervisor->config.measurement_quantization_m));

    if (jump) {
        supervisor->consecutive_jump_count++;
    } else {
        supervisor->consecutive_jump_count = 0U;
    }
    if (freeze) {
        supervisor->fault_latched = true;
        supervisor->latched_fault_flags |= AERAKIA_BARO_FAULT_FREEZE;
        reset_recovery(supervisor);
    }
    if (jump && supervisor->consecutive_jump_count >= supervisor->config.jump_latch_count) {
        if ((supervisor->latched_fault_flags & AERAKIA_BARO_FAULT_JUMP) == 0U) {
            supervisor->datum_recovery_authorized = false;
        }
        supervisor->fault_latched = true;
        supervisor->latched_fault_flags |= AERAKIA_BARO_FAULT_JUMP;
        reset_recovery(supervisor);
    }

    if (supervisor->fault_latched) {
        const bool jump_latched =
            (supervisor->latched_fault_flags & AERAKIA_BARO_FAULT_JUMP) != 0U;
        const bool recovery_authorized = !jump_latched
            || datum_recovery_matches(supervisor, observation);
        bool recovery_candidate = !jump && !freeze && recovery_authorized;
        decision.fault_flags = supervisor->latched_fault_flags | AERAKIA_BARO_FAULT_LATCHED;
        if (jump_latched && !recovery_authorized) {
            decision.fault_flags |= AERAKIA_BARO_FAULT_RECOVERY_AUTH_REQUIRED;
        }
        if (recovery_candidate) {
            if (supervisor->recovery_last_timestamp_us != 0U
                && elapsed_s(observation->sample_timestamp_us,
                    supervisor->recovery_last_timestamp_us)
                    > supervisor->config.recovery_max_gap_s) {
                reset_recovery(supervisor);
            }
            if (supervisor->recovery_start_timestamp_us == 0U) {
                supervisor->recovery_start_timestamp_us = observation->sample_timestamp_us;
            }
            supervisor->recovery_last_timestamp_us = observation->sample_timestamp_us;
            supervisor->recovery_sample_count++;
            if (supervisor->recovery_sample_count >= supervisor->config.recovery_min_samples
                && elapsed_s(observation->sample_timestamp_us,
                    supervisor->recovery_start_timestamp_us)
                    >= supervisor->config.recovery_min_duration_s) {
                stage_observation(supervisor, observation, false, true);
                decision.accepted = true;
            } else {
                decision.fault_flags |= AERAKIA_BARO_FAULT_PROBATION;
            }
        } else {
            reset_recovery(supervisor);
        }
    } else if (jump || freeze) {
        decision.fault_flags = (jump ? AERAKIA_BARO_FAULT_JUMP : 0U)
            | (freeze ? AERAKIA_BARO_FAULT_FREEZE : 0U);
    } else {
        stage_observation(supervisor, observation, false, false);
        decision.accepted = true;
    }

    decision.fault_latched = supervisor->fault_latched;
    decision.recovery_probationary = supervisor->fault_latched
        && supervisor->recovery_sample_count > 0U;
    decision.consecutive_jump_count = supervisor->consecutive_jump_count;
    decision.consecutive_freeze_sample_count = supervisor->consecutive_freeze_sample_count;
    decision.recovery_sample_count = supervisor->recovery_sample_count;
    return decision;
}
