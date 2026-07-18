/**
 * @file eskf_adapter.c
 * @brief Hardware-neutral timestamp, unit, and measurement adapter for ESKF.
 */

#include <aerakia/eskf_adapter.h>

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

static bool vector_is_finite(AerakiaVec3f vector)
{
    return isfinite(vector.x) && isfinite(vector.y) && isfinite(vector.z);
}

static bool state_is_finite_and_normalized(const ESKF_NominalState *state)
{
    int axis;
    eskf_float_t quaternion_norm_squared = 0.0;
    if (state == NULL) return false;
    for (axis = 0; axis < 3; ++axis) {
        if (!isfinite(state->p[axis]) || !isfinite(state->v[axis])
            || !isfinite(state->ab[axis]) || !isfinite(state->gb[axis])) {
            return false;
        }
    }
    for (axis = 0; axis < 4; ++axis) {
        if (!isfinite(state->q[axis])) return false;
        quaternion_norm_squared += state->q[axis] * state->q[axis];
    }
    return isfinite(quaternion_norm_squared)
        && fabs(quaternion_norm_squared - 1.0) <= 1.0e-3;
}

static float vector_norm(AerakiaVec3f vector)
{
    return sqrtf(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

static AerakiaStatus validate_aiding_timestamp(
    const AerakiaEskf *filter,
    uint64_t timestamp_us,
    uint64_t last_source_timestamp_us,
    bool has_source_timestamp
)
{
    double age_s;
    if (filter == NULL || !filter->has_timestamp) return AERAKIA_STATUS_TIMESTAMP_ERROR;
    if (timestamp_us > filter->last_timestamp_us) return AERAKIA_STATUS_TIMESTAMP_ERROR;
    if (has_source_timestamp && timestamp_us <= last_source_timestamp_us) {
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }
    age_s = (double)(filter->last_timestamp_us - timestamp_us) * 1.0e-6;
    if (filter->config.maximum_aiding_age_s >= 0.0f
        && age_s > filter->config.maximum_aiding_age_s) {
        return AERAKIA_STATUS_STALE_MEASUREMENT;
    }
    return AERAKIA_STATUS_OK;
}

static AerakiaVec3f quaternion_to_euler(const eskf_float_t q[4])
{
    AerakiaVec3f result;
    const float w = (float)q[0];
    const float x = (float)q[1];
    const float y = (float)q[2];
    const float z = (float)q[3];
    const float sin_pitch = clampf(2.0f * (w * y - z * x), -1.0f, 1.0f);

    result.x = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    result.y = asinf(sin_pitch);
    result.z = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return result;
}

static AerakiaVec3f gravity_body_from_quaternion(const eskf_float_t q[4])
{
    const float w = (float)q[0];
    const float x = (float)q[1];
    const float y = (float)q[2];
    const float z = (float)q[3];
    AerakiaVec3f gravity = {
        -2.0f * (x * z - w * y),
        -2.0f * (y * z + w * x),
        -(1.0f - 2.0f * (x * x + y * y)),
    };
    return gravity;
}

static void reset_static_collection(AerakiaEskf *filter)
{
    memset(filter->static_acceleration_sum, 0, sizeof(filter->static_acceleration_sum));
    memset(filter->static_angular_rate_sum, 0, sizeof(filter->static_angular_rate_sum));
    memset(filter->static_magnetic_sum, 0, sizeof(filter->static_magnetic_sum));
    filter->static_alignment_start_timestamp_us = 0U;
    filter->static_alignment_samples = 0U;
    filter->static_magnetic_samples = 0U;
}

static bool sample_is_stationary(const AerakiaEskf *filter, const AerakiaImuSample *sample)
{
    if ((sample->flags & AERAKIA_SAMPLE_STATIONARY) == 0U) {
        return false;
    }
    return vector_norm(sample->angular_rate_rad_s)
            <= filter->config.stationary_gyro_threshold_rad_s
        && fabsf(vector_norm(sample->acceleration_m_s2) - AERAKIA_GRAVITY_M_S2)
            <= filter->config.stationary_acceleration_tolerance_m_s2;
}

static bool static_alignment_ready(const AerakiaEskf *filter, uint64_t timestamp_us)
{
    const double duration_s = (double)(timestamp_us - filter->static_alignment_start_timestamp_us)
        * 1.0e-6;
    return filter->static_alignment_samples >= filter->config.static_alignment_min_samples
        && duration_s >= filter->config.static_alignment_duration_s;
}

static void collect_static_sample(AerakiaEskf *filter, const AerakiaImuSample *sample)
{
    int axis;
    bool magnetic_accepted = false;
    if (filter->static_alignment_samples == 0U) {
        filter->static_alignment_start_timestamp_us = sample->timestamp_us;
    }
    filter->static_acceleration_sum[0] += sample->acceleration_m_s2.x;
    filter->static_acceleration_sum[1] += sample->acceleration_m_s2.y;
    filter->static_acceleration_sum[2] += sample->acceleration_m_s2.z;
    filter->static_angular_rate_sum[0] += sample->angular_rate_rad_s.x;
    filter->static_angular_rate_sum[1] += sample->angular_rate_rad_s.y;
    filter->static_angular_rate_sum[2] += sample->angular_rate_rad_s.z;
    filter->static_alignment_samples++;

    if (filter->config.fuse_magnetometer
        && (sample->flags & AERAKIA_SAMPLE_MAG_VALID) != 0U
        && vector_is_finite(sample->magnetic_field_ut)) {
        magnetic_accepted = !filter->config.gate_magnetometer
            || aerakia_mag_gate_accept(
                &filter->magnetic_gate,
                sample->magnetic_field_ut.x,
                sample->magnetic_field_ut.y,
                sample->magnetic_field_ut.z
            );
        if (magnetic_accepted) {
            filter->static_magnetic_sum[0] += sample->magnetic_field_ut.x;
            filter->static_magnetic_sum[1] += sample->magnetic_field_ut.y;
            filter->static_magnetic_sum[2] += sample->magnetic_field_ut.z;
            filter->static_magnetic_samples++;
        }
    }

    if (static_alignment_ready(filter, sample->timestamp_us)) {
        eskf_float_t acceleration_mean[3];
        eskf_float_t angular_rate_mean[3];
        const double inverse_count = 1.0 / (double)filter->static_alignment_samples;
        for (axis = 0; axis < 3; ++axis) {
            acceleration_mean[axis] = filter->static_acceleration_sum[axis] * inverse_count;
            angular_rate_mean[axis] = filter->static_angular_rate_sum[axis] * inverse_count;
        }
        if (filter->config.static_align_attitude && !filter->attitude_seeded) {
            eskf_float_t attitude_variance[3] = {
                filter->core.P[0][0], filter->core.P[1][1], filter->core.P[2][2]
            };
            filter->static_tilt_alignment_complete =
                eskf_align_static_tilt(&filter->core, acceleration_mean);
            if (filter->static_tilt_alignment_complete) {
                const eskf_float_t tilt_variance =
                    filter->config.static_tilt_uncertainty_rad
                    * filter->config.static_tilt_uncertainty_rad;
                attitude_variance[0] = tilt_variance;
                attitude_variance[1] = tilt_variance;
            }
            if (filter->static_tilt_alignment_complete
                && filter->static_magnetic_samples > 0U) {
                eskf_float_t magnetic_mean[3];
                const double inverse_magnetic_count =
                    1.0 / (double)filter->static_magnetic_samples;
                for (axis = 0; axis < 3; ++axis) {
                    magnetic_mean[axis] =
                        filter->static_magnetic_sum[axis] * inverse_magnetic_count;
                }
                filter->static_heading_alignment_complete =
                    eskf_align_static_heading(&filter->core, magnetic_mean);
                if (filter->static_heading_alignment_complete) {
                    attitude_variance[2] = filter->config.static_heading_uncertainty_rad
                        * filter->config.static_heading_uncertainty_rad;
                }
            }
            if (filter->static_tilt_alignment_complete) {
                (void)eskf_reset_attitude_covariance(&filter->core, attitude_variance);
            }
        }
        eskf_align_static_bias_means(&filter->core, acceleration_mean, angular_rate_mean);
        filter->static_alignment_complete = true;
        filter->last_zero_velocity_timestamp_us = sample->timestamp_us;
    }
}

void aerakia_eskf_default_config(AerakiaEskfConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->minimum_dt_s = 0.0001f;
    config->maximum_dt_s = 0.1f;
    config->maximum_aiding_age_s = 0.5f;
    config->fuse_magnetometer = false;
    config->gate_magnetometer = true;
    config->magnetometer_variance = 0.05f;
    config->magnetic_reference_ned[0] = 1.0f;
    config->magnetic_reference_ned[1] = 0.0f;
    config->magnetic_reference_ned[2] = 0.0f;
    config->navigation_recovery_rejection_limit = 10U;
    config->recovery_position_variance_floor_m2 = 25.0f;
    config->recovery_velocity_variance_floor_m2_s2 = 4.0f;
    config->enable_static_alignment = true;
    config->static_align_attitude = true;
    config->static_alignment_duration_s = 1.0f;
    config->static_alignment_min_samples = 100U;
    /* Conservative floors include residual calibration and mounting error. */
    config->static_tilt_uncertainty_rad = 2.0f * AERAKIA_PI_F / 180.0f;
    config->static_heading_uncertainty_rad = 10.0f * AERAKIA_PI_F / 180.0f;
    config->stationary_gyro_threshold_rad_s = 0.05f;
    config->stationary_acceleration_tolerance_m_s2 = 0.20f * AERAKIA_GRAVITY_M_S2;
    config->zero_velocity_interval_s = 0.10f;
    config->zero_velocity_variance_m2_s2 = 0.01f;
    aerakia_mag_gate_default_config(&config->magnetic_gate);
}

void aerakia_eskf_init(
    AerakiaEskf *filter,
    const AerakiaEskfConfig *config,
    const double initial_position_ned_m[3],
    const double initial_quaternion_wxyz[4]
)
{
    AerakiaEskfConfig defaults;
    eskf_float_t reference[3];

    if (filter == NULL) {
        return;
    }
    aerakia_eskf_default_config(&defaults);
    memset(filter, 0, sizeof(*filter));
    filter->config = config != NULL ? *config : defaults;
    filter->attitude_seeded = initial_quaternion_wxyz != NULL;
    eskf_init(&filter->core, initial_position_ned_m, initial_quaternion_wxyz);
    aerakia_mag_gate_init(&filter->magnetic_gate, &filter->config.magnetic_gate);
    reference[0] = filter->config.magnetic_reference_ned[0];
    reference[1] = filter->config.magnetic_reference_ned[1];
    reference[2] = filter->config.magnetic_reference_ned[2];
    eskf_set_mag_reference(&filter->core, reference);
}

AerakiaStatus aerakia_eskf_process_imu(
    AerakiaEskf *filter,
    const AerakiaImuSample *sample,
    AerakiaNavigationEstimate *estimate
)
{
    eskf_float_t acceleration[3];
    eskf_float_t angular_rate[3];
    float dt;

    if (filter == NULL || sample == NULL) {
        return AERAKIA_STATUS_INVALID_ARGUMENT;
    }
    if ((sample->flags & (AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID))
        != (AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID)
        || !vector_is_finite(sample->acceleration_m_s2)
        || !vector_is_finite(sample->angular_rate_rad_s)) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_MISSING_MEASUREMENT;
    }
    if (filter->has_timestamp && sample->timestamp_us <= filter->last_timestamp_us) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }

    filter->stationary_detected = sample_is_stationary(filter, sample);
    filter->zero_velocity_update_applied = false;
    if (filter->config.enable_static_alignment && !filter->static_alignment_complete
        && (sample->flags & AERAKIA_SAMPLE_STATIONARY) != 0U) {
        if (filter->stationary_detected) {
            collect_static_sample(filter, sample);
            filter->last_timestamp_us = sample->timestamp_us;
            filter->has_timestamp = true;
            aerakia_eskf_get_estimate(filter, estimate);
            return filter->static_alignment_complete
                ? AERAKIA_STATUS_INITIALIZED
                : AERAKIA_STATUS_ALIGNING;
        }
        reset_static_collection(filter);
    } else if (!filter->static_alignment_complete && !filter->stationary_detected) {
        reset_static_collection(filter);
    }

    if (!filter->has_timestamp) {
        filter->last_timestamp_us = sample->timestamp_us;
        filter->has_timestamp = true;
        aerakia_eskf_get_estimate(filter, estimate);
        return AERAKIA_STATUS_INITIALIZED;
    }
    dt = (float)(sample->timestamp_us - filter->last_timestamp_us) * 1.0e-6f;
    if (dt < filter->config.minimum_dt_s) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }
    if (dt > filter->config.maximum_dt_s) {
        /* Re-anchor after a forward transport gap; never integrate across it. */
        filter->last_timestamp_us = sample->timestamp_us;
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }
    filter->last_timestamp_us = sample->timestamp_us;

    acceleration[0] = sample->acceleration_m_s2.x;
    acceleration[1] = sample->acceleration_m_s2.y;
    acceleration[2] = sample->acceleration_m_s2.z;
    angular_rate[0] = sample->angular_rate_rad_s.x;
    angular_rate[1] = sample->angular_rate_rad_s.y;
    angular_rate[2] = sample->angular_rate_rad_s.z;
    eskf_predict(&filter->core, acceleration, angular_rate, (eskf_float_t)dt);

    if (filter->static_alignment_complete && filter->stationary_detected
        && filter->config.zero_velocity_interval_s > 0.0f
        && sample->timestamp_us - filter->last_zero_velocity_timestamp_us
            >= (uint64_t)(filter->config.zero_velocity_interval_s * 1.0e6f)) {
        eskf_update_static_constraint(&filter->core, filter->config.zero_velocity_variance_m2_s2);
        filter->last_zero_velocity_timestamp_us = sample->timestamp_us;
        filter->zero_velocity_update_applied = true;
        filter->zero_velocity_update_count++;
    }

    filter->magnetometer_accepted = false;
    memset(&filter->last_magnetometer_innovation, 0, sizeof(filter->last_magnetometer_innovation));
    if (filter->config.fuse_magnetometer
        && (sample->flags & AERAKIA_SAMPLE_MAG_VALID) != 0U
        && vector_is_finite(sample->magnetic_field_ut)) {
        bool use_magnetometer = true;
        eskf_float_t magnetic[3];
        if (filter->config.gate_magnetometer) {
            use_magnetometer = aerakia_mag_gate_accept(
                &filter->magnetic_gate,
                sample->magnetic_field_ut.x,
                sample->magnetic_field_ut.y,
                sample->magnetic_field_ut.z
            );
        }
        if (use_magnetometer) {
            magnetic[0] = sample->magnetic_field_ut.x;
            magnetic[1] = sample->magnetic_field_ut.y;
            magnetic[2] = sample->magnetic_field_ut.z;
            eskf_update_mag(
                &filter->core,
                magnetic,
                filter->config.magnetometer_variance,
                &filter->last_magnetometer_innovation
            );
            filter->magnetometer_accepted = filter->last_magnetometer_innovation.accepted;
        }
    }

    aerakia_eskf_get_estimate(filter, estimate);
    return estimate == NULL || estimate->healthy
        ? AERAKIA_STATUS_OK
        : AERAKIA_STATUS_NUMERICAL_ERROR;
}

void aerakia_eskf_update_position(
    AerakiaEskf *filter,
    AerakiaVec3f position_ned_m,
    float variance_m2
)
{
    eskf_float_t position[3];
    if (filter == NULL || variance_m2 <= 0.0f || !vector_is_finite(position_ned_m)) {
        return;
    }
    position[0] = position_ned_m.x;
    position[1] = position_ned_m.y;
    position[2] = position_ned_m.z;
    memset(&filter->last_position_innovation, 0, sizeof(filter->last_position_innovation));
    eskf_update_position(&filter->core, position, variance_m2, &filter->last_position_innovation);
    filter->position_accepted = filter->last_position_innovation.accepted;
}

void aerakia_eskf_update_velocity(
    AerakiaEskf *filter,
    AerakiaVec3f velocity_ned_m_s,
    float variance_m2_s2
)
{
    eskf_float_t velocity[3];
    if (filter == NULL || variance_m2_s2 <= 0.0f || !vector_is_finite(velocity_ned_m_s)) {
        return;
    }
    velocity[0] = velocity_ned_m_s.x;
    velocity[1] = velocity_ned_m_s.y;
    velocity[2] = velocity_ned_m_s.z;
    memset(&filter->last_velocity_innovation, 0, sizeof(filter->last_velocity_innovation));
    eskf_update_velocity(&filter->core, velocity, variance_m2_s2, &filter->last_velocity_innovation);
    filter->velocity_accepted = filter->last_velocity_innovation.accepted;
}

void aerakia_eskf_update_gps(
    AerakiaEskf *filter,
    AerakiaVec3f position_ned_m,
    AerakiaVec3f velocity_ned_m_s,
    float position_variance_m2,
    float velocity_variance_m2_s2
)
{
    eskf_float_t position[3];
    eskf_float_t velocity[3];
    if (filter == NULL || !vector_is_finite(position_ned_m)
        || !vector_is_finite(velocity_ned_m_s) || position_variance_m2 <= 0.0f
        || velocity_variance_m2_s2 <= 0.0f) return;

    position[0] = position_ned_m.x;
    position[1] = position_ned_m.y;
    position[2] = position_ned_m.z;
    velocity[0] = velocity_ned_m_s.x;
    velocity[1] = velocity_ned_m_s.y;
    velocity[2] = velocity_ned_m_s.z;
    filter->navigation_recovered = false;
    memset(&filter->last_position_innovation, 0, sizeof(filter->last_position_innovation));
    memset(&filter->last_velocity_innovation, 0, sizeof(filter->last_velocity_innovation));
    eskf_update_position(
        &filter->core, position, position_variance_m2, &filter->last_position_innovation
    );
    eskf_update_velocity(
        &filter->core, velocity, velocity_variance_m2_s2, &filter->last_velocity_innovation
    );
    filter->position_accepted = filter->last_position_innovation.accepted;
    filter->velocity_accepted = filter->last_velocity_innovation.accepted;

    if (filter->position_accepted || filter->velocity_accepted) {
        filter->consecutive_navigation_rejections = 0U;
    } else {
        filter->consecutive_navigation_rejections++;
        if (filter->config.navigation_recovery_rejection_limit > 0U
            && filter->consecutive_navigation_rejections
                >= filter->config.navigation_recovery_rejection_limit) {
            eskf_reset_navigation(
                &filter->core,
                position,
                velocity,
                fmaxf(position_variance_m2, filter->config.recovery_position_variance_floor_m2),
                fmaxf(
                    velocity_variance_m2_s2,
                    filter->config.recovery_velocity_variance_floor_m2_s2
                )
            );
            filter->navigation_recovered = true;
            filter->navigation_recovery_count++;
            filter->consecutive_navigation_rejections = 0U;
        }
    }
}

AerakiaStatus aerakia_eskf_update_gps_observation(
    AerakiaEskf *filter,
    const AerakiaGpsObservation *observation
)
{
    AerakiaStatus status;
    if (filter == NULL || observation == NULL) return AERAKIA_STATUS_INVALID_ARGUMENT;
    if (!vector_is_finite(observation->position_ned_m)
        || !vector_is_finite(observation->velocity_ned_m_s)
        || !isfinite(observation->position_variance_m2)
        || !isfinite(observation->velocity_variance_m2_s2)
        || observation->position_variance_m2 <= 0.0f
        || observation->velocity_variance_m2_s2 <= 0.0f) {
        return AERAKIA_STATUS_MISSING_MEASUREMENT;
    }
    status = validate_aiding_timestamp(
        filter, observation->timestamp_us, filter->last_gps_timestamp_us,
        filter->has_gps_timestamp
    );
    if (status != AERAKIA_STATUS_OK) return status;
    aerakia_eskf_update_gps(
        filter, observation->position_ned_m, observation->velocity_ned_m_s,
        observation->position_variance_m2, observation->velocity_variance_m2_s2
    );
    filter->last_gps_timestamp_us = observation->timestamp_us;
    filter->has_gps_timestamp = true;
    return AERAKIA_STATUS_OK;
}

void aerakia_eskf_update_heading(
    AerakiaEskf *filter,
    float heading_ned_rad,
    float variance_rad2
)
{
    if (filter == NULL || !isfinite(heading_ned_rad) || variance_rad2 <= 0.0f) return;
    memset(&filter->last_heading_innovation, 0, sizeof(filter->last_heading_innovation));
    eskf_update_heading(
        &filter->core,
        heading_ned_rad,
        variance_rad2,
        &filter->last_heading_innovation
    );
    filter->heading_accepted = filter->last_heading_innovation.accepted;
    if (filter->heading_accepted && filter->static_alignment_complete
        && filter->static_tilt_alignment_complete) {
        filter->static_heading_alignment_complete = true;
    }
}

AerakiaStatus aerakia_eskf_update_heading_observation(
    AerakiaEskf *filter,
    const AerakiaHeadingObservation *observation
)
{
    AerakiaStatus status;
    if (filter == NULL || observation == NULL) return AERAKIA_STATUS_INVALID_ARGUMENT;
    if (!isfinite(observation->heading_ned_rad) || !isfinite(observation->variance_rad2)
        || observation->variance_rad2 <= 0.0f) {
        return AERAKIA_STATUS_MISSING_MEASUREMENT;
    }
    status = validate_aiding_timestamp(
        filter, observation->timestamp_us, filter->last_heading_timestamp_us,
        filter->has_heading_timestamp
    );
    if (status != AERAKIA_STATUS_OK) return status;
    aerakia_eskf_update_heading(
        filter, observation->heading_ned_rad, observation->variance_rad2
    );
    filter->last_heading_timestamp_us = observation->timestamp_us;
    filter->has_heading_timestamp = true;
    return AERAKIA_STATUS_OK;
}

void aerakia_eskf_update_barometer(
    AerakiaEskf *filter,
    float height_up_m,
    float variance_m2
)
{
    if (filter != NULL && isfinite(height_up_m) && variance_m2 > 0.0f) {
        eskf_update_baro(&filter->core, height_up_m, variance_m2, NULL);
    }
}

AerakiaStatus aerakia_eskf_update_barometer_observation(
    AerakiaEskf *filter,
    const AerakiaBarometerObservation *observation
)
{
    AerakiaStatus status;
    if (filter == NULL || observation == NULL) return AERAKIA_STATUS_INVALID_ARGUMENT;
    if (!isfinite(observation->height_up_m) || !isfinite(observation->variance_m2)
        || observation->variance_m2 <= 0.0f) {
        return AERAKIA_STATUS_MISSING_MEASUREMENT;
    }
    status = validate_aiding_timestamp(
        filter, observation->timestamp_us, filter->last_barometer_timestamp_us,
        filter->has_barometer_timestamp
    );
    if (status != AERAKIA_STATUS_OK) return status;
    aerakia_eskf_update_barometer(filter, observation->height_up_m, observation->variance_m2);
    filter->last_barometer_timestamp_us = observation->timestamp_us;
    filter->has_barometer_timestamp = true;
    return AERAKIA_STATUS_OK;
}

void aerakia_eskf_apply_zero_velocity(AerakiaEskf *filter, float variance_m2_s2)
{
    if (filter != NULL && variance_m2_s2 > 0.0f) {
        eskf_update_static_constraint(&filter->core, variance_m2_s2);
    }
}

void aerakia_eskf_get_estimate(
    const AerakiaEskf *filter,
    AerakiaNavigationEstimate *estimate
)
{
    int index;
    bool healthy;
    if (filter == NULL || estimate == NULL) {
        return;
    }
    healthy = state_is_finite_and_normalized(&filter->core.state);
    memset(estimate, 0, sizeof(*estimate));
    for (index = 0; index < 4; ++index) {
        estimate->attitude.quaternion_wxyz[index] = (float)filter->core.state.q[index];
    }
    estimate->attitude.euler_rad = quaternion_to_euler(filter->core.state.q);
    estimate->attitude.gravity_body_unit = gravity_body_from_quaternion(filter->core.state.q);
    estimate->attitude.gyro_bias_rad_s.x = (float)filter->core.state.gb[0];
    estimate->attitude.gyro_bias_rad_s.y = (float)filter->core.state.gb[1];
    estimate->attitude.gyro_bias_rad_s.z = (float)filter->core.state.gb[2];
    estimate->attitude.magnetometer_weight = filter->magnetometer_accepted ? 1.0f : 0.0f;
    estimate->attitude.accelerometer_weight = 1.0f;

    estimate->position_ned_m.x = (float)filter->core.state.p[0];
    estimate->position_ned_m.y = (float)filter->core.state.p[1];
    estimate->position_ned_m.z = (float)filter->core.state.p[2];
    estimate->velocity_ned_m_s.x = (float)filter->core.state.v[0];
    estimate->velocity_ned_m_s.y = (float)filter->core.state.v[1];
    estimate->velocity_ned_m_s.z = (float)filter->core.state.v[2];
    estimate->accelerometer_bias_m_s2.x = (float)filter->core.state.ab[0];
    estimate->accelerometer_bias_m_s2.y = (float)filter->core.state.ab[1];
    estimate->accelerometer_bias_m_s2.z = (float)filter->core.state.ab[2];
    estimate->gyroscope_bias_rad_s.x = (float)filter->core.state.gb[0];
    estimate->gyroscope_bias_rad_s.y = (float)filter->core.state.gb[1];
    estimate->gyroscope_bias_rad_s.z = (float)filter->core.state.gb[2];
    for (index = 0; index < 15; ++index) {
        estimate->covariance_diagonal[index] = filter->core.P[index][index];
        healthy = healthy && isfinite(estimate->covariance_diagonal[index])
            && estimate->covariance_diagonal[index] >= 0.0;
    }
    estimate->last_magnetometer_innovation = filter->last_magnetometer_innovation;
    estimate->last_heading_innovation = filter->last_heading_innovation;
    estimate->last_position_innovation = filter->last_position_innovation;
    estimate->last_velocity_innovation = filter->last_velocity_innovation;
    estimate->magnetometer_accepted = filter->magnetometer_accepted;
    estimate->heading_accepted = filter->heading_accepted;
    estimate->position_accepted = filter->position_accepted;
    estimate->velocity_accepted = filter->velocity_accepted;
    estimate->navigation_recovered = filter->navigation_recovered;
    estimate->navigation_recovery_count = filter->navigation_recovery_count;
    estimate->consecutive_navigation_rejections = filter->consecutive_navigation_rejections;
    estimate->static_alignment_complete = filter->static_alignment_complete;
    estimate->static_tilt_alignment_complete = filter->static_tilt_alignment_complete;
    estimate->static_heading_alignment_complete = filter->static_heading_alignment_complete;
    estimate->stationary_detected = filter->stationary_detected;
    estimate->zero_velocity_update_applied = filter->zero_velocity_update_applied;
    estimate->static_alignment_samples = filter->static_alignment_samples;
    estimate->zero_velocity_update_count = filter->zero_velocity_update_count;
    estimate->healthy = healthy;
    estimate->attitude.healthy = healthy;
}
