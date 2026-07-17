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

void aerakia_eskf_default_config(AerakiaEskfConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->minimum_dt_s = 0.0001f;
    config->maximum_dt_s = 0.1f;
    config->fuse_magnetometer = false;
    config->gate_magnetometer = true;
    config->magnetometer_variance = 0.05f;
    config->magnetic_reference_ned[0] = 1.0f;
    config->magnetic_reference_ned[1] = 0.0f;
    config->magnetic_reference_ned[2] = 0.0f;
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

    if (!filter->has_timestamp) {
        filter->last_timestamp_us = sample->timestamp_us;
        filter->has_timestamp = true;
        aerakia_eskf_get_estimate(filter, estimate);
        return AERAKIA_STATUS_INITIALIZED;
    }
    if (sample->timestamp_us <= filter->last_timestamp_us) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }

    dt = (float)(sample->timestamp_us - filter->last_timestamp_us) * 1.0e-6f;
    filter->last_timestamp_us = sample->timestamp_us;
    if (dt < filter->config.minimum_dt_s || dt > filter->config.maximum_dt_s) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
    }

    acceleration[0] = sample->acceleration_m_s2.x;
    acceleration[1] = sample->acceleration_m_s2.y;
    acceleration[2] = sample->acceleration_m_s2.z;
    angular_rate[0] = sample->angular_rate_rad_s.x;
    angular_rate[1] = sample->angular_rate_rad_s.y;
    angular_rate[2] = sample->angular_rate_rad_s.z;
    eskf_predict(&filter->core, acceleration, angular_rate, (eskf_float_t)dt);

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
    eskf_update_position(&filter->core, position, variance_m2, NULL);
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
    bool healthy = true;
    if (filter == NULL || estimate == NULL) {
        return;
    }
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
    estimate->magnetometer_accepted = filter->magnetometer_accepted;
    estimate->healthy = healthy;
    estimate->attitude.healthy = healthy;
}
