/**
 * @file mahony.c
 * @brief Nonlinear complementary filter implemented from the published model.
 *
 * Reference: Mahony, Hamel, Pflimlin, "Nonlinear Complementary Filters on
 * the Special Orthogonal Group", IEEE TAC 53(5), 2008.
 */

#include <aerakia/mahony.h>

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

static float vector_norm(AerakiaVec3f vector)
{
    return sqrtf(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

static bool normalize(AerakiaVec3f *vector)
{
    const float norm = vector_norm(*vector);
    if (!(norm > 1.0e-6f) || !isfinite(norm)) {
        return false;
    }
    vector->x /= norm;
    vector->y /= norm;
    vector->z /= norm;
    return true;
}

static AerakiaVec3f cross(AerakiaVec3f left, AerakiaVec3f right)
{
    AerakiaVec3f result = {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
    return result;
}

static void quaternion_normalize(float quaternion[4])
{
    const float norm = sqrtf(
        quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3]
    );
    if (!(norm > 1.0e-6f) || !isfinite(norm)) {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }
    quaternion[0] /= norm;
    quaternion[1] /= norm;
    quaternion[2] /= norm;
    quaternion[3] /= norm;
}

static void quaternion_to_rotation(const float q[4], float rotation[3][3])
{
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];

    rotation[0][0] = 1.0f - 2.0f * (y * y + z * z);
    rotation[0][1] = 2.0f * (x * y - w * z);
    rotation[0][2] = 2.0f * (x * z + w * y);
    rotation[1][0] = 2.0f * (x * y + w * z);
    rotation[1][1] = 1.0f - 2.0f * (x * x + z * z);
    rotation[1][2] = 2.0f * (y * z - w * x);
    rotation[2][0] = 2.0f * (x * z - w * y);
    rotation[2][1] = 2.0f * (y * z + w * x);
    rotation[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void quaternion_from_euler(float roll, float pitch, float yaw, float q[4])
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    quaternion_normalize(q);
}

static AerakiaVec3f quaternion_to_euler(const float q[4])
{
    AerakiaVec3f result;
    const float sin_pitch = clampf(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f);
    result.x = atan2f(
        2.0f * (q[0] * q[1] + q[2] * q[3]),
        1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])
    );
    result.y = asinf(sin_pitch);
    result.z = atan2f(
        2.0f * (q[0] * q[3] + q[1] * q[2]),
        1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])
    );
    return result;
}

static bool magnetometer_is_valid(const AerakiaMahony *filter, AerakiaVec3f magnetic_field)
{
    const float norm = vector_norm(magnetic_field);
    return vector_is_finite(magnetic_field)
        && norm >= filter->config.magnetometer_min_ut
        && norm <= filter->config.magnetometer_max_ut;
}

void aerakia_mahony_default_config(AerakiaMahonyConfig *config)
{
    if (config == NULL) {
        return;
    }
    config->kp_accelerometer = 0.8f;
    config->kp_magnetometer = 0.6f;
    config->ki = 0.02f;
    config->integral_limit_rad_s = 0.2f;
    config->accelerometer_full_trust_error_m_s2 = 0.15f * AERAKIA_GRAVITY_M_S2;
    config->accelerometer_min_trust_error_m_s2 = 0.60f * AERAKIA_GRAVITY_M_S2;
    config->minimum_accelerometer_weight = 0.05f;
    config->magnetometer_min_ut = 10.0f;
    config->magnetometer_max_ut = 100.0f;
    config->magnetometer_weight = 0.5f;
    config->minimum_dt_s = 0.0001f;
    config->maximum_dt_s = 0.1f;
    config->adaptive_accelerometer = true;
    config->yaw_only_magnetometer = true;
    config->gate_magnetometer = true;
    aerakia_mag_gate_default_config(&config->magnetic_gate);
}

void aerakia_mahony_init(AerakiaMahony *filter, const AerakiaMahonyConfig *config)
{
    AerakiaMahonyConfig defaults;
    if (filter == NULL) {
        return;
    }
    aerakia_mahony_default_config(&defaults);
    memset(filter, 0, sizeof(*filter));
    filter->config = config != NULL ? *config : defaults;
    aerakia_mag_gate_init(&filter->magnetic_gate, &filter->config.magnetic_gate);
    filter->quaternion_wxyz[0] = 1.0f;
    filter->gravity_body_unit.z = -1.0f;
    filter->accelerometer_weight = 1.0f;
    filter->healthy = true;
}

AerakiaStatus aerakia_mahony_seed_attitude(
    AerakiaMahony *filter,
    const float quaternion_wxyz[4]
)
{
    float norm;
    float rotation[3][3];
    int index;

    if (filter == NULL || quaternion_wxyz == NULL) {
        return AERAKIA_STATUS_INVALID_ARGUMENT;
    }
    norm = sqrtf(
        quaternion_wxyz[0] * quaternion_wxyz[0]
        + quaternion_wxyz[1] * quaternion_wxyz[1]
        + quaternion_wxyz[2] * quaternion_wxyz[2]
        + quaternion_wxyz[3] * quaternion_wxyz[3]
    );
    if (!(norm > 1.0e-6f) || !isfinite(norm)) {
        return AERAKIA_STATUS_INVALID_ARGUMENT;
    }
    for (index = 0; index < 4; ++index) {
        if (!isfinite(quaternion_wxyz[index])) {
            return AERAKIA_STATUS_INVALID_ARGUMENT;
        }
        filter->quaternion_wxyz[index] = quaternion_wxyz[index] / norm;
    }
    memset(&filter->integral_feedback_rad_s, 0, sizeof(filter->integral_feedback_rad_s));
    memset(&filter->accelerometer_error, 0, sizeof(filter->accelerometer_error));
    memset(&filter->magnetometer_error, 0, sizeof(filter->magnetometer_error));
    filter->accelerometer_weight = 1.0f;
    filter->magnetometer_weight = 0.0f;
    filter->last_timestamp_us = 0U;
    filter->has_timestamp = false;
    filter->initialized = true;
    filter->healthy = true;
    quaternion_to_rotation(filter->quaternion_wxyz, rotation);
    filter->gravity_body_unit.x = -rotation[2][0];
    filter->gravity_body_unit.y = -rotation[2][1];
    filter->gravity_body_unit.z = -rotation[2][2];
    return AERAKIA_STATUS_INITIALIZED;
}

AerakiaStatus aerakia_mahony_initialize_from_sample(
    AerakiaMahony *filter,
    const AerakiaImuSample *sample,
    AerakiaAttitudeEstimate *estimate
)
{
    AerakiaVec3f gravity;
    float roll;
    float pitch;
    float yaw = 0.0f;

    if (filter == NULL || sample == NULL) {
        return AERAKIA_STATUS_INVALID_ARGUMENT;
    }
    if ((sample->flags & AERAKIA_SAMPLE_ACCEL_VALID) == 0U
        || !vector_is_finite(sample->acceleration_m_s2)) {
        return AERAKIA_STATUS_MISSING_MEASUREMENT;
    }

    gravity.x = -sample->acceleration_m_s2.x;
    gravity.y = -sample->acceleration_m_s2.y;
    gravity.z = -sample->acceleration_m_s2.z;
    if (!normalize(&gravity)) {
        return AERAKIA_STATUS_NUMERICAL_ERROR;
    }

    roll = atan2f(gravity.y, gravity.z);
    pitch = atan2f(-gravity.x, sqrtf(gravity.y * gravity.y + gravity.z * gravity.z));

    if ((sample->flags & AERAKIA_SAMPLE_MAG_VALID) != 0U
        && magnetometer_is_valid(filter, sample->magnetic_field_ut)) {
        AerakiaVec3f magnetic = sample->magnetic_field_ut;
        const float cos_roll = cosf(roll);
        const float sin_roll = sinf(roll);
        const float cos_pitch = cosf(pitch);
        const float sin_pitch = sinf(pitch);
        (void)normalize(&magnetic);
        const float horizontal_x = magnetic.x * cos_pitch
            + magnetic.y * sin_roll * sin_pitch
            + magnetic.z * cos_roll * sin_pitch;
        const float horizontal_y = magnetic.y * cos_roll - magnetic.z * sin_roll;
        yaw = atan2f(-horizontal_y, horizontal_x);
        if (filter->config.gate_magnetometer) {
            (void)aerakia_mag_gate_accept(
                &filter->magnetic_gate,
                sample->magnetic_field_ut.x,
                sample->magnetic_field_ut.y,
                sample->magnetic_field_ut.z
            );
        }
    }

    quaternion_from_euler(roll, pitch, yaw, filter->quaternion_wxyz);
    filter->last_timestamp_us = sample->timestamp_us;
    filter->has_timestamp = true;
    filter->initialized = true;
    filter->healthy = true;
    aerakia_mahony_get_estimate(filter, estimate);
    return AERAKIA_STATUS_INITIALIZED;
}

AerakiaStatus aerakia_mahony_update(
    AerakiaMahony *filter,
    const AerakiaImuSample *sample,
    AerakiaAttitudeEstimate *estimate
)
{
    AerakiaVec3f accelerometer;
    AerakiaVec3f angular_rate;
    AerakiaVec3f accelerometer_error = {0.0f, 0.0f, 0.0f};
    AerakiaVec3f magnetometer_error = {0.0f, 0.0f, 0.0f};
    float rotation[3][3];
    float dt;
    float acceleration_norm;
    float accelerometer_weight = 0.0f;
    float magnetometer_weight = 0.0f;
    float q_previous[4];

    if (filter == NULL || sample == NULL) {
        return AERAKIA_STATUS_INVALID_ARGUMENT;
    }
    if (!filter->initialized) {
        return aerakia_mahony_initialize_from_sample(filter, sample, estimate);
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
        aerakia_mahony_get_estimate(filter, estimate);
        return AERAKIA_STATUS_INITIALIZED;
    }
    if (sample->timestamp_us <= filter->last_timestamp_us) {
        filter->rejected_samples++;
        return AERAKIA_STATUS_TIMESTAMP_ERROR;
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

    accelerometer = sample->acceleration_m_s2;
    angular_rate = sample->angular_rate_rad_s;
    acceleration_norm = vector_norm(accelerometer);
    if (!normalize(&accelerometer)) {
        filter->healthy = false;
        return AERAKIA_STATUS_NUMERICAL_ERROR;
    }

    quaternion_to_rotation(filter->quaternion_wxyz, rotation);
    filter->gravity_body_unit.x = -rotation[2][0];
    filter->gravity_body_unit.y = -rotation[2][1];
    filter->gravity_body_unit.z = -rotation[2][2];

    accelerometer_weight = 1.0f;
    if (filter->config.adaptive_accelerometer) {
        const float error = fabsf(acceleration_norm - AERAKIA_GRAVITY_M_S2);
        const float low = filter->config.accelerometer_full_trust_error_m_s2;
        const float high = filter->config.accelerometer_min_trust_error_m_s2;
        if (error >= high) {
            accelerometer_weight = filter->config.minimum_accelerometer_weight;
        } else if (error > low && high > low) {
            const float blend = (error - low) / (high - low);
            accelerometer_weight = 1.0f
                - blend * (1.0f - filter->config.minimum_accelerometer_weight);
        }
    }
    accelerometer_error = cross(accelerometer, filter->gravity_body_unit);

    if ((sample->flags & AERAKIA_SAMPLE_MAG_VALID) != 0U
        && magnetometer_is_valid(filter, sample->magnetic_field_ut)
        && (!filter->config.gate_magnetometer
            || aerakia_mag_gate_accept(
                &filter->magnetic_gate,
                sample->magnetic_field_ut.x,
                sample->magnetic_field_ut.y,
                sample->magnetic_field_ut.z
            ))) {
        AerakiaVec3f measured = sample->magnetic_field_ut;
        AerakiaVec3f earth;
        AerakiaVec3f reference;
        AerakiaVec3f predicted;
        (void)normalize(&measured);

        earth.x = rotation[0][0] * measured.x + rotation[0][1] * measured.y + rotation[0][2] * measured.z;
        earth.y = rotation[1][0] * measured.x + rotation[1][1] * measured.y + rotation[1][2] * measured.z;
        earth.z = rotation[2][0] * measured.x + rotation[2][1] * measured.y + rotation[2][2] * measured.z;
        reference.x = sqrtf(earth.x * earth.x + earth.y * earth.y);
        reference.y = 0.0f;
        reference.z = earth.z;

        predicted.x = rotation[0][0] * reference.x + rotation[1][0] * reference.y + rotation[2][0] * reference.z;
        predicted.y = rotation[0][1] * reference.x + rotation[1][1] * reference.y + rotation[2][1] * reference.z;
        predicted.z = rotation[0][2] * reference.x + rotation[1][2] * reference.y + rotation[2][2] * reference.z;
        (void)normalize(&predicted);
        magnetometer_error = cross(measured, predicted);
        if (filter->config.yaw_only_magnetometer) {
            magnetometer_error.x = 0.0f;
            magnetometer_error.y = 0.0f;
        }
        magnetometer_weight = filter->config.magnetometer_weight
            * (filter->config.adaptive_accelerometer ? accelerometer_weight : 1.0f);
    }

    filter->integral_feedback_rad_s.x += filter->config.ki
        * (accelerometer_error.x * accelerometer_weight + magnetometer_error.x * magnetometer_weight) * dt;
    filter->integral_feedback_rad_s.y += filter->config.ki
        * (accelerometer_error.y * accelerometer_weight + magnetometer_error.y * magnetometer_weight) * dt;
    filter->integral_feedback_rad_s.z += filter->config.ki
        * (accelerometer_error.z * accelerometer_weight + magnetometer_error.z * magnetometer_weight) * dt;
    filter->integral_feedback_rad_s.x = clampf(filter->integral_feedback_rad_s.x, -filter->config.integral_limit_rad_s, filter->config.integral_limit_rad_s);
    filter->integral_feedback_rad_s.y = clampf(filter->integral_feedback_rad_s.y, -filter->config.integral_limit_rad_s, filter->config.integral_limit_rad_s);
    filter->integral_feedback_rad_s.z = clampf(filter->integral_feedback_rad_s.z, -filter->config.integral_limit_rad_s, filter->config.integral_limit_rad_s);

    angular_rate.x += filter->config.kp_accelerometer * accelerometer_error.x * accelerometer_weight
        + filter->config.kp_magnetometer * magnetometer_error.x * magnetometer_weight
        + filter->integral_feedback_rad_s.x;
    angular_rate.y += filter->config.kp_accelerometer * accelerometer_error.y * accelerometer_weight
        + filter->config.kp_magnetometer * magnetometer_error.y * magnetometer_weight
        + filter->integral_feedback_rad_s.y;
    angular_rate.z += filter->config.kp_accelerometer * accelerometer_error.z * accelerometer_weight
        + filter->config.kp_magnetometer * magnetometer_error.z * magnetometer_weight
        + filter->integral_feedback_rad_s.z;

    memcpy(q_previous, filter->quaternion_wxyz, sizeof(q_previous));
    filter->quaternion_wxyz[0] += 0.5f * (-q_previous[1] * angular_rate.x - q_previous[2] * angular_rate.y - q_previous[3] * angular_rate.z) * dt;
    filter->quaternion_wxyz[1] += 0.5f * ( q_previous[0] * angular_rate.x + q_previous[2] * angular_rate.z - q_previous[3] * angular_rate.y) * dt;
    filter->quaternion_wxyz[2] += 0.5f * ( q_previous[0] * angular_rate.y - q_previous[1] * angular_rate.z + q_previous[3] * angular_rate.x) * dt;
    filter->quaternion_wxyz[3] += 0.5f * ( q_previous[0] * angular_rate.z + q_previous[1] * angular_rate.y - q_previous[2] * angular_rate.x) * dt;
    quaternion_normalize(filter->quaternion_wxyz);

    filter->accelerometer_error = accelerometer_error;
    filter->magnetometer_error = magnetometer_error;
    filter->accelerometer_weight = accelerometer_weight;
    filter->magnetometer_weight = magnetometer_weight;
    filter->healthy = isfinite(filter->quaternion_wxyz[0])
        && isfinite(filter->quaternion_wxyz[1])
        && isfinite(filter->quaternion_wxyz[2])
        && isfinite(filter->quaternion_wxyz[3]);
    aerakia_mahony_get_estimate(filter, estimate);
    return filter->healthy ? AERAKIA_STATUS_OK : AERAKIA_STATUS_NUMERICAL_ERROR;
}

void aerakia_mahony_get_estimate(
    const AerakiaMahony *filter,
    AerakiaAttitudeEstimate *estimate
)
{
    if (filter == NULL || estimate == NULL) {
        return;
    }
    memcpy(estimate->quaternion_wxyz, filter->quaternion_wxyz, sizeof(estimate->quaternion_wxyz));
    estimate->euler_rad = quaternion_to_euler(filter->quaternion_wxyz);
    estimate->gravity_body_unit = filter->gravity_body_unit;
    estimate->gyro_bias_rad_s.x = -filter->integral_feedback_rad_s.x;
    estimate->gyro_bias_rad_s.y = -filter->integral_feedback_rad_s.y;
    estimate->gyro_bias_rad_s.z = -filter->integral_feedback_rad_s.z;
    estimate->accelerometer_weight = filter->accelerometer_weight;
    estimate->magnetometer_weight = filter->magnetometer_weight;
    estimate->healthy = filter->healthy;
}
