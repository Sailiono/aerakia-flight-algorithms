/**
 * @file mahony.h
 * @brief Context-based nonlinear complementary attitude filter.
 */

#ifndef AERAKIA_MAHONY_H
#define AERAKIA_MAHONY_H

#include <aerakia/mag_gate.h>
#include <aerakia/types.h>

typedef struct {
    float kp_accelerometer;
    float kp_magnetometer;
    float ki;
    float integral_limit_rad_s;
    float accelerometer_full_trust_error_m_s2;
    float accelerometer_min_trust_error_m_s2;
    float minimum_accelerometer_weight;
    float magnetometer_min_ut;
    float magnetometer_max_ut;
    float magnetometer_weight;
    float minimum_dt_s;
    float maximum_dt_s;
    bool adaptive_accelerometer;
    bool yaw_only_magnetometer;
    bool gate_magnetometer;
    AerakiaMagGateConfig magnetic_gate;
} AerakiaMahonyConfig;

typedef struct {
    AerakiaMahonyConfig config;
    AerakiaMagGate magnetic_gate;
    float quaternion_wxyz[4];
    AerakiaVec3f integral_feedback_rad_s;
    AerakiaVec3f gravity_body_unit;
    AerakiaVec3f accelerometer_error;
    AerakiaVec3f magnetometer_error;
    float accelerometer_weight;
    float magnetometer_weight;
    uint64_t last_timestamp_us;
    uint32_t rejected_samples;
    bool has_timestamp;
    bool initialized;
    bool healthy;
} AerakiaMahony;

/** Robust Aerakia defaults. Disable all three robustness booleans for a standard baseline. */
void aerakia_mahony_default_config(AerakiaMahonyConfig *config);

void aerakia_mahony_init(AerakiaMahony *filter, const AerakiaMahonyConfig *config);

/** Seed a trusted body-to-NED attitude before the first timestamped sample. */
AerakiaStatus aerakia_mahony_seed_attitude(
    AerakiaMahony *filter,
    const float quaternion_wxyz[4]
);

/** Initialize attitude from an accelerometer and optional magnetometer sample. */
AerakiaStatus aerakia_mahony_initialize_from_sample(
    AerakiaMahony *filter,
    const AerakiaImuSample *sample,
    AerakiaAttitudeEstimate *estimate
);

/** Update using timestamp-derived dt and return the current attitude. */
AerakiaStatus aerakia_mahony_update(
    AerakiaMahony *filter,
    const AerakiaImuSample *sample,
    AerakiaAttitudeEstimate *estimate
);

void aerakia_mahony_get_estimate(
    const AerakiaMahony *filter,
    AerakiaAttitudeEstimate *estimate
);

#endif
