/**
 * @file eskf_adapter.h
 * @brief Driver-facing adapter for the portable ESKF core.
 */

#ifndef AERAKIA_ESKF_ADAPTER_H
#define AERAKIA_ESKF_ADAPTER_H

#include <aerakia/eskf.h>
#include <aerakia/mag_gate.h>
#include <aerakia/types.h>

typedef struct {
    float minimum_dt_s;
    float maximum_dt_s;
    bool fuse_magnetometer;
    bool gate_magnetometer;
    float magnetometer_variance;
    float magnetic_reference_ned[3];
    AerakiaMagGateConfig magnetic_gate;
} AerakiaEskfConfig;

typedef struct {
    AerakiaAttitudeEstimate attitude;
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
    AerakiaVec3f accelerometer_bias_m_s2;
    AerakiaVec3f gyroscope_bias_rad_s;
    double covariance_diagonal[15];
    ESKF_InnovResult last_magnetometer_innovation;
    bool magnetometer_accepted;
    bool healthy;
} AerakiaNavigationEstimate;

typedef struct {
    ESKF_Handle core;
    AerakiaMagGate magnetic_gate;
    AerakiaEskfConfig config;
    uint64_t last_timestamp_us;
    uint32_t rejected_samples;
    ESKF_InnovResult last_magnetometer_innovation;
    bool magnetometer_accepted;
    bool has_timestamp;
} AerakiaEskf;

void aerakia_eskf_default_config(AerakiaEskfConfig *config);

void aerakia_eskf_init(
    AerakiaEskf *filter,
    const AerakiaEskfConfig *config,
    const double initial_position_ned_m[3],
    const double initial_quaternion_wxyz[4]
);

/** Predict from one driver-published IMU sample and optionally fuse its mag. */
AerakiaStatus aerakia_eskf_process_imu(
    AerakiaEskf *filter,
    const AerakiaImuSample *sample,
    AerakiaNavigationEstimate *estimate
);

void aerakia_eskf_update_position(
    AerakiaEskf *filter,
    AerakiaVec3f position_ned_m,
    float variance_m2
);

void aerakia_eskf_update_barometer(
    AerakiaEskf *filter,
    float height_up_m,
    float variance_m2
);

void aerakia_eskf_apply_zero_velocity(AerakiaEskf *filter, float variance_m2_s2);

void aerakia_eskf_get_estimate(
    const AerakiaEskf *filter,
    AerakiaNavigationEstimate *estimate
);

#endif
