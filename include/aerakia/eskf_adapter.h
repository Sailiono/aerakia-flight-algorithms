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
    uint32_t navigation_recovery_rejection_limit;
    float recovery_position_variance_floor_m2;
    float recovery_velocity_variance_floor_m2_s2;
    bool enable_static_alignment;
    float static_alignment_duration_s;
    uint32_t static_alignment_min_samples;
    float stationary_gyro_threshold_rad_s;
    float stationary_acceleration_tolerance_m_s2;
    float zero_velocity_interval_s;
    float zero_velocity_variance_m2_s2;
} AerakiaEskfConfig;

typedef struct {
    AerakiaAttitudeEstimate attitude;
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
    AerakiaVec3f accelerometer_bias_m_s2;
    AerakiaVec3f gyroscope_bias_rad_s;
    double covariance_diagonal[15];
    ESKF_InnovResult last_magnetometer_innovation;
    ESKF_InnovResult last_heading_innovation;
    ESKF_InnovResult last_position_innovation;
    ESKF_InnovResult last_velocity_innovation;
    bool magnetometer_accepted;
    bool heading_accepted;
    bool position_accepted;
    bool velocity_accepted;
    bool navigation_recovered;
    uint32_t navigation_recovery_count;
    uint32_t consecutive_navigation_rejections;
    bool static_alignment_complete;
    bool stationary_detected;
    bool zero_velocity_update_applied;
    uint32_t static_alignment_samples;
    uint32_t zero_velocity_update_count;
    bool healthy;
} AerakiaNavigationEstimate;

typedef struct {
    ESKF_Handle core;
    AerakiaMagGate magnetic_gate;
    AerakiaEskfConfig config;
    uint64_t last_timestamp_us;
    uint32_t rejected_samples;
    ESKF_InnovResult last_magnetometer_innovation;
    ESKF_InnovResult last_heading_innovation;
    ESKF_InnovResult last_position_innovation;
    ESKF_InnovResult last_velocity_innovation;
    bool magnetometer_accepted;
    bool heading_accepted;
    bool position_accepted;
    bool velocity_accepted;
    bool navigation_recovered;
    uint32_t navigation_recovery_count;
    uint32_t consecutive_navigation_rejections;
    double static_acceleration_sum[3];
    double static_angular_rate_sum[3];
    uint64_t static_alignment_start_timestamp_us;
    uint64_t last_zero_velocity_timestamp_us;
    uint32_t static_alignment_samples;
    uint32_t zero_velocity_update_count;
    bool static_alignment_complete;
    bool stationary_detected;
    bool zero_velocity_update_applied;
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

void aerakia_eskf_update_velocity(
    AerakiaEskf *filter,
    AerakiaVec3f velocity_ned_m_s,
    float variance_m2_s2
);

/** Fuse one paired GNSS position/velocity observation with recovery supervision. */
void aerakia_eskf_update_gps(
    AerakiaEskf *filter,
    AerakiaVec3f position_ned_m,
    AerakiaVec3f velocity_ned_m_s,
    float position_variance_m2,
    float velocity_variance_m2_s2
);

/** Fuse a trusted yaw/heading source such as dual-GNSS or vision. */
void aerakia_eskf_update_heading(
    AerakiaEskf *filter,
    float heading_ned_rad,
    float variance_rad2
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
