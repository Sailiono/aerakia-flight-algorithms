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
    /** Core continuous process-noise profile; initialize with aerakia_eskf_default_config(). */
    ESKF_Config process_noise;
    /** Maximum observation age relative to the latest IMU sample; negative disables the check. */
    float maximum_aiding_age_s;
    /** Independent horizontal position/velocity validity timeouts. */
    float maximum_horizontal_position_dead_reckoning_s;
    float maximum_horizontal_velocity_dead_reckoning_s;
    /** Maximum time without an accepted magnetic/trusted heading observation. */
    float maximum_heading_dead_reckoning_s;
    /** Independent vertical position/velocity validity timeouts. */
    float maximum_vertical_position_dead_reckoning_s;
    float maximum_vertical_velocity_dead_reckoning_s;
    bool fuse_magnetometer;
    bool gate_magnetometer;
    float magnetometer_variance;
    float magnetic_reference_ned[3];
    AerakiaMagGateConfig magnetic_gate;
    uint32_t navigation_recovery_rejection_limit;
    uint32_t navigation_recovery_min_consistent_observations;
    uint32_t navigation_recovery_probationary_acceptances;
    float navigation_recovery_max_candidate_gap_s;
    float navigation_recovery_min_candidate_duration_s;
    float navigation_recovery_authorization_max_age_s;
    float navigation_recovery_probation_min_duration_s;
    float navigation_recovery_probation_max_update_gap_s;
    float recovery_max_position_variance_m2;
    float recovery_max_velocity_variance_m2_s2;
    float recovery_position_consistency_m;
    float recovery_velocity_consistency_m_s;
    float recovery_max_position_correction_m;
    float recovery_max_velocity_correction_m_s;
    float recovery_position_variance_floor_m2;
    float recovery_velocity_variance_floor_m2_s2;
    bool enable_static_alignment;
    bool static_align_attitude;
    float static_alignment_duration_s;
    uint32_t static_alignment_min_samples;
    float static_tilt_uncertainty_rad;
    float static_heading_uncertainty_rad;
    float stationary_gyro_threshold_rad_s;
    float stationary_acceleration_tolerance_m_s2;
    float zero_velocity_interval_s;
    float zero_velocity_variance_m2_s2;
} AerakiaEskfConfig;

typedef struct {
    uint64_t timestamp_us;
    AerakiaVec3f position_ned_m;
    AerakiaVec3f velocity_ned_m_s;
    float position_variance_m2;
    float velocity_variance_m2_s2;
    /** Stable private-supervisor identity and quality sequence used only by recovery safety. */
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
} AerakiaGpsObservation;

typedef struct {
    uint64_t timestamp_us;
    AerakiaVec3f position_ned_m;
    float variance_m2;
} AerakiaPositionObservation;

typedef struct {
    uint64_t timestamp_us;
    AerakiaVec3f velocity_ned_m_s;
    float variance_m2_s2;
} AerakiaVelocityObservation;

typedef struct {
    uint64_t timestamp_us;
    float heading_ned_rad;
    float variance_rad2;
} AerakiaHeadingObservation;

typedef struct {
    uint64_t timestamp_us;
    float height_up_m;
    float variance_m2;
} AerakiaBarometerObservation;

/**
 * One-shot authorization from the private estimator supervisor.
 *
 * The adapter still enforces rejection count, observation consistency, variance, correction
 * bounds, timestamp binding, and post-reset probation. The application attests only that the
 * physical source-quality checks outside this hardware-neutral library passed.
 */
typedef struct {
    uint64_t observation_timestamp_us;
    bool source_quality_verified;
    float maximum_position_correction_m;
    float maximum_velocity_correction_m_s;
    uint32_t source_id;
    uint32_t source_generation;
    uint64_t quality_sequence;
} AerakiaNavigationRecoveryAuthorization;

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
    ESKF_InnovResult last_barometer_innovation;
    bool magnetometer_accepted;
    bool heading_accepted;
    bool position_accepted;
    bool velocity_accepted;
    bool barometer_accepted;
    bool navigation_recovered;
    uint32_t navigation_recovery_count;
    uint32_t consecutive_navigation_rejections;
    uint32_t consecutive_position_rejections;
    uint32_t consecutive_velocity_rejections;
    uint32_t recovery_candidate_consistent_observations;
    float recovery_candidate_duration_s;
    bool navigation_recovery_candidate_ready;
    bool navigation_recovery_probationary;
    uint32_t navigation_recovery_probation_acceptances;
    bool static_alignment_complete;
    bool static_tilt_alignment_complete;
    bool static_heading_alignment_complete;
    bool stationary_detected;
    bool zero_velocity_update_applied;
    uint32_t static_alignment_samples;
    uint32_t zero_velocity_update_count;
    /** Maximum of independent horizontal position/velocity ages; infinity if either is absent. */
    float horizontal_aiding_age_s;
    float horizontal_position_aiding_age_s;
    float horizontal_velocity_aiding_age_s;
    float heading_aiding_age_s;
    float vertical_position_aiding_age_s;
    float vertical_velocity_aiding_age_s;
    bool horizontal_position_valid;
    bool horizontal_velocity_valid;
    bool horizontal_navigation_valid;
    bool heading_valid;
    bool vertical_position_valid;
    bool vertical_velocity_valid;
    bool vertical_navigation_valid;
    bool healthy;
} AerakiaNavigationEstimate;

typedef struct {
    ESKF_Handle core;
    AerakiaMagGate magnetic_gate;
    AerakiaEskfConfig config;
    uint64_t last_timestamp_us;
    uint64_t last_gps_timestamp_us;
    uint64_t last_position_timestamp_us;
    uint64_t last_velocity_timestamp_us;
    uint64_t last_heading_timestamp_us;
    uint64_t last_barometer_timestamp_us;
    uint64_t last_horizontal_position_aiding_timestamp_us;
    uint64_t last_horizontal_velocity_aiding_timestamp_us;
    uint64_t last_heading_aiding_timestamp_us;
    uint64_t last_vertical_position_aiding_timestamp_us;
    uint64_t last_vertical_velocity_aiding_timestamp_us;
    uint32_t rejected_samples;
    ESKF_InnovResult last_magnetometer_innovation;
    ESKF_InnovResult last_heading_innovation;
    ESKF_InnovResult last_position_innovation;
    ESKF_InnovResult last_velocity_innovation;
    ESKF_InnovResult last_barometer_innovation;
    bool magnetometer_accepted;
    bool heading_accepted;
    bool position_accepted;
    bool velocity_accepted;
    bool barometer_accepted;
    bool navigation_recovered;
    uint32_t navigation_recovery_count;
    uint32_t consecutive_navigation_rejections;
    uint32_t consecutive_position_rejections;
    uint32_t consecutive_velocity_rejections;
    AerakiaGpsObservation recovery_candidate;
    uint64_t recovery_candidate_start_timestamp_us;
    uint32_t recovery_candidate_consistent_observations;
    uint32_t navigation_recovery_probation_acceptances;
    uint64_t navigation_recovery_probation_start_timestamp_us;
    uint64_t navigation_recovery_probation_last_timestamp_us;
    uint32_t navigation_recovery_source_id;
    uint32_t navigation_recovery_source_generation;
    uint64_t navigation_recovery_quality_sequence;
    double static_acceleration_sum[3];
    double static_angular_rate_sum[3];
    double static_magnetic_sum[3];
    uint64_t static_alignment_start_timestamp_us;
    uint64_t last_zero_velocity_timestamp_us;
    uint32_t static_alignment_samples;
    uint32_t static_magnetic_samples;
    uint32_t zero_velocity_update_count;
    bool static_alignment_complete;
    bool static_tilt_alignment_complete;
    bool static_heading_alignment_complete;
    bool stationary_detected;
    bool zero_velocity_update_applied;
    bool attitude_seeded;
    bool has_timestamp;
    bool has_gps_timestamp;
    bool has_position_timestamp;
    bool has_velocity_timestamp;
    bool has_heading_timestamp;
    bool has_barometer_timestamp;
    bool has_horizontal_position_aiding_timestamp;
    bool has_horizontal_velocity_aiding_timestamp;
    bool has_heading_aiding_timestamp;
    bool has_vertical_position_aiding_timestamp;
    bool has_vertical_velocity_aiding_timestamp;
    bool has_recovery_candidate;
    bool navigation_recovery_probationary;
    bool horizontal_position_initialized;
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

/**
 * Fuse a timestamped paired GNSS observation.
 *
 * Duplicate, reordered, future, and stale observations are rejected before the filter state is
 * changed. This is the preferred FCOne integration API.
 */
AerakiaStatus aerakia_eskf_update_gps_observation(
    AerakiaEskf *filter,
    const AerakiaGpsObservation *observation
);

/**
 * Apply a bounded, explicitly authorized re-anchor to the latest consistent rejected GNSS pair.
 * The resulting navigation output remains invalid until probationary accepted updates complete.
 */
AerakiaStatus aerakia_eskf_authorize_navigation_recovery(
    AerakiaEskf *filter,
    const AerakiaNavigationRecoveryAuthorization *authorization
);

/** Fuse timestamped position when a receiver does not publish valid velocity. */
AerakiaStatus aerakia_eskf_update_position_observation(
    AerakiaEskf *filter,
    const AerakiaPositionObservation *observation
);

/** Fuse timestamped velocity independently of position validity. */
AerakiaStatus aerakia_eskf_update_velocity_observation(
    AerakiaEskf *filter,
    const AerakiaVelocityObservation *observation
);

/** Fuse a trusted yaw/heading source such as dual-GNSS or vision. */
void aerakia_eskf_update_heading(
    AerakiaEskf *filter,
    float heading_ned_rad,
    float variance_rad2
);

AerakiaStatus aerakia_eskf_update_heading_observation(
    AerakiaEskf *filter,
    const AerakiaHeadingObservation *observation
);

void aerakia_eskf_update_barometer(
    AerakiaEskf *filter,
    float height_up_m,
    float variance_m2
);

AerakiaStatus aerakia_eskf_update_barometer_observation(
    AerakiaEskf *filter,
    const AerakiaBarometerObservation *observation
);

void aerakia_eskf_apply_zero_velocity(AerakiaEskf *filter, float variance_m2_s2);

void aerakia_eskf_get_estimate(
    const AerakiaEskf *filter,
    AerakiaNavigationEstimate *estimate
);

#endif
