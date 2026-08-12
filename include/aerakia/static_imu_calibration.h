/**
 * @file static_imu_calibration.h
 * @brief Explicit preflight multi-pose static IMU bias calibration.
 *
 * This utility estimates only constant accelerometer and gyroscope offsets
 * from several stationary pose means in the final calibrated body-FRD frame.
 * It does not estimate scale, misalignment, temperature behavior, vibration,
 * or in-flight bias drift. A rejected result must never be used as an ESKF
 * bias seed.
 */

#ifndef AERAKIA_STATIC_IMU_CALIBRATION_H
#define AERAKIA_STATIC_IMU_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include <aerakia/types.h>

/** Bounded preflight procedure; more poses must be reduced offline first. */
#define AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES 16U

typedef struct {
    /** Mean calibrated acceleration in final body FRD coordinates, m/s^2. */
    AerakiaVec3f acceleration_m_s2;
    /** Mean calibrated angular rate in final body FRD coordinates, rad/s. */
    AerakiaVec3f angular_rate_rad_s;
    /** Number of raw samples represented by this stationary mean. */
    uint32_t sample_count;
} AerakiaStaticImuPoseMean;

typedef struct {
    /** At least four non-coplanar poses are required; six is the default. */
    uint32_t minimum_pose_count;
    /** Local gravity magnitude expected by the calibrated accelerometer. */
    float gravity_m_s2;
    /** Raw stationary gyro-mean norm gate before bias estimation. */
    float maximum_stationary_gyro_norm_rad_s;
    /** Minimum lambda_min / lambda_max of the sphere-fit normal matrix. */
    float minimum_geometry_eigenvalue_ratio;
    /** Minimum eigenvalue of mean(u*u^T), where u is corrected gravity direction. */
    float minimum_direction_coverage_eigenvalue;
    /** Maximum absolute radial gravity residual of an accepted pose mean. */
    float maximum_gravity_residual_m_s2;
    /** Maximum RMS radial gravity residual across all accepted pose means. */
    float maximum_gravity_residual_rms_m_s2;
    /** Maximum RMS scatter of pose gyro means about their common bias. */
    float maximum_gyroscope_residual_rms_rad_s;
    /**
     * Maximum change in the fitted accelerometer bias when one pose is left
     * out. This is an influence diagnostic, not a replacement for private
     * window-quality checks. It is applied only when six or more poses are
     * supplied, so the default six-pose procedure retains at least five poses
     * for every leave-one-out fit.
     */
    float maximum_leave_one_out_bias_delta_m_s2;
} AerakiaStaticImuCalibrationConfig;

typedef enum {
    AERAKIA_STATIC_IMU_CALIBRATION_OK = 0,
    AERAKIA_STATIC_IMU_CALIBRATION_INVALID_ARGUMENT = -1,
    AERAKIA_STATIC_IMU_CALIBRATION_INSUFFICIENT_POSES = -2,
    AERAKIA_STATIC_IMU_CALIBRATION_TOO_MANY_POSES = -3,
    AERAKIA_STATIC_IMU_CALIBRATION_NONFINITE_INPUT = -4,
    AERAKIA_STATIC_IMU_CALIBRATION_NONSTATIONARY_INPUT = -5,
    AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY = -6,
    AERAKIA_STATIC_IMU_CALIBRATION_GRAVITY_RESIDUAL_EXCEEDED = -7,
    AERAKIA_STATIC_IMU_CALIBRATION_GYROSCOPE_RESIDUAL_EXCEEDED = -8,
    AERAKIA_STATIC_IMU_CALIBRATION_OUTLIER_SENSITIVITY_EXCEEDED = -9
} AerakiaStaticImuCalibrationStatus;

typedef struct {
    bool accepted;
    AerakiaStaticImuCalibrationStatus status;
    uint32_t pose_count;
    AerakiaVec3f accelerometer_bias_m_s2;
    AerakiaVec3f gyroscope_bias_rad_s;
    float gravity_residual_rms_m_s2;
    float gravity_residual_max_abs_m_s2;
    float gyroscope_residual_rms_rad_s;
    /** Ascending eigenvalues of the pairwise sphere-fit normal matrix. */
    float geometry_eigenvalues[3];
    /** Ascending eigenvalues of corrected gravity-direction coverage. */
    float direction_coverage_eigenvalues[3];
    /** Largest leave-one-pose-out change in the fitted accelerometer bias. */
    float maximum_leave_one_out_bias_delta_m_s2;
} AerakiaStaticImuCalibrationResult;

void aerakia_static_imu_calibration_default_config(
    AerakiaStaticImuCalibrationConfig *config
);

/**
 * Estimate a constant accelerometer bias from the gravity sphere and a
 * constant gyroscope bias from stationary means.
 *
 * Every pose must be measured while stationary and already transformed into
 * the final calibrated body frame. The function performs no allocation and
 * never removes outliers. It rejects the whole calibration when its residual,
 * geometry, gyro-scatter, or leave-one-out influence contracts are exceeded;
 * these checks do not guarantee detection of every possible corrupted pose.
 * At most AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES pose means are accepted.
 */
AerakiaStaticImuCalibrationStatus aerakia_static_imu_calibrate(
    const AerakiaStaticImuPoseMean *poses,
    uint32_t pose_count,
    const AerakiaStaticImuCalibrationConfig *config,
    AerakiaStaticImuCalibrationResult *result
);

#endif
