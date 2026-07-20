/**
 * @file eskf_types.h
 * @brief Core Data Structures for Error-State Kalman Filter
 *
 * Based on Joan Solà "Quaternion kinematics for the error-state Kalman filter"
 *
 * Architecture:
 *   - Nominal State (16 components): Position, Velocity, Quaternion, Accel Bias, Gyro Bias
 *   - Error State (15-DOF): δθ, δv, δp, δab, δgb
 *
 * Coordinate Frames:
 *   - Navigation Frame: NED (North-East-Down)
 *   - Body Frame: FRD (Forward-Right-Down)
 *
 * Maintained by Aerakia contributors.
 */

#ifndef ESKF_TYPES_H
#define ESKF_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/*
 * The reviewed public default is double precision.  A single-precision build
 * is available only as a host/target evaluation profile; it must not be
 * selected for a flight target until its own numerical and timing evidence is
 * recorded.  CMake exports the definition to all consumers so public structs
 * never disagree about their layout.
 */
#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
typedef float eskf_float_t;
#else
typedef double eskf_float_t;
#endif

#define ESKF_PI         3.14159265358979323846
#define ESKF_GRAVITY    9.80665
#define ESKF_EPSILON    1e-12

/* ============================================================================
 * Error State Index Definitions
 * Order: dtheta(0-2), dv(3-5), dp(6-8), dab(9-11), dgb(12-14)
 * ============================================================================ */

#define ESKF_IDX_DTHETA     0   /* Attitude error (0-2) */
#define ESKF_IDX_DV         3   /* Velocity error (3-5) */
#define ESKF_IDX_DP         6   /* Position error (6-8) */
#define ESKF_IDX_DAB        9   /* Accel bias error (9-11) */
#define ESKF_IDX_DGB        12  /* Gyro bias error (12-14) */

#define ESKF_ERROR_STATE_DIM    15
#define ESKF_NOMINAL_STATE_DIM  16  /* 3 + 3 + 4 + 3 + 3 */

/* ============================================================================
 * Nominal State Structure (16 components)
 * ============================================================================ */

/**
 * @brief Nominal State Vector
 *
 * Contains the physically meaningful state estimate.
 * This is the "true" state that we integrate forward in time.
 */
typedef struct {
    eskf_float_t p[3];    /**< Position [N, E, D] in meters */
    eskf_float_t v[3];    /**< Velocity [N, E, D] in m/s */
    eskf_float_t q[4];    /**< Quaternion [w, x, y, z], Body to Earth */
    eskf_float_t ab[3];   /**< Accelerometer bias in m/s² */
    eskf_float_t gb[3];   /**< Gyroscope bias in rad/s */
} ESKF_NominalState;

/* ============================================================================
 * Error State Structure (15-DOF)
 * ============================================================================ */

/**
 * @brief Error State Vector
 *
 * Represents the small perturbation from the nominal state.
 * In ESKF, this is what the Kalman filter actually estimates.
 * After each update, errors are injected into nominal state and reset to zero.
 */
typedef struct {
    eskf_float_t dtheta[3];  /**< Attitude error (Rodrigues rotation vector) */
    eskf_float_t dv[3];      /**< Velocity error in m/s */
    eskf_float_t dp[3];      /**< Position error in meters */
    eskf_float_t dab[3];     /**< Accelerometer bias error in m/s² */
    eskf_float_t dgb[3];     /**< Gyroscope bias error in rad/s */
} ESKF_ErrorState;

/* ============================================================================
 * Configuration Structure
 * ============================================================================ */

/**
 * @brief Filter Configuration Parameters
 *
 * Process noise parameters (continuous-time standard deviations)
 */
typedef struct {
    eskf_float_t sigma_acc;       /**< Accelerometer noise σ (m/s²/√Hz) */
    eskf_float_t sigma_gyr;       /**< Gyroscope noise σ (rad/s/√Hz) */
    eskf_float_t sigma_acc_bias;  /**< Accel bias random walk σ (m/s³/√Hz) */
    eskf_float_t sigma_gyr_bias;  /**< Gyro bias random walk σ (rad/s²/√Hz) */
} ESKF_Config;

/* ============================================================================
 * Innovation Quality Monitoring
 * ============================================================================ */

/**
 * @brief Innovation Test Result for Integrity Monitoring
 *
 * Returned by measurement update functions to report gating results.
 * Enables detection of sensor faults and dynamic maneuvers.
 */
typedef struct {
    bool accepted;           /**< True if update passed gating and was applied */
    float nis;               /**< Normalized innovation squared (Mahalanobis distance squared) */
    float test_ratio;        /**< Innovation test ratio: NIS / chi-square limit (< 1 healthy) */
    float innovation[3];     /**< Raw innovation vector: measurement - prediction */
    float innov_var[3];      /**< Innovation variance (diagonal of S matrix) */
} ESKF_InnovResult;

/* ============================================================================
 * Main Filter Handle
 * ============================================================================ */

/**
 * @brief Main ESKF Handle Structure
 *
 * This is the "object" that holds all filter state.
 * It must be allocated by the caller (static or stack).
 *
 * Memory Layout:
 *   - state: 16 * sizeof(double) = 128 bytes
 *   - error_state: 15 * sizeof(double) = 120 bytes (transient, often zero)
 *   - P: 15 * 15 * sizeof(double) = 1800 bytes
 *   - cfg: 4 * sizeof(double) = 32 bytes
 *   - misc: ~50 bytes
 *   Total: ~2130 bytes
 */
typedef struct {
    /* State Estimates */
    ESKF_NominalState state;       /**< Nominal state (integrated) */
    ESKF_ErrorState   error_state; /**< Error state (transient, reset after update) */

    /* Covariance Matrix */
    eskf_float_t P[15][15];        /**< Error state covariance */

    /* Configuration */
    ESKF_Config cfg;               /**< Process noise parameters */

    /* Reference Vectors (for measurement updates) */
    eskf_float_t mag_ref[3];       /**< Magnetic field reference in NED frame */
    eskf_float_t gravity[3];       /**< Gravity vector in NED frame [0, 0, +g] */

    /* Status Flags */
    bool initialized;              /**< True if filter has been initialized */

} ESKF_Handle;

/* ============================================================================
 * IMU Measurement Structure
 * ============================================================================ */

/**
 * @brief IMU Measurement Input
 */
typedef struct {
    eskf_float_t acc[3];   /**< Accelerometer reading [x, y, z] in m/s² */
    eskf_float_t gyr[3];   /**< Gyroscope reading [x, y, z] in rad/s */
    eskf_float_t dt;       /**< Time step in seconds */
} ESKF_IMU_Input;

/* ============================================================================
 * Measurement Structures
 * ============================================================================ */

/**
 * @brief GPS Position Measurement
 */
typedef struct {
    eskf_float_t pos_ned[3];  /**< Position in NED frame [N, E, D] in meters */
    eskf_float_t R_pos;       /**< Position measurement noise variance (m²) */
    bool valid;               /**< Measurement validity flag */
} ESKF_GPS_Measurement;

/**
 * @brief Magnetometer Measurement
 */
typedef struct {
    eskf_float_t mag[3];      /**< Magnetic field [x, y, z] in Gauss or normalized */
    eskf_float_t R_mag;       /**< Measurement noise variance */
    bool valid;               /**< Measurement validity flag */
} ESKF_Mag_Measurement;

/**
 * @brief Barometer Measurement
 */
typedef struct {
    eskf_float_t height;      /**< Barometric height (up is positive) in meters */
    eskf_float_t R_baro;      /**< Measurement noise variance (m²) */
    bool valid;               /**< Measurement validity flag */
} ESKF_Baro_Measurement;

#endif /* ESKF_TYPES_H */
