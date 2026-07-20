/**
 * @file eskf.h
 * @brief Error-State Kalman Filter - Public API
 *
 * This is the main interface for the ESKF library.
 * All functions are stateless - caller must provide the ESKF_Handle.
 *
 * Usage:
 *   1. Allocate ESKF_Handle (static or stack)
 *   2. Call eskf_init() once
 *   3. For each IMU sample:
 *      a. Call eskf_predict()
 *      b. Optionally call eskf_update_*() when measurements arrive
 *   4. Read state from handle->state
 *
 * Maintained by Aerakia contributors.
 */

#ifndef ESKF_H
#define ESKF_H

#include <aerakia/eskf_types.h>

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize the ESKF filter
 *
 * Sets up initial state, covariance, and configuration.
 * Must be called before any other ESKF function.
 *
 * @param h         Pointer to filter handle (must be allocated by caller)
 * @param init_pos  Initial position [N, E, D] in meters (can be NULL for origin)
 * @param init_q    Initial quaternion [w, x, y, z] (can be NULL for identity)
 */
void eskf_init(ESKF_Handle *h,
               const eskf_float_t init_pos[3],
               const eskf_float_t init_q[4]);

/**
 * @brief Set filter configuration (noise parameters)
 *
 * @param h       Pointer to filter handle
 * @param cfg     Configuration structure with noise parameters
 */
void eskf_set_config(ESKF_Handle *h, const ESKF_Config *cfg);

/**
 * @brief Set magnetic field reference vector
 *
 * @param h         Pointer to filter handle
 * @param mag_ref   Magnetic field reference in NED frame. It must be finite and
 *                  retain a non-zero horizontal North/East component.
 * @return true when the reference was accepted; false leaves the prior
 *         reference unchanged.
 */
bool eskf_set_mag_reference(ESKF_Handle *h, const eskf_float_t mag_ref[3]);

/* ============================================================================
 * Prediction Step
 * ============================================================================ */

/**
 * @brief ESKF Prediction Step (IMU Integration)
 *
 * Integrates IMU measurements to propagate nominal state and covariance.
 * This is the time update step of the Kalman filter.
 *
 * Algorithm (Solà formulation):
 *   1. De-bias IMU inputs
 *   2. Integrate nominal state (position, velocity, quaternion)
 *   3. Compute state transition Jacobian F
 *   4. Propagate covariance: P = F*P*F' + Q
 *
 * @param h       Pointer to filter handle
 * @param acc_m   Measured acceleration [x, y, z] in body frame (m/s²)
 * @param gyr_m   Measured angular rate [x, y, z] in body frame (rad/s)
 * @param dt      Time step in seconds
 */
void eskf_predict(ESKF_Handle *h,
                  const eskf_float_t acc_m[3],
                  const eskf_float_t gyr_m[3],
                  eskf_float_t dt);

/* ============================================================================
 * Measurement Update Steps
 * ============================================================================ */

/**
 * @brief GPS Position Update
 *
 * Corrects position estimate using GPS measurement.
 * Observation model: z = p_gps - p_est
 *
 * @param h       Pointer to filter handle
 * @param pos_m   Measured position [N, E, D] in meters
 * @param R_pos   Position measurement noise variance (m²)
 * @param result  Output: Innovation test result (can be NULL if not needed)
 */
void eskf_update_position(ESKF_Handle *h,
                          const eskf_float_t pos_m[3],
                          eskf_float_t R_pos,
                          ESKF_InnovResult *result);

/** Correct velocity from a NED-frame observation. */
void eskf_update_velocity(ESKF_Handle *h,
                          const eskf_float_t velocity_m_s[3],
                          eskf_float_t R_velocity,
                          ESKF_InnovResult *result);

/**
 * @brief Magnetometer Update (Yaw Correction)
 *
 * Corrects yaw using a tilt-conditioned local NED-yaw pseudo observation.
 * This deliberately prevents magnetic inclination/model errors from updating
 * roll and pitch. R_mag is a reviewed yaw-correction tuning variance; the
 * returned NIS is a pseudo-innovation diagnostic and is not a general
 * physical-heading consistency statistic at arbitrary tilt.
 *
 * @param h       Pointer to filter handle
 * @param mag_m   Measured magnetic field [x, y, z] (normalized or Gauss)
 * @param R_mag   Local yaw-correction tuning variance (rad²)
 * @param result  Output: Innovation test result (can be NULL if not needed)
 */
void eskf_update_mag(ESKF_Handle *h,
                     const eskf_float_t mag_m[3],
                     eskf_float_t R_mag,
                     ESKF_InnovResult *result);

/**
 * Correct yaw from a trusted navigation-frame heading observation.
 * heading_ned_rad is clockwise from North in the NED convention.
 * The update uses the complete body-X atan2 heading Jacobian; at large tilt it
 * can legitimately update correlated attitude components. Reject geometrically
 * ill-conditioned observations before fusion.
 */
void eskf_update_heading(ESKF_Handle *h,
                         eskf_float_t heading_ned_rad,
                         eskf_float_t R_heading,
                         ESKF_InnovResult *result);

/**
 * @brief Barometer Update (Height Correction)
 *
 * Corrects Down position using barometric height measurement.
 * Note: baro_height is positive-up, state.p[2] is positive-down.
 *
 * @param h              Pointer to filter handle
 * @param baro_height_m  Measured height (up is positive) in meters
 * @param R_baro         Barometer measurement noise variance (m²)
 * @param result         Output: Innovation test result (can be NULL if not needed)
 */
void eskf_update_baro(ESKF_Handle *h,
                      eskf_float_t baro_height_m,
                      eskf_float_t R_baro,
                      ESKF_InnovResult *result);

/**
 * @brief Zero Velocity Update (ZUPT)
 *
 * Applies velocity constraint when vehicle is known to be stationary.
 * Strongly constrains velocity to [0, 0, 0].
 *
 * @param h       Pointer to filter handle
 * @param R_zupt  ZUPT measurement noise variance (small = strong constraint)
 */
void eskf_update_static_constraint(ESKF_Handle *h, eskf_float_t R_zupt);

/**
 * Re-anchor only position and velocity after persistent navigation rejection.
 * Attitude and learned IMU biases are deliberately preserved.
 */
void eskf_reset_navigation(ESKF_Handle *h,
                           const eskf_float_t position_ned_m[3],
                           const eskf_float_t velocity_ned_m_s[3],
                           eskf_float_t position_variance_m2,
                           eskf_float_t velocity_variance_m2_s2);

/** Re-anchor position without changing velocity, attitude, or learned IMU biases. */
void eskf_reset_position(ESKF_Handle *h,
                         const eskf_float_t position_ned_m[3],
                         eskf_float_t position_variance_m2);

/** Re-anchor velocity without changing position, attitude, or learned IMU biases. */
void eskf_reset_velocity(ESKF_Handle *h,
                         const eskf_float_t velocity_ned_m_s[3],
                         eskf_float_t velocity_variance_m2_s2);

/* ============================================================================
 * Calibration / Alignment
 * ============================================================================ */

/**
 * Coarsely align roll and pitch from a stationary specific-force mean.
 * The current navigation-frame yaw is preserved.
 *
 * @return true when the acceleration vector was valid and alignment was applied.
 */
bool eskf_align_static_tilt(
    ESKF_Handle *h,
    const eskf_float_t acceleration_mean_m_s2[3]
);

/**
 * Coarsely align yaw from a stationary magnetic-field mean and configured
 * navigation-frame magnetic reference. Existing roll and pitch are preserved.
 *
 * @return true when both horizontal magnetic vectors were observable.
 */
bool eskf_align_static_heading(
    ESKF_Handle *h,
    const eskf_float_t magnetic_mean[3]
);

/**
 * Replace the three attitude-error variances after an absolute attitude seed.
 * Existing attitude cross-covariances are cleared because they describe the
 * pre-alignment linearization point.
 */
bool eskf_reset_attitude_covariance(
    ESKF_Handle *h,
    const eskf_float_t attitude_variance_rad2[3]
);

/**
 * @brief Static Bias Alignment
 *
 * Initialize biases from stationary IMU data.
 * Call this with data collected while vehicle is stationary. Gyroscope bias is
 * directly observable. A single gravity direction does not fully distinguish
 * accelerometer bias from tilt, so accelerometer-bias covariance deliberately
 * remains broad for later motion-aided convergence.
 *
 * @param h         Pointer to filter handle
 * @param acc_buf   Buffer of accelerometer readings [n_samples x 3]
 * @param gyr_buf   Buffer of gyroscope readings [n_samples x 3]
 * @param n_samples Number of samples in buffers
 */
void eskf_align_static_biases(ESKF_Handle *h,
                               const eskf_float_t (*acc_buf)[3],
                               const eskf_float_t (*gyr_buf)[3],
                               int n_samples);

/** Align biases from already-computed stationary IMU means. */
void eskf_align_static_bias_means(ESKF_Handle *h,
                                  const eskf_float_t acceleration_mean_m_s2[3],
                                  const eskf_float_t angular_rate_mean_rad_s[3]);

/* ============================================================================
 * State Access
 * ============================================================================ */

/**
 * @brief Get current state estimate
 *
 * Copies state values to caller-provided buffers.
 * Any pointer can be NULL if that value is not needed.
 *
 * @param h   Pointer to filter handle
 * @param p   Output: Position [N, E, D] (can be NULL)
 * @param v   Output: Velocity [N, E, D] (can be NULL)
 * @param q   Output: Quaternion [w, x, y, z] (can be NULL)
 * @param ab  Output: Accel bias [x, y, z] (can be NULL)
 * @param gb  Output: Gyro bias [x, y, z] (can be NULL)
 */
void eskf_get_state(const ESKF_Handle *h,
                    eskf_float_t *p,
                    eskf_float_t *v,
                    eskf_float_t *q,
                    eskf_float_t *ab,
                    eskf_float_t *gb);

/**
 * @brief Get diagonal elements of covariance matrix
 *
 * Useful for monitoring filter health.
 *
 * @param h       Pointer to filter handle
 * @param P_diag  Output: 15-element array with diagonal of P
 */
void eskf_get_covariance_diag(const ESKF_Handle *h, eskf_float_t P_diag[15]);

#endif /* ESKF_H */
