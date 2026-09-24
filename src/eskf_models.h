/**
 * @file eskf_models.h
 * @brief Internal ESKF models shared by the implementation and numerical tests.
 */

#ifndef AERAKIA_ESKF_MODELS_H
#define AERAKIA_ESKF_MODELS_H

#include <stdbool.h>

#include <aerakia/eskf_types.h>

/** Discrete right-error transition with exact SO(3) attitude and position dt-squared coupling. */
void eskf_model_transition(const eskf_float_t q[4],
                           const eskf_float_t acceleration_body[3],
                           const eskf_float_t angular_rate_body[3],
                           eskf_float_t dt,
                           eskf_float_t F[15][15]);

/**
 * High-rate reduced-model process-noise mapping for continuous densities.
 * Includes direct IMU/bias noise and acceleration velocity-position
 * integration; higher-order within-step F/bias couplings are omitted.
 */
void eskf_model_process_noise(const ESKF_Config *config,
                              eskf_float_t dt,
                              eskf_float_t Q[15][15]);

/** Body-X NED heading and its complete right-error atan2 Jacobian. */
bool eskf_model_heading(const eskf_float_t q[4],
                        eskf_float_t *heading_rad,
                        eskf_float_t H_theta[3]);

/**
 * Robust magnetic yaw-only correction in the local NED-yaw error coordinate.
 *
 * This deliberately conditions on an already converged tilt estimate. It is a
 * local error-space pseudo observation, not the complete derivative of raw
 * magnetic heading with respect to arbitrary attitude errors. The variance is
 * therefore a reviewed yaw-correction tuning variance, and the resulting NIS
 * must not be interpreted as a general physical-heading consistency statistic.
 */
bool eskf_model_magnetic_yaw_correction(const eskf_float_t q[4],
                                        const eskf_float_t mag_body[3],
                                        const eskf_float_t mag_reference_ned[3],
                                        eskf_float_t *residual_rad,
                                        eskf_float_t H_theta[3]);

#endif
