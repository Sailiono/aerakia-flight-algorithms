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

/** Discrete process-noise mapping for continuous white-noise densities. */
void eskf_model_process_noise(const ESKF_Config *config,
                              eskf_float_t dt,
                              eskf_float_t Q[15][15]);

/** Heading of body X and the right-error direction for a pure NED-yaw correction. */
bool eskf_model_heading(const eskf_float_t q[4],
                        eskf_float_t *heading_rad,
                        eskf_float_t H_theta[3]);

/** Magnetic-heading residual and right-error direction for a pure NED-yaw correction. */
bool eskf_model_magnetic_heading(const eskf_float_t q[4],
                                 const eskf_float_t mag_body[3],
                                 const eskf_float_t mag_reference_ned[3],
                                 eskf_float_t *residual_rad,
                                 eskf_float_t H_theta[3]);

#endif
