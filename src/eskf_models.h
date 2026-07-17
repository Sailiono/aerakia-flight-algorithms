/**
 * @file eskf_models.h
 * @brief Internal measurement models shared by the ESKF and numerical tests.
 */

#ifndef AERAKIA_ESKF_MODELS_H
#define AERAKIA_ESKF_MODELS_H

#include <stdbool.h>

#include <aerakia/eskf_types.h>

/** First-order right-error transition, including position's dt-squared coupling. */
void eskf_model_transition(const eskf_float_t q[4],
                           const eskf_float_t acceleration_body[3],
                           const eskf_float_t angular_rate_body[3],
                           eskf_float_t dt,
                           eskf_float_t F[15][15]);

/** Heading of the body X axis and the right-error axis for a pure NED-yaw correction. */
bool eskf_model_heading(const eskf_float_t q[4],
                        eskf_float_t *heading_rad,
                        eskf_float_t H_theta[3]);

/** Magnetic-heading residual and the right-error axis for a pure NED-yaw correction. */
bool eskf_model_magnetic_heading(const eskf_float_t q[4],
                                 const eskf_float_t mag_body[3],
                                 const eskf_float_t mag_reference_ned[3],
                                 eskf_float_t *residual_rad,
                                 eskf_float_t H_theta[3]);

#endif
