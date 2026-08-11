/**
 * @file eskf_lag_covariance.h
 * @brief Validation-only composition of ESKF transition and process covariance.
 *
 * A fixed-lag smoother needs the full transition Phi(k, 0) and process
 * covariance Q(k, 0) across its retained IMU interval. This helper verifies
 * that composition against the production per-sample propagation. It is not a
 * public API, a product history buffer, or an out-of-sequence fusion feature.
 */

#ifndef AERAKIA_VALIDATION_ESKF_LAG_COVARIANCE_H
#define AERAKIA_VALIDATION_ESKF_LAG_COVARIANCE_H

#include <aerakia/eskf.h>

#include <stddef.h>

#include "eskf_math.h"

typedef struct {
    eskf_float_t transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
    eskf_float_t process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
} AerakiaValidationLagCovariance;

static inline void aerakia_validation_lag_covariance_reset(
    AerakiaValidationLagCovariance *lag
)
{
    if (lag == NULL) return;
    eskf_mat15_identity(lag->transition);
    eskf_mat15_zero(lag->process_covariance);
}

/**
 * Append one discrete ESKF interval:
 * Phi_new = F * Phi_old
 * Q_new = F * Q_old * F^T + Q_interval.
 */
static inline void aerakia_validation_lag_covariance_append(
    AerakiaValidationLagCovariance *lag,
    eskf_float_t transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM],
    eskf_float_t process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    eskf_float_t next_transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
    eskf_float_t next_process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];

    if (lag == NULL || transition == NULL || process_covariance == NULL) return;
    eskf_mat15_mul_mat15(transition, lag->transition, next_transition);
    eskf_mat15_copy(lag->process_covariance, next_process_covariance);
    eskf_mat15_propagate(next_process_covariance, transition, process_covariance);
    eskf_mat15_symmetrize(next_process_covariance);
    eskf_mat15_copy(next_transition, lag->transition);
    eskf_mat15_copy(next_process_covariance, lag->process_covariance);
}

/** Apply the composed lag model to a covariance at the retained boundary. */
static inline void aerakia_validation_lag_covariance_apply(
    AerakiaValidationLagCovariance *lag,
    eskf_float_t boundary_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM],
    eskf_float_t output_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    if (lag == NULL || boundary_covariance == NULL || output_covariance == NULL) return;
    eskf_mat15_copy(boundary_covariance, output_covariance);
    eskf_mat15_propagate(
        output_covariance, lag->transition, lag->process_covariance
    );
    eskf_mat15_symmetrize(output_covariance);
}

/**
 * Return Cov(dx_k, dx_0) = Phi(k, 0) P_0 for a future smoother test.
 */
static inline void aerakia_validation_lag_covariance_cross_to_boundary(
    AerakiaValidationLagCovariance *lag,
    eskf_float_t boundary_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM],
    eskf_float_t cross_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    if (lag == NULL || boundary_covariance == NULL || cross_covariance == NULL) return;
    eskf_mat15_mul_mat15(lag->transition, boundary_covariance, cross_covariance);
}

#endif /* AERAKIA_VALIDATION_ESKF_LAG_COVARIANCE_H */
