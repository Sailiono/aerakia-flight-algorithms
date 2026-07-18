/**
 * @file eskf_joint_covariance.h
 * @brief Validation-only extraction of the observable tilt/accelerometer-bias covariance.
 */

#ifndef AERAKIA_VALIDATION_ESKF_JOINT_COVARIANCE_H
#define AERAKIA_VALIDATION_ESKF_JOINT_COVARIANCE_H

#include <aerakia/eskf.h>

#define AERAKIA_TILT_ACCEL_BIAS_DIM 5

/**
 * Extract P for [right-error dtheta_x, dtheta_y, dab_x, dab_y, dab_z].
 *
 * Yaw error is intentionally omitted: the no-trusted-heading cold-start cases used by the bias
 * campaign do not make it observable. The returned matrix is the exact marginal submatrix of P;
 * it is not a conditional covariance and it preserves all tilt/bias cross terms.
 */
static inline void aerakia_validation_extract_tilt_accel_bias_covariance(
    const ESKF_Handle *filter,
    eskf_float_t covariance[AERAKIA_TILT_ACCEL_BIAS_DIM][AERAKIA_TILT_ACCEL_BIAS_DIM]
)
{
    const int source_index[AERAKIA_TILT_ACCEL_BIAS_DIM] = {
        ESKF_IDX_DTHETA + 0,
        ESKF_IDX_DTHETA + 1,
        ESKF_IDX_DAB + 0,
        ESKF_IDX_DAB + 1,
        ESKF_IDX_DAB + 2
    };
    int row;
    int column;

    for (row = 0; row < AERAKIA_TILT_ACCEL_BIAS_DIM; ++row) {
        for (column = 0; column < AERAKIA_TILT_ACCEL_BIAS_DIM; ++column) {
            covariance[row][column] = filter->P[source_index[row]][source_index[column]];
        }
    }
}

#endif
