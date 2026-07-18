#include <aerakia/eskf.h>

#include "../validation/eskf_joint_covariance.h"

#include <math.h>
#include <stdio.h>

int main(void)
{
    ESKF_Handle filter;
    eskf_float_t extracted[AERAKIA_TILT_ACCEL_BIAS_DIM][AERAKIA_TILT_ACCEL_BIAS_DIM];
    const int source_index[AERAKIA_TILT_ACCEL_BIAS_DIM] = {
        ESKF_IDX_DTHETA + 0,
        ESKF_IDX_DTHETA + 1,
        ESKF_IDX_DAB + 0,
        ESKF_IDX_DAB + 1,
        ESKF_IDX_DAB + 2
    };
    int row;
    int column;

    eskf_init(&filter, NULL, NULL);
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            filter.P[row][column] = 1000.0 + 10.0 * row + column;
        }
    }
    aerakia_validation_extract_tilt_accel_bias_covariance(&filter, extracted);

    for (row = 0; row < AERAKIA_TILT_ACCEL_BIAS_DIM; ++row) {
        for (column = 0; column < AERAKIA_TILT_ACCEL_BIAS_DIM; ++column) {
            const double expected = filter.P[source_index[row]][source_index[column]];
            if (!isfinite(extracted[row][column])
                    || fabs(extracted[row][column] - expected) > 1.0e-15) {
                fprintf(stderr, "joint covariance extraction mismatch at %d,%d\n", row, column);
                return 1;
            }
        }
    }
    if (extracted[0][0] == filter.P[ESKF_IDX_DTHETA + 2][ESKF_IDX_DTHETA + 2]) {
        fputs("yaw covariance leaked into observable tilt marginal\n", stderr);
        return 1;
    }
    puts("validation covariance extraction passed");
    return 0;
}
