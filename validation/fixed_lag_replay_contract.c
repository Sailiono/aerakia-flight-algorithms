/**
 * @file fixed_lag_replay_contract.c
 * @brief Host-only contract for the complete ESKF correction transaction.
 */

#include <aerakia/eskf.h>

#include "eskf_internal.h"
#include "eskf_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static int nearly_equal(eskf_float_t first, eskf_float_t second, eskf_float_t tolerance)
{
    return fabs((double)(first - second)) <= (double)tolerance;
}

static void test_zero_error_is_exact_noop(void)
{
    ESKF_Handle handle;
    ESKF_Handle before;
    eskf_float_t zero[ESKF_ERROR_STATE_DIM] = {0};
    eskf_init(&handle, NULL, NULL);
    handle.state.p[0] = ESKF_SCALAR(3.0);
    handle.state.v[1] = ESKF_SCALAR(-0.25);
    handle.state.ab[2] = ESKF_SCALAR(0.03);
    before = handle;
    eskf_internal_apply_error_state(&handle, zero);
    check_true(memcmp(&before.state, &handle.state, sizeof(handle.state)) == 0,
               "zero error preserves nominal state exactly");
    check_true(memcmp(before.P, handle.P, sizeof(handle.P)) == 0,
               "zero error preserves covariance exactly");
}

static void test_full_error_changes_all_nominal_blocks_and_keeps_covariance_symmetric(void)
{
    ESKF_Handle handle;
    eskf_float_t error[ESKF_ERROR_STATE_DIM] = {0};
    int row;
    int column;
    eskf_init(&handle, NULL, NULL);
    error[ESKF_IDX_DTHETA + 0] = ESKF_SCALAR(0.02);
    error[ESKF_IDX_DTHETA + 1] = ESKF_SCALAR(-0.01);
    error[ESKF_IDX_DV + 0] = ESKF_SCALAR(0.4);
    error[ESKF_IDX_DP + 1] = ESKF_SCALAR(-1.5);
    error[ESKF_IDX_DAB + 2] = ESKF_SCALAR(0.08);
    error[ESKF_IDX_DGB + 1] = ESKF_SCALAR(-0.004);
    eskf_internal_apply_error_state(&handle, error);
    check_true(nearly_equal(handle.state.v[0], ESKF_SCALAR(0.4), ESKF_SCALAR(1e-12)),
               "error transaction injects velocity");
    check_true(nearly_equal(handle.state.p[1], ESKF_SCALAR(-1.5), ESKF_SCALAR(1e-12)),
               "error transaction injects position");
    check_true(nearly_equal(handle.state.ab[2], ESKF_SCALAR(0.08), ESKF_SCALAR(1e-12)),
               "error transaction injects accelerometer bias");
    check_true(nearly_equal(handle.state.gb[1], ESKF_SCALAR(-0.004), ESKF_SCALAR(1e-12)),
               "error transaction injects gyroscope bias");
    check_true(fabs((double)(handle.state.q[0] - ESKF_SCALAR(1.0))) > 1e-6,
               "error transaction injects right attitude error");
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            check_true(isfinite(handle.P[row][column]), "reset covariance remains finite");
            check_true(nearly_equal(handle.P[row][column], handle.P[column][row], ESKF_SCALAR(1e-12)),
                       "reset covariance remains symmetric");
        }
    }
}

int main(void)
{
    test_zero_error_is_exact_noop();
    test_full_error_changes_all_nominal_blocks_and_keeps_covariance_symmetric();
    if (failures != 0) {
        fprintf(stderr, "%d fixed-lag replay contract checks failed\n", failures);
        return 1;
    }
    puts("fixed-lag replay contract checks passed");
    return 0;
}
