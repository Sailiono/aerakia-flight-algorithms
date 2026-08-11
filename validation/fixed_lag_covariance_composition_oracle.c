/**
 * @file fixed_lag_covariance_composition_oracle.c
 * @brief Host-only variable-rate covariance-composition oracle.
 *
 * This tests the no-measurement propagation part of a future fixed-lag
 * smoother. It does not implement delayed fusion, smoothing, or a product
 * estimator feature.
 */

#include <aerakia/eskf.h>

#include "eskf_lag_covariance.h"
#include "eskf_models.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define COMPOSITION_CASES 384U
#define MINIMUM_STEPS 4U
#define MAXIMUM_STEPS 23U

#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
#define COMPOSITION_TOLERANCE ESKF_SCALAR(5.0e-4)
#else
#define COMPOSITION_TOLERANCE ESKF_SCALAR(2.0e-10)
#endif

static int failures = 0;
static uint32_t random_state = 0x5d4f2a19U;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static double uniform(double minimum, double maximum)
{
    random_state = random_state * 1664525U + 1013904223U;
    return minimum + (maximum - minimum) * ((double)random_state / 4294967295.0);
}

static unsigned random_index(unsigned exclusive_upper_bound)
{
    random_state = random_state * 1664525U + 1013904223U;
    return random_state % exclusive_upper_bound;
}

static void quaternion_from_euler(double yaw, double pitch, double roll,
                                  eskf_float_t quaternion[4])
{
    const double cr = cos(0.5 * roll);
    const double sr = sin(0.5 * roll);
    const double cp = cos(0.5 * pitch);
    const double sp = sin(0.5 * pitch);
    const double cy = cos(0.5 * yaw);
    const double sy = sin(0.5 * yaw);

    quaternion[0] = (eskf_float_t)(cr * cp * cy + sr * sp * sy);
    quaternion[1] = (eskf_float_t)(sr * cp * cy - cr * sp * sy);
    quaternion[2] = (eskf_float_t)(cr * sp * cy + sr * cp * sy);
    quaternion[3] = (eskf_float_t)(cr * cp * sy - sr * sp * cy);
}

static double error_state_scale(int index)
{
    if (index < ESKF_IDX_DV) return 0.05;
    if (index < ESKF_IDX_DP) return 0.5;
    if (index < ESKF_IDX_DAB) return 5.0;
    if (index < ESKF_IDX_DGB) return 0.1;
    return 0.01;
}

static void set_dense_psd_covariance(ESKF_Handle *filter)
{
    double lower[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM] = {{0.0}};
    int row;
    int column;
    int inner;

    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column <= row; ++column) {
            if (row == column) {
                lower[row][column] = error_state_scale(row) * uniform(0.5, 1.5);
            } else {
                lower[row][column] = error_state_scale(row) * uniform(-0.08, 0.08);
            }
        }
    }
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            double value = 0.0;
            const int upper = row < column ? row : column;
            for (inner = 0; inner <= upper; ++inner) {
                value += lower[row][inner] * lower[column][inner];
            }
            filter->P[row][column] = (eskf_float_t)value;
        }
    }
}

static void initialize_case(ESKF_Handle *filter)
{
    ESKF_Config config;
    int axis;

    eskf_init(filter, NULL, NULL);
    quaternion_from_euler(
        uniform(-ESKF_PI, ESKF_PI), uniform(-1.35, 1.35),
        uniform(-ESKF_PI, ESKF_PI), filter->state.q
    );
    for (axis = 0; axis < 3; ++axis) {
        filter->state.p[axis] = (eskf_float_t)uniform(-1000.0, 1000.0);
        filter->state.v[axis] = (eskf_float_t)uniform(-40.0, 40.0);
        filter->state.ab[axis] = (eskf_float_t)uniform(-0.3, 0.3);
        filter->state.gb[axis] = (eskf_float_t)uniform(-0.03, 0.03);
    }
    config.sigma_acc = (eskf_float_t)uniform(0.05, 0.65);
    config.sigma_gyr = (eskf_float_t)uniform(0.002, 0.05);
    config.sigma_acc_bias = (eskf_float_t)uniform(0.0003, 0.01);
    config.sigma_gyr_bias = (eskf_float_t)uniform(0.00005, 0.002);
    eskf_set_config(filter, &config);
    set_dense_psd_covariance(filter);
}

static double matrix_max_difference(
    eskf_float_t first[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM],
    eskf_float_t second[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    double maximum = 0.0;
    int row;
    int column;

    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            const double difference = fabs((double)(first[row][column] - second[row][column]));
            if (difference > maximum) maximum = difference;
        }
    }
    return maximum;
}

static int covariance_is_finite_symmetric_psd(
    eskf_float_t covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    double lower[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM] = {{0.0}};
    const double tolerance = 1.0e-9;
    int row;
    int column;
    int inner;

    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column <= row; ++column) {
            double value = (double)covariance[row][column];
            if (!isfinite(value)
                || fabs(value - (double)covariance[column][row]) > tolerance) {
                return 0;
            }
            for (inner = 0; inner < column; ++inner) {
                value -= lower[row][inner] * lower[column][inner];
            }
            if (row == column) {
                if (value < -tolerance) return 0;
                lower[row][column] = value > 0.0 ? sqrt(value) : 0.0;
            } else if (lower[column][column] > tolerance) {
                lower[row][column] = value / lower[column][column];
            } else if (fabs(value) > tolerance) {
                return 0;
            }
        }
    }
    return 1;
}

static void run_composition_campaign(void)
{
    double maximum_covariance_difference = 0.0;
    double maximum_cross_difference = 0.0;
    unsigned total_intervals = 0U;
    unsigned case_index;

    for (case_index = 0U; case_index < COMPOSITION_CASES; ++case_index) {
        ESKF_Handle filter;
        ESKF_Handle boundary;
        AerakiaValidationLagCovariance lag;
        const unsigned steps = MINIMUM_STEPS
            + random_index(MAXIMUM_STEPS - MINIMUM_STEPS + 1U);
        unsigned step;

        initialize_case(&filter);
        boundary = filter;
        aerakia_validation_lag_covariance_reset(&lag);

        for (step = 0U; step < steps; ++step) {
            eskf_float_t acceleration[3];
            eskf_float_t angular_rate[3];
            eskf_float_t corrected_acceleration[3];
            eskf_float_t corrected_angular_rate[3];
            eskf_float_t transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            eskf_float_t process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            eskf_float_t composed_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            const eskf_float_t dt = (eskf_float_t)uniform(0.001, 0.01);
            int axis;

            for (axis = 0; axis < 3; ++axis) {
                acceleration[axis] = (eskf_float_t)uniform(-25.0, 25.0);
                angular_rate[axis] = (eskf_float_t)uniform(-6.0, 6.0);
                corrected_acceleration[axis] = acceleration[axis] - filter.state.ab[axis];
                corrected_angular_rate[axis] = angular_rate[axis] - filter.state.gb[axis];
            }
            eskf_model_transition(
                filter.state.q, corrected_acceleration, corrected_angular_rate, dt, transition
            );
            eskf_model_process_noise(&filter.cfg, dt, process_covariance);
            aerakia_validation_lag_covariance_append(
                &lag, transition, process_covariance
            );
            eskf_predict(&filter, acceleration, angular_rate, dt);
            aerakia_validation_lag_covariance_apply(
                &lag, boundary.P, composed_covariance
            );
            {
                const double difference = matrix_max_difference(
                    filter.P, composed_covariance
                );
                if (difference > maximum_covariance_difference) {
                    maximum_covariance_difference = difference;
                }
            }
            check_true(
                covariance_is_finite_symmetric_psd(lag.process_covariance),
                "composed lag process covariance remains finite, symmetric, and PSD"
            );
            check_true(
                covariance_is_finite_symmetric_psd(composed_covariance),
                "composed endpoint covariance remains finite, symmetric, and PSD"
            );
            total_intervals++;
        }
        {
            eskf_float_t helper_cross[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            eskf_float_t expected_cross[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            double difference;

            aerakia_validation_lag_covariance_cross_to_boundary(
                &lag, boundary.P, helper_cross
            );
            eskf_mat15_mul_mat15(lag.transition, boundary.P, expected_cross);
            difference = matrix_max_difference(helper_cross, expected_cross);
            if (difference > maximum_cross_difference) maximum_cross_difference = difference;
        }
    }
    check_true(
        maximum_covariance_difference <= (double)COMPOSITION_TOLERANCE,
        "composed lag covariance matches repeated production prediction"
    );
    check_true(
        maximum_cross_difference <= (double)COMPOSITION_TOLERANCE,
        "lag cross covariance matches Phi times boundary covariance"
    );
    printf(
        "fixed-lag covariance composition passed: %u chains, %u intervals, max endpoint %.9g, max cross %.9g\n",
        COMPOSITION_CASES, total_intervals, maximum_covariance_difference,
        maximum_cross_difference
    );
}

int main(void)
{
    run_composition_campaign();
    if (failures != 0) {
        fprintf(stderr, "%d fixed-lag covariance composition checks failed\n", failures);
        return 1;
    }
    return 0;
}
