/**
 * @file fixed_lag_measurement_cross_covariance_oracle.c
 * @brief Host-only augmented-covariance oracle for future fixed-lag work.
 *
 * The augmented covariance contains the live 15-state ESKF error and a
 * retained 15-state boundary error.  It exercises prediction, position and
 * velocity Joseph updates, and the same attitude reset Jacobian as the core.
 * It is validation-only; it is not a smoother or a product history buffer.
 */

#include <aerakia/eskf.h>

#include "eskf_math.h"
#include "eskf_models.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AUGMENTED_DIM 30
#define CROSS_CASES 192U
#define MINIMUM_STEPS 5U
#define MAXIMUM_STEPS 17U

#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
#define CROSS_TOLERANCE ESKF_SCALAR(5.0e-3)
#define PSD_TOLERANCE 1.0e-5
#else
#define CROSS_TOLERANCE ESKF_SCALAR(2.0e-9)
#define PSD_TOLERANCE 1.0e-9
#endif

typedef struct {
    eskf_float_t value[AUGMENTED_DIM][AUGMENTED_DIM];
} AugmentedCovariance;

static int failures = 0;
static uint32_t random_state = 0x17b4a9e3U;
static double minimum_cholesky_pivot = 1.0e300;
static double maximum_symmetry_error = 0.0;
static double maximum_zero_pivot_residual = 0.0;
static double maximum_attitude_reset_norm = 0.0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        if (failures < 8) {
            fprintf(stderr, "FAIL: %s\n", message);
        } else if (failures == 8) {
            fputs("FAIL: additional failures suppressed; see final diagnostic\n", stderr);
        }
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
        filter->state.p[axis] = (eskf_float_t)uniform(-100.0, 100.0);
        filter->state.v[axis] = (eskf_float_t)uniform(-12.0, 12.0);
        filter->state.ab[axis] = (eskf_float_t)uniform(-0.15, 0.15);
        filter->state.gb[axis] = (eskf_float_t)uniform(-0.02, 0.02);
    }
    config.sigma_acc = (eskf_float_t)uniform(0.05, 0.65);
    config.sigma_gyr = (eskf_float_t)uniform(0.002, 0.05);
    config.sigma_acc_bias = (eskf_float_t)uniform(0.0003, 0.01);
    config.sigma_gyr_bias = (eskf_float_t)uniform(0.00005, 0.002);
    eskf_set_config(filter, &config);
    set_dense_psd_covariance(filter);
}

static void augmented_from_boundary(
    AugmentedCovariance *augmented,
    eskf_float_t boundary_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    int row;
    int column;

    memset(augmented, 0, sizeof(*augmented));
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            const eskf_float_t value = boundary_covariance[row][column];
            augmented->value[row][column] = value;
            augmented->value[row][ESKF_ERROR_STATE_DIM + column] = value;
            augmented->value[ESKF_ERROR_STATE_DIM + row][column] = value;
            augmented->value[ESKF_ERROR_STATE_DIM + row]
                [ESKF_ERROR_STATE_DIM + column] = value;
        }
    }
}

static void matrix30_symmetrize(
    eskf_float_t matrix[AUGMENTED_DIM][AUGMENTED_DIM]
)
{
    int row;
    int column;
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = row + 1; column < AUGMENTED_DIM; ++column) {
            const eskf_float_t average = ESKF_SCALAR(0.5) * (
                matrix[row][column] + matrix[column][row]
            );
            matrix[row][column] = average;
            matrix[column][row] = average;
        }
    }
}

static void augmented_symmetrize(AugmentedCovariance *augmented)
{
    matrix30_symmetrize(augmented->value);
}

static void augmented_propagate(
    AugmentedCovariance *augmented,
    eskf_float_t transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM],
    eskf_float_t process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM]
)
{
    eskf_float_t state_transition[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t temporary[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t next[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    int row;
    int column;
    int inner;

    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            state_transition[row][column] = transition[row][column];
        }
        state_transition[ESKF_ERROR_STATE_DIM + row]
            [ESKF_ERROR_STATE_DIM + row] = ESKF_SCALAR(1.0);
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += state_transition[row][inner] * augmented->value[inner][column];
            }
            temporary[row][column] = value;
        }
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += temporary[row][inner] * state_transition[column][inner];
            }
            if (row < ESKF_ERROR_STATE_DIM && column < ESKF_ERROR_STATE_DIM) {
                value += process_covariance[row][column];
            }
            next[row][column] = value;
        }
    }
    memcpy(augmented->value, next, sizeof(next));
    augmented_symmetrize(augmented);
}

static int augmented_update(
    AugmentedCovariance *augmented,
    eskf_float_t H[3][ESKF_ERROR_STATE_DIM],
    eskf_float_t measurement_covariance[3][3],
    eskf_float_t residual[3]
)
{
    eskf_float_t PHt[AUGMENTED_DIM][3] = {{0.0}};
    eskf_float_t innovation_covariance[3][3] = {{0.0}};
    eskf_float_t innovation_inverse[3][3];
    eskf_float_t gain[AUGMENTED_DIM][3] = {{0.0}};
    eskf_float_t dx[ESKF_ERROR_STATE_DIM] = {0.0};
    eskf_float_t identity_minus_KH[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t temporary[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t joseph[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t reset[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t reset_temporary[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t next[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    eskf_float_t attitude_error[3];
    eskf_float_t skew[3][3];
    int row;
    int column;
    int inner;

    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < 3; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < ESKF_ERROR_STATE_DIM; ++inner) {
                value += augmented->value[row][inner] * H[column][inner];
            }
            PHt[row][column] = value;
        }
    }
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            eskf_float_t value = measurement_covariance[row][column];
            for (inner = 0; inner < ESKF_ERROR_STATE_DIM; ++inner) {
                value += H[row][inner] * PHt[inner][column];
            }
            innovation_covariance[row][column] = value;
        }
    }
    if (!eskf_mat3_inv(innovation_covariance, innovation_inverse)) return 0;
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < 3; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < 3; ++inner) {
                value += PHt[row][inner] * innovation_inverse[inner][column];
            }
            gain[row][column] = value;
        }
    }
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < 3; ++column) {
            dx[row] += gain[row][column] * residual[column];
        }
    }
    {
        const double attitude_reset_norm = sqrt(
            (double)dx[ESKF_IDX_DTHETA + 0] * dx[ESKF_IDX_DTHETA + 0]
            + (double)dx[ESKF_IDX_DTHETA + 1] * dx[ESKF_IDX_DTHETA + 1]
            + (double)dx[ESKF_IDX_DTHETA + 2] * dx[ESKF_IDX_DTHETA + 2]
        );
        if (attitude_reset_norm > maximum_attitude_reset_norm) {
            maximum_attitude_reset_norm = attitude_reset_norm;
        }
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < 3; ++inner) {
                value += gain[row][inner] * (
                    column < ESKF_ERROR_STATE_DIM ? H[inner][column]
                        : ESKF_SCALAR(0.0)
                );
            }
            identity_minus_KH[row][column] = (
                row == column ? ESKF_SCALAR(1.0) : ESKF_SCALAR(0.0)
            ) - value;
        }
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += identity_minus_KH[row][inner] * augmented->value[inner][column];
            }
            temporary[row][column] = value;
        }
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            int measurement_row;
            int measurement_column;
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += temporary[row][inner] * identity_minus_KH[column][inner];
            }
            for (measurement_row = 0; measurement_row < 3; ++measurement_row) {
                for (measurement_column = 0; measurement_column < 3; ++measurement_column) {
                    value += gain[row][measurement_row]
                        * measurement_covariance[measurement_row][measurement_column]
                        * gain[column][measurement_column];
                }
            }
            joseph[row][column] = value;
        }
    }
    matrix30_symmetrize(joseph);
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            reset[row][column] = row == column ? ESKF_SCALAR(1.0) : ESKF_SCALAR(0.0);
        }
    }
    attitude_error[0] = dx[ESKF_IDX_DTHETA + 0];
    attitude_error[1] = dx[ESKF_IDX_DTHETA + 1];
    attitude_error[2] = dx[ESKF_IDX_DTHETA + 2];
    eskf_mat3_skew(attitude_error, skew);
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            reset[row][column] -= ESKF_SCALAR(0.5) * skew[row][column];
        }
    }
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        reset[ESKF_ERROR_STATE_DIM + row][ESKF_ERROR_STATE_DIM + row]
            = ESKF_SCALAR(1.0);
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += reset[row][inner] * joseph[inner][column];
            }
            reset_temporary[row][column] = value;
        }
    }
    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column < AUGMENTED_DIM; ++column) {
            eskf_float_t value = ESKF_SCALAR(0.0);
            for (inner = 0; inner < AUGMENTED_DIM; ++inner) {
                value += reset_temporary[row][inner] * reset[column][inner];
            }
            next[row][column] = value;
        }
    }
    memcpy(augmented->value, next, sizeof(next));
    augmented_symmetrize(augmented);
    return 1;
}

static double max_top_left_difference(
    const ESKF_Handle *filter,
    const AugmentedCovariance *augmented
)
{
    double maximum = 0.0;
    int row;
    int column;
    for (row = 0; row < ESKF_ERROR_STATE_DIM; ++row) {
        for (column = 0; column < ESKF_ERROR_STATE_DIM; ++column) {
            const double difference = fabs(
                (double)(filter->P[row][column] - augmented->value[row][column])
            );
            if (difference > maximum) maximum = difference;
        }
    }
    return maximum;
}

static int augmented_is_finite_symmetric_psd(const AugmentedCovariance *augmented)
{
    double lower[AUGMENTED_DIM][AUGMENTED_DIM] = {{0.0}};
    int row;
    int column;
    int inner;

    for (row = 0; row < AUGMENTED_DIM; ++row) {
        for (column = 0; column <= row; ++column) {
            double value = (double)augmented->value[row][column];
            const double symmetry_error = fabs(
                value - (double)augmented->value[column][row]
            );
            if (symmetry_error > maximum_symmetry_error) {
                maximum_symmetry_error = symmetry_error;
            }
            if (!isfinite(value) || symmetry_error > PSD_TOLERANCE) {
                return 0;
            }
            for (inner = 0; inner < column; ++inner) {
                value -= lower[row][inner] * lower[column][inner];
            }
            if (row == column) {
                if (value < minimum_cholesky_pivot) minimum_cholesky_pivot = value;
                if (value < -PSD_TOLERANCE) return 0;
                lower[row][column] = value > 0.0 ? sqrt(value) : 0.0;
            } else if (lower[column][column] > PSD_TOLERANCE) {
                lower[row][column] = value / lower[column][column];
            } else if (fabs(value) > PSD_TOLERANCE) {
                if (fabs(value) > maximum_zero_pivot_residual) {
                    maximum_zero_pivot_residual = fabs(value);
                }
                return 0;
            }
        }
    }
    return 1;
}

static void build_measurement(
    unsigned kind,
    const ESKF_Handle *filter,
    eskf_float_t H[3][ESKF_ERROR_STATE_DIM],
    eskf_float_t residual[3],
    eskf_float_t measurement[3]
)
{
    int axis;
    memset(H, 0, sizeof(eskf_float_t) * 3 * ESKF_ERROR_STATE_DIM);
    for (axis = 0; axis < 3; ++axis) {
        H[axis][kind == 0U ? ESKF_IDX_DP + axis : ESKF_IDX_DV + axis]
            = ESKF_SCALAR(1.0);
        residual[axis] = (eskf_float_t)uniform(-0.5, 0.5);
        measurement[axis] = (kind == 0U ? filter->state.p[axis] : filter->state.v[axis])
            + residual[axis];
    }
}

static void run_campaign(void)
{
    double maximum_prediction_difference = 0.0;
    double maximum_update_difference = 0.0;
    unsigned total_intervals = 0U;
    unsigned total_updates = 0U;
    unsigned case_index;

    for (case_index = 0U; case_index < CROSS_CASES; ++case_index) {
        ESKF_Handle filter;
        AugmentedCovariance augmented;
        const unsigned steps = MINIMUM_STEPS
            + random_index(MAXIMUM_STEPS - MINIMUM_STEPS + 1U);
        unsigned step;

        initialize_case(&filter);
        augmented_from_boundary(&augmented, filter.P);
        for (step = 0U; step < steps; ++step) {
            eskf_float_t acceleration[3];
            eskf_float_t angular_rate[3];
            eskf_float_t corrected_acceleration[3];
            eskf_float_t corrected_angular_rate[3];
            eskf_float_t transition[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            eskf_float_t process_covariance[ESKF_ERROR_STATE_DIM][ESKF_ERROR_STATE_DIM];
            const eskf_float_t dt = (eskf_float_t)uniform(0.001, 0.01);
            int axis;

            for (axis = 0; axis < 3; ++axis) {
                acceleration[axis] = (eskf_float_t)uniform(-18.0, 18.0);
                angular_rate[axis] = (eskf_float_t)uniform(-4.0, 4.0);
                corrected_acceleration[axis] = acceleration[axis] - filter.state.ab[axis];
                corrected_angular_rate[axis] = angular_rate[axis] - filter.state.gb[axis];
            }
            eskf_model_transition(
                filter.state.q, corrected_acceleration, corrected_angular_rate, dt, transition
            );
            eskf_model_process_noise(&filter.cfg, dt, process_covariance);
            augmented_propagate(&augmented, transition, process_covariance);
            eskf_predict(&filter, acceleration, angular_rate, dt);
            {
                const double difference = max_top_left_difference(&filter, &augmented);
                if (difference > maximum_prediction_difference) {
                    maximum_prediction_difference = difference;
                }
            }
            check_true(
                augmented_is_finite_symmetric_psd(&augmented),
                "augmented covariance remains finite, symmetric, and PSD after prediction"
            );
            total_intervals++;

            if ((step % 2U) == 0U) {
                eskf_float_t H[3][ESKF_ERROR_STATE_DIM];
                eskf_float_t residual[3];
                eskf_float_t measurement[3];
                eskf_float_t measurement_covariance[3][3] = {
                    {ESKF_SCALAR(25.0), ESKF_SCALAR(0.0), ESKF_SCALAR(0.0)},
                    {ESKF_SCALAR(0.0), ESKF_SCALAR(25.0), ESKF_SCALAR(0.0)},
                    {ESKF_SCALAR(0.0), ESKF_SCALAR(0.0), ESKF_SCALAR(25.0)}
                };
                ESKF_InnovResult result = {0};
                const unsigned kind = (step / 2U) % 2U;

                build_measurement(kind, &filter, H, residual, measurement);
                check_true(
                    augmented_update(&augmented, H, measurement_covariance, residual),
                    "augmented P/V update has invertible innovation covariance"
                );
                if (kind == 0U) {
                    eskf_update_position(
                        &filter, measurement, ESKF_SCALAR(25.0), &result
                    );
                } else {
                    eskf_update_velocity(
                        &filter, measurement, ESKF_SCALAR(25.0), &result
                    );
                }
                check_true(result.accepted, "production P/V update remains accepted");
                {
                    const double difference = max_top_left_difference(&filter, &augmented);
                    if (difference > maximum_update_difference) {
                        maximum_update_difference = difference;
                    }
                }
                check_true(
                    augmented_is_finite_symmetric_psd(&augmented),
                    "augmented covariance remains finite, symmetric, and PSD after update"
                );
                total_updates++;
            }
        }
    }
    check_true(
        maximum_prediction_difference <= (double)CROSS_TOLERANCE,
        "augmented prediction top-left matches production covariance"
    );
    check_true(
        maximum_update_difference <= (double)CROSS_TOLERANCE,
        "augmented Joseph/reset update top-left matches production covariance"
    );
    check_true(
        maximum_attitude_reset_norm > 1.0e-8,
        "nonzero attitude reset is exercised by the P/V covariance campaign"
    );
    printf(
        "fixed-lag measurement cross-covariance %s: %u chains, %u intervals, %u P/V updates, max prediction %.9g, max update %.9g, max reset %.9g rad\n",
        failures == 0 ? "passed" : "failed", CROSS_CASES, total_intervals,
        total_updates, maximum_prediction_difference, maximum_update_difference,
        maximum_attitude_reset_norm
    );
    if (failures != 0) {
        fprintf(
            stderr,
            "PSD diagnostic: minimum Cholesky pivot %.9g, maximum symmetry %.9g, "
            "maximum zero-pivot residual %.9g\n",
            minimum_cholesky_pivot, maximum_symmetry_error,
            maximum_zero_pivot_residual
        );
    }
}

int main(void)
{
    run_campaign();
    if (failures != 0) {
        fprintf(stderr, "%d fixed-lag measurement cross-covariance checks failed\n", failures);
        return 1;
    }
    return 0;
}
