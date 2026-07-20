#include "eskf_models.h"

#include <aerakia/eskf.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define JACOBIAN_RANDOM_CASES 10000U

static int failures = 0;
static uint32_t random_state = 0x8d12e43bU;

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

static double wrap_pi(double angle)
{
    return atan2(sin(angle), cos(angle));
}

static void quat_multiply(const double p[4], const double q[4], double out[4])
{
    out[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    out[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
    out[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
    out[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

static void rotation_vector_quaternion(const double vector[3], double out[4])
{
    const double angle = sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
    );
    const double scale = angle > 1.0e-15 ? sin(0.5 * angle) / angle : 0.5;
    out[0] = cos(0.5 * angle);
    out[1] = scale * vector[0];
    out[2] = scale * vector[1];
    out[3] = scale * vector[2];
}

static void euler_quaternion(double yaw, double pitch, double roll, double out[4])
{
    const double qz[4] = {cos(0.5 * yaw), 0.0, 0.0, sin(0.5 * yaw)};
    const double qy[4] = {cos(0.5 * pitch), 0.0, sin(0.5 * pitch), 0.0};
    const double qx[4] = {cos(0.5 * roll), sin(0.5 * roll), 0.0, 0.0};
    double temporary[4];
    quat_multiply(qz, qy, temporary);
    quat_multiply(temporary, qx, out);
}

static void rotate_transpose(const double q[4], const double vector[3], double out[3])
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double R[3][3] = {
        {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)},
        {2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
        {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)}
    };
    int row;
    for (row = 0; row < 3; ++row) {
        out[row] = R[0][row] * vector[0] + R[1][row] * vector[1]
            + R[2][row] * vector[2];
    }
}

static void error_between_handles(const ESKF_Handle *nominal,
                                  const ESKF_Handle *perturbed,
                                  double error[15])
{
    const double q_conjugate[4] = {
        nominal->state.q[0], -nominal->state.q[1],
        -nominal->state.q[2], -nominal->state.q[3]
    };
    double relative[4];
    double vector_norm;
    double scale;
    int axis;

    quat_multiply(q_conjugate, perturbed->state.q, relative);
    if (relative[0] < 0.0) {
        for (axis = 0; axis < 4; ++axis) relative[axis] = -relative[axis];
    }
    vector_norm = sqrt(relative[1] * relative[1] + relative[2] * relative[2]
                       + relative[3] * relative[3]);
    scale = vector_norm > 1.0e-15
        ? 2.0 * atan2(vector_norm, relative[0]) / vector_norm : 2.0;
    for (axis = 0; axis < 3; ++axis) {
        error[ESKF_IDX_DTHETA + axis] = relative[axis + 1] * scale;
        error[ESKF_IDX_DV + axis] = perturbed->state.v[axis] - nominal->state.v[axis];
        error[ESKF_IDX_DP + axis] = perturbed->state.p[axis] - nominal->state.p[axis];
        error[ESKF_IDX_DAB + axis] = perturbed->state.ab[axis] - nominal->state.ab[axis];
        error[ESKF_IDX_DGB + axis] = perturbed->state.gb[axis] - nominal->state.gb[axis];
    }
}

static void inject_test_error(ESKF_Handle *handle, int index, double epsilon)
{
    if (index < ESKF_IDX_DV) {
        double rotation[3] = {0.0, 0.0, 0.0};
        double dq[4];
        double result[4];
        int axis;
        rotation[index] = epsilon;
        rotation_vector_quaternion(rotation, dq);
        quat_multiply(handle->state.q, dq, result);
        for (axis = 0; axis < 4; ++axis) handle->state.q[axis] = result[axis];
    } else if (index < ESKF_IDX_DP) {
        handle->state.v[index - ESKF_IDX_DV] += epsilon;
    } else if (index < ESKF_IDX_DAB) {
        handle->state.p[index - ESKF_IDX_DP] += epsilon;
    } else if (index < ESKF_IDX_DGB) {
        handle->state.ab[index - ESKF_IDX_DAB] += epsilon;
    } else {
        handle->state.gb[index - ESKF_IDX_DGB] += epsilon;
    }
}

static void test_randomized_prediction_transition(void)
{
    const double epsilon = 1.0e-6;
    double maximum_error = 0.0;
    uint32_t case_index;

    for (case_index = 0U; case_index < JACOBIAN_RANDOM_CASES; ++case_index) {
        double acceleration[3];
        double angular_rate[3];
        double corrected_acceleration[3];
        double corrected_angular_rate[3];
        double F[15][15];
        const double dt = uniform(0.001, 0.01);
        ESKF_Handle initial;
        ESKF_Handle nominal;
        int row;
        int column;

        eskf_init(&initial, NULL, NULL);
        euler_quaternion(
            uniform(-ESKF_PI, ESKF_PI), uniform(-1.35, 1.35),
            uniform(-ESKF_PI, ESKF_PI), initial.state.q
        );
        for (row = 0; row < 3; ++row) {
            initial.state.v[row] = uniform(-40.0, 40.0);
            initial.state.p[row] = uniform(-1000.0, 1000.0);
            initial.state.ab[row] = uniform(-0.3, 0.3);
            initial.state.gb[row] = uniform(-0.03, 0.03);
            acceleration[row] = uniform(-25.0, 25.0);
            angular_rate[row] = uniform(-6.0, 6.0);
            corrected_acceleration[row] = acceleration[row] - initial.state.ab[row];
            corrected_angular_rate[row] = angular_rate[row] - initial.state.gb[row];
        }
        eskf_model_transition(
            initial.state.q, corrected_acceleration, corrected_angular_rate, dt, F
        );
        nominal = initial;
        eskf_predict(&nominal, acceleration, angular_rate, dt);

        for (column = 0; column < 15; ++column) {
            ESKF_Handle perturbed = initial;
            double propagated_error[15];
            inject_test_error(&perturbed, column, epsilon);
            eskf_predict(&perturbed, acceleration, angular_rate, dt);
            error_between_handles(&nominal, &perturbed, propagated_error);
            for (row = 0; row < 15; ++row) {
                const double difference = fabs(
                    propagated_error[row] / epsilon - F[row][column]
                );
                if (difference > maximum_error) maximum_error = difference;
            }
        }
    }
    if (maximum_error >= 2.5e-3) {
        fprintf(stderr, "maximum randomized F error: %.9g\n", maximum_error);
    }
    check_true(maximum_error < 2.5e-3,
               "10,000-point transition campaign matches finite differences");
    printf("transition finite-difference maximum absolute error: %.9g\n", maximum_error);
}

static int matrix15_is_positive_semidefinite(double matrix[15][15])
{
    double lower[15][15] = {{0.0}};
    const double tolerance = 1.0e-13;
    int row;
    int column;
    int inner;

    for (row = 0; row < 15; ++row) {
        for (column = 0; column <= row; ++column) {
            double value = matrix[row][column];
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

static void expected_process_noise(const ESKF_Config *config,
                                   double dt,
                                   double expected[15][15])
{
    const double sigma_acc_squared = config->sigma_acc * config->sigma_acc;
    const double q_theta = config->sigma_gyr * config->sigma_gyr * dt;
    const double q_v = sigma_acc_squared * dt;
    const double q_vp = sigma_acc_squared * dt * dt * 0.5;
    const double q_p = sigma_acc_squared * dt * dt * dt / 3.0;
    const double q_ab = config->sigma_acc_bias * config->sigma_acc_bias * dt;
    const double q_gb = config->sigma_gyr_bias * config->sigma_gyr_bias * dt;
    int row;
    int column;
    int axis;

    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) expected[row][column] = 0.0;
    }
    for (axis = 0; axis < 3; ++axis) {
        expected[ESKF_IDX_DTHETA + axis][ESKF_IDX_DTHETA + axis] = q_theta;
        expected[ESKF_IDX_DV + axis][ESKF_IDX_DV + axis] = q_v;
        expected[ESKF_IDX_DV + axis][ESKF_IDX_DP + axis] = q_vp;
        expected[ESKF_IDX_DP + axis][ESKF_IDX_DV + axis] = q_vp;
        expected[ESKF_IDX_DP + axis][ESKF_IDX_DP + axis] = q_p;
        expected[ESKF_IDX_DAB + axis][ESKF_IDX_DAB + axis] = q_ab;
        expected[ESKF_IDX_DGB + axis][ESKF_IDX_DGB + axis] = q_gb;
    }
}

static void continuous_covariance_derivative(double Q[15][15],
                                             double A[15][15],
                                             double W[15][15],
                                             double derivative[15][15])
{
    int row;
    int column;
    int inner;
    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) {
            double value = W[row][column];
            for (inner = 0; inner < 15; ++inner) {
                value += A[row][inner] * Q[inner][column]
                    + Q[row][inner] * A[column][inner];
            }
            derivative[row][column] = value;
        }
    }
}

static void numerical_constant_process_noise(double A[15][15],
                                             double W[15][15],
                                             double dt,
                                             int integration_steps,
                                             double Q[15][15])
{
    const double step = dt / (double)integration_steps;
    double k1[15][15];
    double k2[15][15];
    double k3[15][15];
    double k4[15][15];
    double stage[15][15];
    int iteration;
    int row;
    int column;

    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) Q[row][column] = 0.0;
    }
    for (iteration = 0; iteration < integration_steps; ++iteration) {
        continuous_covariance_derivative(Q, A, W, k1);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                stage[row][column] = Q[row][column] + 0.5 * step * k1[row][column];
            }
        }
        continuous_covariance_derivative(stage, A, W, k2);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                stage[row][column] = Q[row][column] + 0.5 * step * k2[row][column];
            }
        }
        continuous_covariance_derivative(stage, A, W, k3);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                stage[row][column] = Q[row][column] + step * k3[row][column];
            }
        }
        continuous_covariance_derivative(stage, A, W, k4);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                Q[row][column] += step * (
                    k1[row][column] + 2.0 * k2[row][column]
                    + 2.0 * k3[row][column] + k4[row][column]
                ) / 6.0;
            }
        }
    }
}

static void numerical_reduced_process_noise(const ESKF_Config *config,
                                            double dt,
                                            double Q[15][15])
{
    double A[15][15] = {{0.0}};
    double W[15][15] = {{0.0}};
    int axis;

    for (axis = 0; axis < 3; ++axis) {
        A[ESKF_IDX_DP + axis][ESKF_IDX_DV + axis] = 1.0;
        W[ESKF_IDX_DTHETA + axis][ESKF_IDX_DTHETA + axis]
            = config->sigma_gyr * config->sigma_gyr;
        W[ESKF_IDX_DV + axis][ESKF_IDX_DV + axis]
            = config->sigma_acc * config->sigma_acc;
        W[ESKF_IDX_DAB + axis][ESKF_IDX_DAB + axis]
            = config->sigma_acc_bias * config->sigma_acc_bias;
        W[ESKF_IDX_DGB + axis][ESKF_IDX_DGB + axis]
            = config->sigma_gyr_bias * config->sigma_gyr_bias;
    }
    numerical_constant_process_noise(A, W, dt, 20, Q);
}

static void quaternion_to_rotation_matrix(const double q[4], double R[3][3])
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    R[0][0] = 1.0 - 2.0 * (y * y + z * z);
    R[0][1] = 2.0 * (x * y - w * z);
    R[0][2] = 2.0 * (x * z + w * y);
    R[1][0] = 2.0 * (x * y + w * z);
    R[1][1] = 1.0 - 2.0 * (x * x + z * z);
    R[1][2] = 2.0 * (y * z - w * x);
    R[2][0] = 2.0 * (x * z - w * y);
    R[2][1] = 2.0 * (y * z + w * x);
    R[2][2] = 1.0 - 2.0 * (x * x + y * y);
}

static void full_continuous_process_noise_model(const ESKF_Config *config,
                                                const double q[4],
                                                const double acceleration_body[3],
                                                const double angular_rate_body[3],
                                                double A[15][15],
                                                double W[15][15])
{
    double R[3][3];
    double acceleration_skew[3][3];
    int row;
    int column;
    int inner;

    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) {
            A[row][column] = 0.0;
            W[row][column] = 0.0;
        }
    }
    quaternion_to_rotation_matrix(q, R);
    acceleration_skew[0][0] = 0.0;
    acceleration_skew[0][1] = -acceleration_body[2];
    acceleration_skew[0][2] = acceleration_body[1];
    acceleration_skew[1][0] = acceleration_body[2];
    acceleration_skew[1][1] = 0.0;
    acceleration_skew[1][2] = -acceleration_body[0];
    acceleration_skew[2][0] = -acceleration_body[1];
    acceleration_skew[2][1] = acceleration_body[0];
    acceleration_skew[2][2] = 0.0;

    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            const double angular_skew = row == 0 && column == 1 ? -angular_rate_body[2]
                : row == 0 && column == 2 ? angular_rate_body[1]
                : row == 1 && column == 0 ? angular_rate_body[2]
                : row == 1 && column == 2 ? -angular_rate_body[0]
                : row == 2 && column == 0 ? -angular_rate_body[1]
                : row == 2 && column == 1 ? angular_rate_body[0] : 0.0;
            double rotated_acceleration_skew = 0.0;
            for (inner = 0; inner < 3; ++inner) {
                rotated_acceleration_skew += R[row][inner] * acceleration_skew[inner][column];
            }
            A[ESKF_IDX_DTHETA + row][ESKF_IDX_DTHETA + column] = -angular_skew;
            A[ESKF_IDX_DV + row][ESKF_IDX_DTHETA + column] = -rotated_acceleration_skew;
            A[ESKF_IDX_DV + row][ESKF_IDX_DAB + column] = -R[row][column];
            W[ESKF_IDX_DV + row][ESKF_IDX_DV + column] =
                config->sigma_acc * config->sigma_acc
                * (R[row][0] * R[column][0] + R[row][1] * R[column][1]
                   + R[row][2] * R[column][2]);
        }
        A[ESKF_IDX_DTHETA + row][ESKF_IDX_DGB + row] = -1.0;
        A[ESKF_IDX_DP + row][ESKF_IDX_DV + row] = 1.0;
        W[ESKF_IDX_DTHETA + row][ESKF_IDX_DTHETA + row]
            = config->sigma_gyr * config->sigma_gyr;
        W[ESKF_IDX_DAB + row][ESKF_IDX_DAB + row]
            = config->sigma_acc_bias * config->sigma_acc_bias;
        W[ESKF_IDX_DGB + row][ESKF_IDX_DGB + row]
            = config->sigma_gyr_bias * config->sigma_gyr_bias;
    }
}

static void test_randomized_process_noise_mapping(void)
{
    static const double deterministic_dt[] = {
        0.0001, 0.001, 0.0025, 0.005, 0.01, 0.02, 0.1
    };
    const uint32_t deterministic_count = (uint32_t)(
        sizeof(deterministic_dt) / sizeof(deterministic_dt[0])
    );
    double maximum_scaled_error = 0.0;
    int all_finite = 1;
    int all_symmetric = 1;
    int all_psd = 1;
    int all_elements_match = 1;
    uint32_t case_index;
    for (case_index = 0U;
         case_index < JACOBIAN_RANDOM_CASES + deterministic_count;
         ++case_index) {
        ESKF_Config config;
        double Q[15][15];
        double expected[15][15];
        const double dt = case_index < deterministic_count
            ? deterministic_dt[case_index] : pow(10.0, uniform(-4.0, -1.0));
        int row;
        int column;
        config.sigma_acc = pow(10.0, uniform(-2.0, 0.0));
        config.sigma_gyr = pow(10.0, uniform(-4.0, -1.0));
        config.sigma_acc_bias = pow(10.0, uniform(-5.0, -2.0));
        config.sigma_gyr_bias = pow(10.0, uniform(-6.0, -3.0));
        eskf_model_process_noise(&config, dt, Q);
        expected_process_noise(&config, dt, expected);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                const double error = fabs(Q[row][column] - expected[row][column]);
                const double tolerance = fmax(1.0e-15, 1.0e-12 * fabs(expected[row][column]));
                const double scaled_error = error / tolerance;
                all_finite = all_finite && isfinite(Q[row][column]);
                all_symmetric = all_symmetric
                    && fabs(Q[row][column] - Q[column][row]) < 1.0e-15;
                all_elements_match = all_elements_match && error <= tolerance;
                if (scaled_error > maximum_scaled_error) maximum_scaled_error = scaled_error;
            }
        }
        all_psd = all_psd && matrix15_is_positive_semidefinite(Q);
    }
    {
        const ESKF_Config zero_config = {0.0, 0.0, 0.0, 0.0};
        double zero_Q[15][15];
        int row;
        int column;
        eskf_model_process_noise(&zero_config, 0.01, zero_Q);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                all_elements_match = all_elements_match && zero_Q[row][column] == 0.0;
            }
        }
        all_psd = all_psd && matrix15_is_positive_semidefinite(zero_Q);
    }
    check_true(all_finite, "all 225 process-noise elements remain finite");
    check_true(all_symmetric, "full process-noise matrices are symmetric");
    check_true(all_psd, "full process-noise matrices are positive semidefinite");
    check_true(all_elements_match,
               "all 225 process-noise elements match the complete structure oracle");
    printf("process-noise oracle maximum tolerance ratio: %.9g\n", maximum_scaled_error);
}

static void test_process_noise_continuous_model_oracle(void)
{
    static const double dt_values[] = {0.0025, 0.005, 0.01, 0.1};
    const ESKF_Config config = {0.37, 0.018, 0.004, 0.0007};
    double maximum_error = 0.0;
    uint32_t index;
    for (index = 0U; index < (uint32_t)(sizeof(dt_values) / sizeof(dt_values[0]));
         ++index) {
        double production[15][15];
        double numerical[15][15];
        int row;
        int column;
        eskf_model_process_noise(&config, dt_values[index], production);
        numerical_reduced_process_noise(&config, dt_values[index], numerical);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                const double error = fabs(production[row][column] - numerical[row][column]);
                if (error > maximum_error) maximum_error = error;
            }
        }
    }
    check_true(maximum_error < 1.0e-12,
               "process noise matches independent continuous reduced-model integration");
    printf("continuous reduced-model Q maximum absolute error: %.9g\n", maximum_error);
}

static void test_full_process_noise_high_rate_defect(void)
{
    const ESKF_Config config = {0.37, 0.018, 0.004, 0.0007};
    const uint32_t cases = 1000U;
    double maximum_relative_defect = 0.0;
    double maximum_integration_error = 0.0;
    double maximum_linearization_error = 0.0;
    int all_full_matrices_finite_symmetric = 1;
    int all_full_matrices_psd = 1;
    int omitted_coupling_observed = 0;
    uint32_t case_index;

    for (case_index = 0U; case_index < cases; ++case_index) {
        double q[4];
        double acceleration_body[3];
        double angular_rate_body[3];
        double A[15][15];
        double W[15][15];
        double full[15][15];
        double refined[15][15];
        double production[15][15];
        double transition[15][15];
        double defect_squared = 0.0;
        double full_squared = 0.0;
        const double dt = uniform(0.001, 0.01);
        int row;
        int column;

        euler_quaternion(
            uniform(-ESKF_PI, ESKF_PI), uniform(-1.35, 1.35),
            uniform(-ESKF_PI, ESKF_PI), q
        );
        for (row = 0; row < 3; ++row) {
            acceleration_body[row] = uniform(-25.0, 25.0);
            angular_rate_body[row] = uniform(-6.0, 6.0);
        }
        full_continuous_process_noise_model(
            &config, q, acceleration_body, angular_rate_body, A, W
        );
        numerical_constant_process_noise(A, W, dt, 20, full);
        numerical_constant_process_noise(A, W, dt, 40, refined);
        eskf_model_process_noise(&config, dt, production);
        eskf_model_transition(q, acceleration_body, angular_rate_body, 1.0e-7, transition);
        all_full_matrices_psd = all_full_matrices_psd && matrix15_is_positive_semidefinite(full);
        for (row = 0; row < 15; ++row) {
            for (column = 0; column < 15; ++column) {
                const double defect = full[row][column] - production[row][column];
                const double integration_error = fabs(full[row][column] - refined[row][column]);
                const double linearization_error = fabs(
                    (transition[row][column] - (row == column ? 1.0 : 0.0)) / 1.0e-7
                    - A[row][column]
                );
                defect_squared += defect * defect;
                full_squared += full[row][column] * full[row][column];
                all_full_matrices_finite_symmetric = all_full_matrices_finite_symmetric
                    && isfinite(full[row][column])
                    && fabs(full[row][column] - full[column][row]) < 1.0e-12;
                if (integration_error > maximum_integration_error) {
                    maximum_integration_error = integration_error;
                }
                if (linearization_error > maximum_linearization_error) {
                    maximum_linearization_error = linearization_error;
                }
            }
        }
        if (sqrt(defect_squared) > 1.0e-14) omitted_coupling_observed = 1;
        if (sqrt(defect_squared) / sqrt(full_squared) > maximum_relative_defect) {
            maximum_relative_defect = sqrt(defect_squared) / sqrt(full_squared);
        }
    }
    check_true(all_full_matrices_finite_symmetric,
               "full continuous-model Q oracle remains finite and symmetric");
    check_true(all_full_matrices_psd,
               "full continuous-model Q oracle remains positive semidefinite");
    check_true(maximum_integration_error < 1.0e-13,
               "full continuous-model Q oracle converges under step refinement");
    check_true(maximum_linearization_error < 1.0e-5,
               "full continuous-model Q oracle matches the production transition derivative");
    check_true(omitted_coupling_observed,
               "full continuous-model oracle exposes nonzero omitted within-step coupling");
    check_true(maximum_relative_defect < 0.01,
               "reduced Q defect remains below one percent in the declared high-rate stress envelope");
    printf("full-model Q high-rate maximum relative defect: %.9g\n", maximum_relative_defect);
    printf("full-model Q oracle step-refinement maximum absolute error: %.9g\n",
           maximum_integration_error);
    printf("full-model Q transition-derivative maximum absolute error: %.9g\n",
           maximum_linearization_error);
}

static void test_randomized_heading_models(void)
{
    const double epsilon = 1.0e-7;
    const double vertical_ned[3] = {0.0, 0.0, 1.0};
    uint32_t case_index;
    double maximum_heading_error = 0.0;
    double maximum_magnetic_error = 0.0;
    double maximum_yaw_axis_error = 0.0;
    double maximum_tilt_invariance_error = 0.0;

    for (case_index = 0U; case_index < JACOBIAN_RANDOM_CASES; ++case_index) {
        double q[4];
        double heading;
        double heading_H[3];
        double yaw_error_H[3];
        double magnetic_H[3];
        double expected_yaw_axis[3];
        double perturbation[3];
        double dq[4];
        double plus_q[4];
        double minus_q[4];
        double plus_heading;
        double minus_heading;
        double unused_H[3];
        double reference_ned[3] = {
            uniform(0.2, 1.0), uniform(-0.8, 0.8), uniform(-1.0, 1.0)
        };
        double mag_body[3];
        double residual;
        double plus_residual;
        int axis;

        euler_quaternion(
            uniform(-ESKF_PI, ESKF_PI), uniform(-1.35, 1.35),
            uniform(-ESKF_PI, ESKF_PI), q
        );
        check_true(eskf_model_heading(q, &heading, heading_H),
                   "random trusted-heading geometry is observable");
        for (axis = 0; axis < 3; ++axis) {
            double derivative;
            perturbation[0] = 0.0;
            perturbation[1] = 0.0;
            perturbation[2] = 0.0;
            perturbation[axis] = epsilon;
            rotation_vector_quaternion(perturbation, dq);
            quat_multiply(q, dq, plus_q);
            perturbation[axis] = -epsilon;
            rotation_vector_quaternion(perturbation, dq);
            quat_multiply(q, dq, minus_q);
            check_true(eskf_model_heading(plus_q, &plus_heading, unused_H)
                           && eskf_model_heading(minus_q, &minus_heading, unused_H),
                       "perturbed trusted heading remains observable");
            derivative = wrap_pi(plus_heading - minus_heading) / (2.0 * epsilon);
            if (fabs(derivative - heading_H[axis]) > maximum_heading_error) {
                maximum_heading_error = fabs(derivative - heading_H[axis]);
            }
        }

        {
            const double commanded_residual = uniform(-0.5, 0.5);
            double corrected_q[4];
            double corrected_heading;
            double gravity_before[3];
            double gravity_after[3];
            rotate_transpose(q, vertical_ned, expected_yaw_axis);
            for (axis = 0; axis < 3; ++axis) {
                yaw_error_H[axis] = expected_yaw_axis[axis];
                perturbation[axis] = yaw_error_H[axis] * commanded_residual;
            }
            rotation_vector_quaternion(perturbation, dq);
            quat_multiply(q, dq, corrected_q);
            check_true(eskf_model_heading(corrected_q, &corrected_heading, unused_H),
                       "yaw-error corrected heading remains observable");
            if (fabs(wrap_pi(corrected_heading - heading - commanded_residual))
                > maximum_heading_error) {
                maximum_heading_error = fabs(
                    wrap_pi(corrected_heading - heading - commanded_residual)
                );
            }
            rotate_transpose(q, vertical_ned, gravity_before);
            rotate_transpose(corrected_q, vertical_ned, gravity_after);
            for (axis = 0; axis < 3; ++axis) {
                const double tilt_error = fabs(gravity_after[axis] - gravity_before[axis]);
                if (tilt_error > maximum_tilt_invariance_error) {
                    maximum_tilt_invariance_error = tilt_error;
                }
            }
        }

        rotate_transpose(q, reference_ned, mag_body);
        check_true(eskf_model_magnetic_yaw_correction(
                       q, mag_body, reference_ned, &residual, magnetic_H),
                   "random magnetic-heading geometry is observable");
        for (axis = 0; axis < 3; ++axis) {
            const double axis_error = fabs(magnetic_H[axis] - expected_yaw_axis[axis]);
            if (axis_error > maximum_yaw_axis_error) maximum_yaw_axis_error = axis_error;
            perturbation[axis] = epsilon * magnetic_H[axis];
        }
        rotation_vector_quaternion(perturbation, dq);
        quat_multiply(q, dq, plus_q);
        check_true(eskf_model_magnetic_yaw_correction(
                       plus_q, mag_body, reference_ned,
                       &plus_residual, unused_H),
                   "yaw-axis perturbed magnetic heading remains observable");
        {
            const double derivative = -wrap_pi(plus_residual - residual) / epsilon;
            if (fabs(derivative - 1.0) > maximum_magnetic_error) {
                maximum_magnetic_error = fabs(derivative - 1.0);
            }
        }
        if (failures != 0) break;
    }
    check_true(maximum_heading_error < 5.0e-6,
               "10,000-point trusted-heading derivative campaign passes");
    check_true(maximum_magnetic_error < 5.0e-6,
               "10,000-point magnetic-heading derivative campaign passes");
    check_true(maximum_yaw_axis_error < 1.0e-12,
               "trusted and magnetic yaw-error models use the NED-down correction axis");
    check_true(maximum_tilt_invariance_error < 1.0e-12,
               "finite yaw-error corrections preserve the gravity direction exactly");
    printf("heading finite-difference maximum absolute error: %.9g\n",
           maximum_heading_error);
    printf("magnetic-heading finite-difference maximum absolute error: %.9g\n",
           maximum_magnetic_error);
    printf("yaw-error axis maximum absolute error: %.9g\n", maximum_yaw_axis_error);
    printf("yaw-error tilt invariance maximum absolute error: %.9g\n",
           maximum_tilt_invariance_error);
}

static void test_heading_observability_boundaries(void)
{
    const double vertical_forward_q[4] = {sqrt(0.5), 0.0, sqrt(0.5), 0.0};
    const double identity_q[4] = {1.0, 0.0, 0.0, 0.0};
    const double vertical_field[3] = {0.0, 0.0, 1.0};
    const double north_reference[3] = {1.0, 0.0, 0.0};
    static const double factors[3] = {0.99, 1.0, 1.01};
    double value;
    double H[3];
    int index;
    check_true(!eskf_model_heading(vertical_forward_q, &value, H),
               "trusted heading explicitly reports vertical geometry unobservable");
    check_true(!eskf_model_magnetic_yaw_correction(
                   identity_q, vertical_field, north_reference, &value, H),
               "magnetic heading explicitly reports vertical field unobservable");
    for (index = 0; index < 3; ++index) {
        const double horizontal = sqrt(factors[index] * 1.0e-4);
        const double pitch = acos(horizontal);
        double q[4];
        const int expected_observable = index == 2;
        euler_quaternion(0.0, pitch, 0.0, q);
        check_true(eskf_model_heading(q, &value, H) == expected_observable,
                   "trusted-heading threshold is fail-closed at and below its boundary");
    }
    for (index = 0; index < 3; ++index) {
        const double horizontal = sqrt(factors[index] * ESKF_EPSILON);
        const double measured[3] = {horizontal, 0.0, 1.0};
        const double reference[3] = {horizontal, 0.0, 1.0};
        const int expected_observable = index == 2;
        check_true(eskf_model_magnetic_yaw_correction(
                       identity_q, measured, north_reference, &value, H)
                       == expected_observable,
                   "measured magnetic horizontal threshold is fail-closed");
        check_true(eskf_model_magnetic_yaw_correction(
                       identity_q, north_reference, reference, &value, H)
                       == expected_observable,
                   "reference magnetic horizontal threshold is fail-closed");
    }
}

static void test_dimension_aware_nis_limits(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    const double position[3] = {sqrt(10.0), 0.0, 0.0};
    int row;
    int column;

    eskf_init(&filter, NULL, NULL);
    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) filter.P[row][column] = 0.0;
    }
    eskf_update_position(&filter, position, 1.0, &result);
    check_true(result.accepted && fabs(result.nis - 10.0f) < 1.0e-5f,
               "3D NIS 10 passes the 99.73-percent three-axis limit");

    eskf_init(&filter, NULL, NULL);
    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) filter.P[row][column] = 0.0;
    }
    eskf_update_heading(&filter, 3.1, 1.0, &result);
    check_true(!result.accepted && result.nis > 9.0f,
               "1D NIS above 9 fails the scalar three-sigma limit");
}

int main(void)
{
    test_randomized_prediction_transition();
    test_randomized_process_noise_mapping();
    test_process_noise_continuous_model_oracle();
    test_full_process_noise_high_rate_defect();
    test_randomized_heading_models();
    test_heading_observability_boundaries();
    test_dimension_aware_nis_limits();
    if (failures != 0) {
        fprintf(stderr, "%d ESKF model assertion(s) failed\n", failures);
        return 1;
    }
    printf("ESKF model campaign passed: %u randomized cases per model family\n",
           (unsigned)JACOBIAN_RANDOM_CASES);
    return 0;
}
