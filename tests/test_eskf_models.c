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

static void test_randomized_process_noise_mapping(void)
{
    uint32_t case_index;
    for (case_index = 0U; case_index < JACOBIAN_RANDOM_CASES; ++case_index) {
        ESKF_Config config;
        double Q[15][15];
        const double dt = uniform(0.0005, 0.02);
        int axis;
        config.sigma_acc = uniform(0.01, 1.0);
        config.sigma_gyr = uniform(0.0001, 0.1);
        config.sigma_acc_bias = uniform(1.0e-5, 0.01);
        config.sigma_gyr_bias = uniform(1.0e-6, 0.001);
        eskf_model_process_noise(&config, dt, Q);
        for (axis = 0; axis < 3; ++axis) {
            const double determinant =
                Q[ESKF_IDX_DV + axis][ESKF_IDX_DV + axis]
                    * Q[ESKF_IDX_DP + axis][ESKF_IDX_DP + axis]
                - Q[ESKF_IDX_DV + axis][ESKF_IDX_DP + axis]
                    * Q[ESKF_IDX_DP + axis][ESKF_IDX_DV + axis];
            check_true(Q[ESKF_IDX_DTHETA + axis][ESKF_IDX_DTHETA + axis] > 0.0,
                       "gyro process noise is positive");
            check_true(Q[ESKF_IDX_DV + axis][ESKF_IDX_DP + axis]
                           == Q[ESKF_IDX_DP + axis][ESKF_IDX_DV + axis],
                       "velocity-position process noise is symmetric");
            check_true(determinant >= 0.0,
                       "integrated acceleration process-noise block is PSD");
        }
        if (failures != 0) break;
    }
}

static void test_randomized_heading_models(void)
{
    const double epsilon = 1.0e-7;
    uint32_t case_index;
    double maximum_heading_error = 0.0;
    double maximum_magnetic_error = 0.0;

    for (case_index = 0U; case_index < JACOBIAN_RANDOM_CASES; ++case_index) {
        double q[4];
        double heading;
        double H[3];
        double perturbation[3];
        double dq[4];
        double perturbed_q[4];
        double perturbed_heading;
        double unused_H[3];
        double reference_ned[3] = {
            uniform(0.2, 1.0), uniform(-0.8, 0.8), uniform(-1.0, 1.0)
        };
        double mag_body[3];
        double residual;
        double perturbed_residual;
        int axis;

        euler_quaternion(
            uniform(-ESKF_PI, ESKF_PI), uniform(-1.35, 1.35),
            uniform(-ESKF_PI, ESKF_PI), q
        );
        check_true(eskf_model_heading(q, &heading, H),
                   "random trusted-heading geometry is observable");
        for (axis = 0; axis < 3; ++axis) perturbation[axis] = epsilon * H[axis];
        rotation_vector_quaternion(perturbation, dq);
        quat_multiply(q, dq, perturbed_q);
        check_true(eskf_model_heading(perturbed_q, &perturbed_heading, unused_H),
                   "perturbed trusted heading remains observable");
        {
            const double error = fabs(wrap_pi(perturbed_heading - heading) / epsilon - 1.0);
            if (error > maximum_heading_error) maximum_heading_error = error;
        }

        rotate_transpose(q, reference_ned, mag_body);
        check_true(eskf_model_magnetic_heading(
                       q, mag_body, reference_ned, &residual, H),
                   "random magnetic-heading geometry is observable");
        for (axis = 0; axis < 3; ++axis) perturbation[axis] = epsilon * H[axis];
        rotation_vector_quaternion(perturbation, dq);
        quat_multiply(q, dq, perturbed_q);
        check_true(eskf_model_magnetic_heading(
                       perturbed_q, mag_body, reference_ned,
                       &perturbed_residual, unused_H),
                   "perturbed magnetic heading remains observable");
        {
            const double error = fabs(
                -wrap_pi(perturbed_residual - residual) / epsilon - 1.0
            );
            if (error > maximum_magnetic_error) maximum_magnetic_error = error;
        }
        if (failures != 0) break;
    }
    check_true(maximum_heading_error < 5.0e-6,
               "10,000-point trusted-heading derivative campaign passes");
    check_true(maximum_magnetic_error < 5.0e-6,
               "10,000-point magnetic-heading derivative campaign passes");
    printf("heading finite-difference maximum absolute error: %.9g\n",
           maximum_heading_error);
    printf("magnetic-heading finite-difference maximum absolute error: %.9g\n",
           maximum_magnetic_error);
}

static void test_heading_singularities(void)
{
    const double vertical_forward_q[4] = {sqrt(0.5), 0.0, sqrt(0.5), 0.0};
    const double identity_q[4] = {1.0, 0.0, 0.0, 0.0};
    const double vertical_field[3] = {0.0, 0.0, 1.0};
    const double north_reference[3] = {1.0, 0.0, 0.0};
    double value;
    double H[3];
    check_true(!eskf_model_heading(vertical_forward_q, &value, H),
               "trusted heading explicitly reports vertical geometry unobservable");
    check_true(!eskf_model_magnetic_heading(
                   identity_q, vertical_field, north_reference, &value, H),
               "magnetic heading explicitly reports vertical field unobservable");
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
    test_randomized_heading_models();
    test_heading_singularities();
    test_dimension_aware_nis_limits();
    if (failures != 0) {
        fprintf(stderr, "%d ESKF model assertion(s) failed\n", failures);
        return 1;
    }
    printf("ESKF model campaign passed: %u randomized cases per model family\n",
           (unsigned)JACOBIAN_RANDOM_CASES);
    return 0;
}
