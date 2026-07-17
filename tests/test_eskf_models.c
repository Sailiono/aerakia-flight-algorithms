#include "eskf_models.h"

#include <aerakia/eskf.h>

#include <math.h>
#include <stdio.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
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
    const double scale = sin(0.5 * angle) / angle;
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
        out[row] = R[0][row] * vector[0] + R[1][row] * vector[1] + R[2][row] * vector[2];
    }
}

static void test_trusted_heading_correction_geometry(void)
{
    const double epsilon = 1.0e-7;
    double q[4];
    double heading;
    double H[3];
    double perturbation[3];
    double dq[4];
    double perturbed_q[4];
    double perturbed_heading;
    double unused_H[3];
    double left_yaw[4] = {cos(0.5 * epsilon), 0.0, 0.0, sin(0.5 * epsilon)};
    double left_perturbed_q[4];
    double quaternion_dot = 0.0;
    int axis;

    euler_quaternion(0.7, -0.35, 0.42, q);
    check_true(eskf_model_heading(q, &heading, H), "trusted heading model is observable");
    for (axis = 0; axis < 3; ++axis) {
        perturbation[axis] = epsilon * H[axis];
    }
    check_true(fabs(H[0] * H[0] + H[1] * H[1] + H[2] * H[2] - 1.0) < 1.0e-12,
               "NED yaw correction axis has unit norm");
    rotation_vector_quaternion(perturbation, dq);
    quat_multiply(q, dq, perturbed_q);
    quat_multiply(left_yaw, q, left_perturbed_q);
    check_true(
        eskf_model_heading(perturbed_q, &perturbed_heading, unused_H),
        "yaw-axis perturbed trusted heading remains observable"
    );
    check_true(fabs(wrap_pi(perturbed_heading - heading) / epsilon - 1.0) < 2.0e-6,
               "right-error correction axis produces unit NED heading change");
    for (axis = 0; axis < 4; ++axis) quaternion_dot += perturbed_q[axis] * left_perturbed_q[axis];
    check_true(fabs(fabs(quaternion_dot) - 1.0) < 1.0e-12,
               "right-error correction is equivalent to a pure left NED-yaw rotation");
}

static void test_magnetic_heading_correction_geometry(void)
{
    const double epsilon = 1.0e-7;
    const double reference_ned[3] = {0.44, 0.13, 0.89};
    double q[4];
    double mag_body[3];
    double residual;
    double H[3];
    double perturbation[3];
    double dq[4];
    double perturbed_q[4];
    double perturbed_residual;
    double unused_H[3];
    int axis;

    euler_quaternion(-0.8, 0.27, -0.51, q);
    rotate_transpose(q, reference_ned, mag_body);
    check_true(
        eskf_model_magnetic_heading(q, mag_body, reference_ned, &residual, H),
        "magnetic heading model is observable"
    );
    check_true(fabs(residual) < 1.0e-12, "consistent magnetic field has zero residual");
    for (axis = 0; axis < 3; ++axis) perturbation[axis] = epsilon * H[axis];
    rotation_vector_quaternion(perturbation, dq);
    quat_multiply(q, dq, perturbed_q);
    check_true(
        eskf_model_magnetic_heading(
            perturbed_q, mag_body, reference_ned, &perturbed_residual, unused_H
        ),
        "yaw-axis perturbed magnetic heading remains observable"
    );
    /* residual = reference - measured heading. */
    check_true(fabs(-wrap_pi(perturbed_residual - residual) / epsilon - 1.0) < 2.0e-6,
               "magnetic heading changes one-for-one along the NED yaw correction axis");
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
               "trusted heading rejects vertical body-forward axis");
    check_true(!eskf_model_magnetic_heading(
                   identity_q, vertical_field, north_reference, &value, H),
               "magnetic heading rejects a vertical measured field");
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
    vector_norm = sqrt(
        relative[1] * relative[1] + relative[2] * relative[2]
        + relative[3] * relative[3]
    );
    scale = vector_norm > 1.0e-15 ? 2.0 * atan2(vector_norm, relative[0]) / vector_norm : 2.0;
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
        rotation[index] = epsilon;
        rotation_vector_quaternion(rotation, dq);
        quat_multiply(handle->state.q, dq, result);
        for (int axis = 0; axis < 4; ++axis) handle->state.q[axis] = result[axis];
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

static void test_prediction_transition_finite_difference(void)
{
    const double dt = 0.005;
    const double epsilon = 1.0e-5;
    const double acceleration[3] = {0.8, -0.4, -9.2};
    const double angular_rate[3] = {0.31, -0.27, 0.19};
    double corrected_acceleration[3];
    double corrected_angular_rate[3];
    double F[15][15];
    double maximum_error = 0.0;
    ESKF_Handle initial;
    ESKF_Handle nominal;
    int row;
    int column;

    eskf_init(&initial, NULL, NULL);
    euler_quaternion(0.6, -0.3, 0.2, initial.state.q);
    initial.state.v[0] = 3.0; initial.state.v[1] = -1.0; initial.state.v[2] = 0.4;
    initial.state.p[0] = 0.2; initial.state.p[1] = -0.1; initial.state.p[2] = 0.3;
    initial.state.ab[0] = 0.03; initial.state.ab[1] = -0.02; initial.state.ab[2] = 0.01;
    initial.state.gb[0] = 0.002; initial.state.gb[1] = -0.003; initial.state.gb[2] = 0.001;
    for (row = 0; row < 3; ++row) {
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
            const double difference = fabs(propagated_error[row] / epsilon - F[row][column]);
            if (difference > maximum_error) maximum_error = difference;
        }
    }
    check_true(maximum_error < 3.0e-5,
               "prediction transition matches finite-difference nominal propagation");
    check_true(fabs(F[ESKF_IDX_DP][ESKF_IDX_DTHETA + 1]) > 1.0e-8,
               "position transition retains dt-squared attitude coupling");
    check_true(fabs(F[ESKF_IDX_DP][ESKF_IDX_DAB]) > 1.0e-8,
               "position transition retains dt-squared accelerometer-bias coupling");
}

int main(void)
{
    test_trusted_heading_correction_geometry();
    test_magnetic_heading_correction_geometry();
    test_heading_singularities();
    test_prediction_transition_finite_difference();
    if (failures != 0) {
        fprintf(stderr, "%d ESKF model assertion(s) failed\n", failures);
        return 1;
    }
    puts("ESKF model tests passed");
    return 0;
}
