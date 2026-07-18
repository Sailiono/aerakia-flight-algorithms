/**
 * @file eskf_models.c
 * @brief Internal transition, noise, and heading models for the right-error ESKF.
 */

#include "eskf_models.h"

#include "eskf_math.h"

#include <stddef.h>

static eskf_float_t wrap_pi(eskf_float_t angle)
{
    return atan2(sin(angle), cos(angle));
}

void eskf_model_transition(const eskf_float_t q[4],
                           const eskf_float_t acceleration_body[3],
                           const eskf_float_t angular_rate_body[3],
                           eskf_float_t dt,
                           eskf_float_t F[15][15])
{
    eskf_float_t R_nb[3][3];
    eskf_float_t acceleration_skew[3][3];
    eskf_float_t R_acceleration_skew[3][3];
    eskf_float_t delta_q[4];
    eskf_float_t R_delta[3][3];
    eskf_float_t phi[3];
    eskf_float_t phi_skew[3][3];
    eskf_float_t phi_skew_squared[3][3];
    eskf_float_t right_jacobian[3][3];
    const eskf_float_t half_dt_squared = 0.5 * dt * dt;
    eskf_float_t theta;
    eskf_float_t coefficient_a;
    eskf_float_t coefficient_b;
    int row;
    int column;

    eskf_mat15_identity(F);
    eskf_quat_to_rot_mat3(q, R_nb);
    eskf_mat3_skew(acceleration_body, acceleration_skew);
    eskf_mat3_mul_mat3(R_nb, acceleration_skew, R_acceleration_skew);

    for (row = 0; row < 3; ++row) phi[row] = angular_rate_body[row] * dt;
    eskf_quat_from_rotation_vector(phi, delta_q);
    eskf_quat_to_rot_mat3(delta_q, R_delta);
    eskf_mat3_skew(phi, phi_skew);
    eskf_mat3_mul_mat3(phi_skew, phi_skew, phi_skew_squared);
    theta = eskf_vec3_norm(phi);
    if (theta < 1.0e-6) {
        coefficient_a = 0.5 - theta * theta / 24.0;
        coefficient_b = 1.0 / 6.0 - theta * theta / 120.0;
    } else {
        coefficient_a = (1.0 - cos(theta)) / (theta * theta);
        coefficient_b = (theta - sin(theta)) / (theta * theta * theta);
    }
    eskf_mat3_identity(right_jacobian);
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            right_jacobian[row][column] -= coefficient_a * phi_skew[row][column];
            right_jacobian[row][column] += coefficient_b * phi_skew_squared[row][column];
        }
    }

    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            F[ESKF_IDX_DTHETA + row][ESKF_IDX_DTHETA + column]
                = R_delta[column][row];
            F[ESKF_IDX_DV + row][ESKF_IDX_DTHETA + column]
                = -R_acceleration_skew[row][column] * dt;
            F[ESKF_IDX_DV + row][ESKF_IDX_DAB + column]
                = -R_nb[row][column] * dt;
            F[ESKF_IDX_DP + row][ESKF_IDX_DTHETA + column]
                = -R_acceleration_skew[row][column] * half_dt_squared;
            F[ESKF_IDX_DP + row][ESKF_IDX_DAB + column]
                = -R_nb[row][column] * half_dt_squared;
        }
        for (column = 0; column < 3; ++column) {
            F[ESKF_IDX_DTHETA + row][ESKF_IDX_DGB + column]
                = -right_jacobian[row][column] * dt;
        }
        F[ESKF_IDX_DP + row][ESKF_IDX_DV + row] = dt;
    }
}

void eskf_model_process_noise(const ESKF_Config *config,
                              eskf_float_t dt,
                              eskf_float_t Q[15][15])
{
    const eskf_float_t sigma_acc_squared = config->sigma_acc * config->sigma_acc;
    const eskf_float_t q_theta = config->sigma_gyr * config->sigma_gyr * dt;
    const eskf_float_t q_v = sigma_acc_squared * dt;
    const eskf_float_t q_vp = sigma_acc_squared * dt * dt * 0.5;
    const eskf_float_t q_p = sigma_acc_squared * dt * dt * dt / 3.0;
    const eskf_float_t q_ab = config->sigma_acc_bias * config->sigma_acc_bias * dt;
    const eskf_float_t q_gb = config->sigma_gyr_bias * config->sigma_gyr_bias * dt;
    int axis;

    eskf_mat15_zero(Q);
    for (axis = 0; axis < 3; ++axis) {
        Q[ESKF_IDX_DTHETA + axis][ESKF_IDX_DTHETA + axis] = q_theta;
        Q[ESKF_IDX_DV + axis][ESKF_IDX_DV + axis] = q_v;
        Q[ESKF_IDX_DP + axis][ESKF_IDX_DP + axis] = q_p;
        Q[ESKF_IDX_DV + axis][ESKF_IDX_DP + axis] = q_vp;
        Q[ESKF_IDX_DP + axis][ESKF_IDX_DV + axis] = q_vp;
        Q[ESKF_IDX_DAB + axis][ESKF_IDX_DAB + axis] = q_ab;
        Q[ESKF_IDX_DGB + axis][ESKF_IDX_DGB + axis] = q_gb;
    }
}

static void ned_yaw_correction_axis(eskf_float_t R_nb[3][3],
                                    eskf_float_t H_theta[3])
{
    int axis;
    for (axis = 0; axis < 3; ++axis) H_theta[axis] = R_nb[2][axis];
}

bool eskf_model_heading(const eskf_float_t q[4],
                        eskf_float_t *heading_rad,
                        eskf_float_t H_theta[3])
{
    eskf_float_t R_nb[3][3];
    eskf_float_t horizontal_squared;
    if (q == NULL || heading_rad == NULL || H_theta == NULL) return false;
    eskf_quat_to_rot_mat3(q, R_nb);
    horizontal_squared = R_nb[0][0] * R_nb[0][0] + R_nb[1][0] * R_nb[1][0];
    if (!isfinite(horizontal_squared) || horizontal_squared <= 1.0e-4) return false;
    *heading_rad = atan2(R_nb[1][0], R_nb[0][0]);
    ned_yaw_correction_axis(R_nb, H_theta);
    return true;
}

bool eskf_model_magnetic_heading(const eskf_float_t q[4],
                                 const eskf_float_t mag_body[3],
                                 const eskf_float_t mag_reference_ned[3],
                                 eskf_float_t *residual_rad,
                                 eskf_float_t H_theta[3])
{
    eskf_float_t R_nb[3][3];
    eskf_float_t measured_ned[3];
    eskf_float_t measured_horizontal_squared;
    eskf_float_t reference_horizontal_squared;

    if (q == NULL || mag_body == NULL || mag_reference_ned == NULL
        || residual_rad == NULL || H_theta == NULL) return false;
    reference_horizontal_squared = mag_reference_ned[0] * mag_reference_ned[0]
        + mag_reference_ned[1] * mag_reference_ned[1];
    if (!isfinite(reference_horizontal_squared)
        || reference_horizontal_squared <= ESKF_EPSILON) return false;
    eskf_quat_to_rot_mat3(q, R_nb);
    eskf_mat3_mul_vec3(R_nb, mag_body, measured_ned);
    measured_horizontal_squared = measured_ned[0] * measured_ned[0]
        + measured_ned[1] * measured_ned[1];
    if (!isfinite(measured_horizontal_squared)
        || measured_horizontal_squared <= ESKF_EPSILON) return false;

    *residual_rad = wrap_pi(
        atan2(mag_reference_ned[1], mag_reference_ned[0])
        - atan2(measured_ned[1], measured_ned[0])
    );
    ned_yaw_correction_axis(R_nb, H_theta);
    return true;
}
