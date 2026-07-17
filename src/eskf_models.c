/**
 * @file eskf_models.c
 * @brief Internal heading measurement models for the right-error ESKF.
 */

#include "eskf_models.h"

#include "eskf_math.h"

static eskf_float_t wrap_pi(eskf_float_t angle)
{
    return atan2(sin(angle), cos(angle));
}

/* A NED-frame yaw rotation maps into the right/body error as R^T e_z. */
static void ned_yaw_correction_axis(eskf_float_t R_nb[3][3],
                                    eskf_float_t H_theta[3])
{
    int axis;
    for (axis = 0; axis < 3; ++axis) {
        H_theta[axis] = R_nb[2][axis];
    }
}

bool eskf_model_heading(const eskf_float_t q[4],
                        eskf_float_t *heading_rad,
                        eskf_float_t H_theta[3])
{
    eskf_float_t R_nb[3][3];
    eskf_float_t horizontal_squared;
    if (!q || !heading_rad || !H_theta) return false;
    eskf_quat_to_rot_mat3(q, R_nb);
    horizontal_squared = R_nb[0][0] * R_nb[0][0] + R_nb[1][0] * R_nb[1][0];
    if (!isfinite(horizontal_squared) || horizontal_squared <= ESKF_EPSILON) return false;
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

    if (!q || !mag_body || !mag_reference_ned || !residual_rad || !H_theta) return false;
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
