/**
 * @file static_imu_calibration.c
 * @brief Gravity-sphere static IMU bias calibration implementation.
 */

#include <aerakia/static_imu_calibration.h>

#include <math.h>
#include <string.h>

static bool vector_is_finite(AerakiaVec3f vector)
{
    return isfinite(vector.x) && isfinite(vector.y) && isfinite(vector.z);
}

static double vector_norm(AerakiaVec3f vector)
{
    return sqrt(
        (double)vector.x * vector.x
        + (double)vector.y * vector.y
        + (double)vector.z * vector.z
    );
}

static void sort_ascending(double values[3])
{
    int first;
    int second;
    for (first = 0; first < 3; ++first) {
        for (second = first + 1; second < 3; ++second) {
            if (values[second] < values[first]) {
                const double temporary = values[first];
                values[first] = values[second];
                values[second] = temporary;
            }
        }
    }
}

/* Jacobi diagonalization is sufficient for the small symmetric diagnostics here. */
static bool symmetric_eigenvalues_3x3(double input[3][3], double eigenvalues[3])
{
    double matrix[3][3];
    int iteration;
    int row;
    int column;

    memcpy(matrix, input, sizeof(matrix));
    for (iteration = 0; iteration < 32; ++iteration) {
        int pivot_row = 0;
        int pivot_column = 1;
        double maximum = fabs(matrix[0][1]);
        const double candidate_02 = fabs(matrix[0][2]);
        const double candidate_12 = fabs(matrix[1][2]);
        if (candidate_02 > maximum) {
            pivot_row = 0;
            pivot_column = 2;
            maximum = candidate_02;
        }
        if (candidate_12 > maximum) {
            pivot_row = 1;
            pivot_column = 2;
            maximum = candidate_12;
        }
        if (!isfinite(maximum)) return false;
        if (maximum <= 1.0e-14) break;

        {
            const double app = matrix[pivot_row][pivot_row];
            const double aqq = matrix[pivot_column][pivot_column];
            const double apq = matrix[pivot_row][pivot_column];
            const double tau = (aqq - app) / (2.0 * apq);
            const double tangent = (tau >= 0.0 ? 1.0 : -1.0)
                / (fabs(tau) + sqrt(1.0 + tau * tau));
            const double cosine = 1.0 / sqrt(1.0 + tangent * tangent);
            const double sine = tangent * cosine;

            for (row = 0; row < 3; ++row) {
                if (row == pivot_row || row == pivot_column) continue;
                {
                    const double arp = matrix[row][pivot_row];
                    const double arq = matrix[row][pivot_column];
                    matrix[row][pivot_row] = cosine * arp - sine * arq;
                    matrix[pivot_row][row] = matrix[row][pivot_row];
                    matrix[row][pivot_column] = sine * arp + cosine * arq;
                    matrix[pivot_column][row] = matrix[row][pivot_column];
                }
            }
            matrix[pivot_row][pivot_row] = app - tangent * apq;
            matrix[pivot_column][pivot_column] = aqq + tangent * apq;
            matrix[pivot_row][pivot_column] = 0.0;
            matrix[pivot_column][pivot_row] = 0.0;
        }
    }

    for (row = 0; row < 3; ++row) {
        if (!isfinite(matrix[row][row])) return false;
        eigenvalues[row] = matrix[row][row];
        for (column = 0; column < 3; ++column) {
            if (!isfinite(matrix[row][column])) return false;
        }
    }
    sort_ascending(eigenvalues);
    return true;
}

static bool solve_3x3(double input[3][3], const double rhs_input[3], double solution[3])
{
    double augmented[3][4];
    int pivot;
    int row;
    int column;

    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            augmented[row][column] = input[row][column];
        }
        augmented[row][3] = rhs_input[row];
    }
    for (pivot = 0; pivot < 3; ++pivot) {
        int best_row = pivot;
        double best = fabs(augmented[pivot][pivot]);
        for (row = pivot + 1; row < 3; ++row) {
            const double candidate = fabs(augmented[row][pivot]);
            if (candidate > best) {
                best = candidate;
                best_row = row;
            }
        }
        if (!isfinite(best) || best <= 1.0e-15) return false;
        if (best_row != pivot) {
            for (column = pivot; column < 4; ++column) {
                const double temporary = augmented[pivot][column];
                augmented[pivot][column] = augmented[best_row][column];
                augmented[best_row][column] = temporary;
            }
        }
        {
            const double divisor = augmented[pivot][pivot];
            for (column = pivot; column < 4; ++column) {
                augmented[pivot][column] /= divisor;
            }
        }
        for (row = 0; row < 3; ++row) {
            double factor;
            if (row == pivot) continue;
            factor = augmented[row][pivot];
            for (column = pivot; column < 4; ++column) {
                augmented[row][column] -= factor * augmented[pivot][column];
            }
        }
    }
    for (row = 0; row < 3; ++row) {
        if (!isfinite(augmented[row][3])) return false;
        solution[row] = augmented[row][3];
    }
    return true;
}

/* Fit the gravity-sphere centre from all pose pairs except an optional pose. */
static bool fit_accelerometer_bias(
    const AerakiaStaticImuPoseMean *poses,
    uint32_t pose_count,
    int excluded_pose,
    const AerakiaStaticImuCalibrationConfig *config,
    double bias[3],
    double geometry_eigenvalues[3]
)
{
    double normal[3][3] = {{0.0}};
    double rhs[3] = {0.0, 0.0, 0.0};
    uint32_t included_pose_count = 0U;
    uint32_t first;
    uint32_t second;
    for (first = 0; first < pose_count; ++first) {
        if ((int)first != excluded_pose) included_pose_count++;
    }
    if (included_pose_count < 4U) return false;

    for (first = 0; first < pose_count; ++first) {
        const AerakiaVec3f first_accel = poses[first].acceleration_m_s2;
        const double first_norm_squared = (double)first_accel.x * first_accel.x
            + (double)first_accel.y * first_accel.y
            + (double)first_accel.z * first_accel.z;
        if ((int)first == excluded_pose) continue;
        for (second = first + 1U; second < pose_count; ++second) {
            const AerakiaVec3f second_accel = poses[second].acceleration_m_s2;
            const double row[3] = {
                2.0 * ((double)first_accel.x - second_accel.x),
                2.0 * ((double)first_accel.y - second_accel.y),
                2.0 * ((double)first_accel.z - second_accel.z)
            };
            const double second_norm_squared = (double)second_accel.x * second_accel.x
                + (double)second_accel.y * second_accel.y
                + (double)second_accel.z * second_accel.z;
            const double difference = first_norm_squared - second_norm_squared;
            int row_axis;
            int column_axis;
            if ((int)second == excluded_pose) continue;
            for (row_axis = 0; row_axis < 3; ++row_axis) {
                rhs[row_axis] += row[row_axis] * difference;
                for (column_axis = 0; column_axis < 3; ++column_axis) {
                    normal[row_axis][column_axis] += row[row_axis] * row[column_axis];
                }
            }
        }
    }

    if (!symmetric_eigenvalues_3x3(normal, geometry_eigenvalues)
        || geometry_eigenvalues[2] <= 0.0 || geometry_eigenvalues[0] <= 0.0
        || geometry_eigenvalues[0] / geometry_eigenvalues[2]
            < (double)config->minimum_geometry_eigenvalue_ratio) {
        return false;
    }
    return solve_3x3(normal, rhs, bias);
}

static bool config_is_valid(const AerakiaStaticImuCalibrationConfig *config)
{
    return config != NULL
        && config->minimum_pose_count >= 4U
        && config->minimum_pose_count <= AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES
        && isfinite(config->gravity_m_s2) && config->gravity_m_s2 > 0.0f
        && isfinite(config->maximum_stationary_gyro_norm_rad_s)
        && config->maximum_stationary_gyro_norm_rad_s > 0.0f
        && isfinite(config->minimum_geometry_eigenvalue_ratio)
        && config->minimum_geometry_eigenvalue_ratio > 0.0f
        && config->minimum_geometry_eigenvalue_ratio <= 1.0f
        && isfinite(config->minimum_direction_coverage_eigenvalue)
        && config->minimum_direction_coverage_eigenvalue > 0.0f
        && config->minimum_direction_coverage_eigenvalue <= (1.0f / 3.0f)
        && isfinite(config->maximum_gravity_residual_m_s2)
        && config->maximum_gravity_residual_m_s2 > 0.0f
        && isfinite(config->maximum_gravity_residual_rms_m_s2)
        && config->maximum_gravity_residual_rms_m_s2 > 0.0f
        && isfinite(config->maximum_gyroscope_residual_rms_rad_s)
        && config->maximum_gyroscope_residual_rms_rad_s > 0.0f
        && isfinite(config->maximum_leave_one_out_bias_delta_m_s2)
        && config->maximum_leave_one_out_bias_delta_m_s2 > 0.0f;
}

void aerakia_static_imu_calibration_default_config(
    AerakiaStaticImuCalibrationConfig *config
)
{
    if (config == NULL) return;
    config->minimum_pose_count = 6U;
    config->gravity_m_s2 = AERAKIA_GRAVITY_M_S2;
    config->maximum_stationary_gyro_norm_rad_s = 0.05f;
    config->minimum_geometry_eigenvalue_ratio = 0.02f;
    config->minimum_direction_coverage_eigenvalue = 0.05f;
    config->maximum_gravity_residual_m_s2 = 0.10f;
    config->maximum_gravity_residual_rms_m_s2 = 0.05f;
    config->maximum_gyroscope_residual_rms_rad_s = 0.01f;
    /* This catches a material single-pose influence under the six-pose protocol. */
    config->maximum_leave_one_out_bias_delta_m_s2 = 0.02f;
}

AerakiaStaticImuCalibrationStatus aerakia_static_imu_calibrate(
    const AerakiaStaticImuPoseMean *poses,
    uint32_t pose_count,
    const AerakiaStaticImuCalibrationConfig *requested_config,
    AerakiaStaticImuCalibrationResult *result
)
{
    AerakiaStaticImuCalibrationConfig default_config;
    const AerakiaStaticImuCalibrationConfig *config = requested_config;
    double geometry_eigenvalues[3];
    double direction_coverage[3][3] = {{0.0}};
    double coverage_eigenvalues[3];
    double bias[3];
    double gyro_bias[3] = {0.0, 0.0, 0.0};
    double residual_sum_squared = 0.0;
    double gyro_residual_sum_squared = 0.0;
    double maximum_residual = 0.0;
    uint32_t first;
    int axis;

    if (result == NULL) return AERAKIA_STATIC_IMU_CALIBRATION_INVALID_ARGUMENT;
    memset(result, 0, sizeof(*result));
    result->status = AERAKIA_STATIC_IMU_CALIBRATION_INVALID_ARGUMENT;
    result->pose_count = pose_count;
    if (config == NULL) {
        aerakia_static_imu_calibration_default_config(&default_config);
        config = &default_config;
    }
    if (poses == NULL || !config_is_valid(config)) {
        return result->status;
    }
    if (pose_count < config->minimum_pose_count) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_INSUFFICIENT_POSES;
        return result->status;
    }
    if (pose_count > AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_TOO_MANY_POSES;
        return result->status;
    }

    for (first = 0; first < pose_count; ++first) {
        if (poses[first].sample_count == 0U
            || !vector_is_finite(poses[first].acceleration_m_s2)
            || !vector_is_finite(poses[first].angular_rate_rad_s)) {
            result->status = AERAKIA_STATIC_IMU_CALIBRATION_NONFINITE_INPUT;
            return result->status;
        }
        if (vector_norm(poses[first].angular_rate_rad_s)
            > (double)config->maximum_stationary_gyro_norm_rad_s) {
            result->status = AERAKIA_STATIC_IMU_CALIBRATION_NONSTATIONARY_INPUT;
            return result->status;
        }
        gyro_bias[0] += poses[first].angular_rate_rad_s.x;
        gyro_bias[1] += poses[first].angular_rate_rad_s.y;
        gyro_bias[2] += poses[first].angular_rate_rad_s.z;
    }
    for (axis = 0; axis < 3; ++axis) {
        gyro_bias[axis] /= (double)pose_count;
    }

    if (!fit_accelerometer_bias(
            poses, pose_count, -1, config, bias, geometry_eigenvalues
        )) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY;
        return result->status;
    }
    for (axis = 0; axis < 3; ++axis) {
        result->geometry_eigenvalues[axis] = (float)geometry_eigenvalues[axis];
    }

    for (first = 0; first < pose_count; ++first) {
        const AerakiaVec3f acceleration = poses[first].acceleration_m_s2;
        const AerakiaVec3f angular_rate = poses[first].angular_rate_rad_s;
        const double corrected[3] = {
            (double)acceleration.x - bias[0],
            (double)acceleration.y - bias[1],
            (double)acceleration.z - bias[2]
        };
        const double corrected_norm = sqrt(
            corrected[0] * corrected[0]
            + corrected[1] * corrected[1]
            + corrected[2] * corrected[2]
        );
        const double residual = corrected_norm - config->gravity_m_s2;
        const double gyro_residual[3] = {
            (double)angular_rate.x - gyro_bias[0],
            (double)angular_rate.y - gyro_bias[1],
            (double)angular_rate.z - gyro_bias[2]
        };
        int row;
        int column;
        if (!isfinite(corrected_norm) || corrected_norm <= 1.0e-12) {
            result->status = AERAKIA_STATIC_IMU_CALIBRATION_GRAVITY_RESIDUAL_EXCEEDED;
            return result->status;
        }
        residual_sum_squared += residual * residual;
        if (fabs(residual) > maximum_residual) maximum_residual = fabs(residual);
        gyro_residual_sum_squared += gyro_residual[0] * gyro_residual[0]
            + gyro_residual[1] * gyro_residual[1]
            + gyro_residual[2] * gyro_residual[2];
        for (row = 0; row < 3; ++row) {
            for (column = 0; column < 3; ++column) {
                direction_coverage[row][column] += corrected[row] * corrected[column]
                    / (corrected_norm * corrected_norm * (double)pose_count);
            }
        }
    }
    result->gravity_residual_rms_m_s2 = (float)sqrt(residual_sum_squared / pose_count);
    result->gravity_residual_max_abs_m_s2 = (float)maximum_residual;
    result->gyroscope_residual_rms_rad_s =
        (float)sqrt(gyro_residual_sum_squared / (3.0 * pose_count));
    result->accelerometer_bias_m_s2.x = (float)bias[0];
    result->accelerometer_bias_m_s2.y = (float)bias[1];
    result->accelerometer_bias_m_s2.z = (float)bias[2];
    result->gyroscope_bias_rad_s.x = (float)gyro_bias[0];
    result->gyroscope_bias_rad_s.y = (float)gyro_bias[1];
    result->gyroscope_bias_rad_s.z = (float)gyro_bias[2];

    if (!symmetric_eigenvalues_3x3(direction_coverage, coverage_eigenvalues)) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY;
        return result->status;
    }
    for (axis = 0; axis < 3; ++axis) {
        result->direction_coverage_eigenvalues[axis] = (float)coverage_eigenvalues[axis];
    }
    if (coverage_eigenvalues[0] < (double)config->minimum_direction_coverage_eigenvalue) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY;
        return result->status;
    }
    if (result->gravity_residual_max_abs_m_s2 > config->maximum_gravity_residual_m_s2
        || result->gravity_residual_rms_m_s2 > config->maximum_gravity_residual_rms_m_s2) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_GRAVITY_RESIDUAL_EXCEEDED;
        return result->status;
    }
    if (result->gyroscope_residual_rms_rad_s
        > config->maximum_gyroscope_residual_rms_rad_s) {
        result->status = AERAKIA_STATIC_IMU_CALIBRATION_GYROSCOPE_RESIDUAL_EXCEEDED;
        return result->status;
    }

    if (pose_count >= 6U) {
        for (first = 0; first < pose_count; ++first) {
            double leave_one_out_bias[3];
            double leave_one_out_eigenvalues[3];
            double delta_norm;
            if (!fit_accelerometer_bias(
                    poses, pose_count, (int)first, config,
                    leave_one_out_bias, leave_one_out_eigenvalues
                )) {
                result->status = AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY;
                return result->status;
            }
            delta_norm = sqrt(
                (leave_one_out_bias[0] - bias[0]) * (leave_one_out_bias[0] - bias[0])
                + (leave_one_out_bias[1] - bias[1]) * (leave_one_out_bias[1] - bias[1])
                + (leave_one_out_bias[2] - bias[2]) * (leave_one_out_bias[2] - bias[2])
            );
            if (delta_norm > result->maximum_leave_one_out_bias_delta_m_s2) {
                result->maximum_leave_one_out_bias_delta_m_s2 = (float)delta_norm;
            }
        }
        if (result->maximum_leave_one_out_bias_delta_m_s2
            > config->maximum_leave_one_out_bias_delta_m_s2) {
            result->status = AERAKIA_STATIC_IMU_CALIBRATION_OUTLIER_SENSITIVITY_EXCEEDED;
            return result->status;
        }
    }

    result->accepted = true;
    result->status = AERAKIA_STATIC_IMU_CALIBRATION_OK;
    return result->status;
}
