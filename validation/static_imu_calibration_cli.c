/**
 * @file static_imu_calibration_cli.c
 * @brief Host utility converting stationary IMU pose means into an auditable seed.
 */

#include <aerakia/static_imu_calibration.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define POSE_COLUMNS 7

static const char *const EXPECTED_HEADER[POSE_COLUMNS] = {
    "acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2",
    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "sample_count"
};

static int parse_number(const char *text, double *value)
{
    char *end;
    if (text == NULL || value == NULL || text[0] == '\0') return 0;
    errno = 0;
    *value = strtod(text, &end);
    return errno == 0 && end != text && *end == '\0' && isfinite(*value);
}

static int split_csv(char *line, char *columns[], int maximum)
{
    int count = 0;
    char *cursor = line;
    while (count < maximum) {
        char *comma;
        columns[count++] = cursor;
        comma = strchr(cursor, ',');
        if (comma == NULL) break;
        *comma = '\0';
        cursor = comma + 1;
    }
    if (count > 0) columns[count - 1][strcspn(columns[count - 1], "\r\n")] = '\0';
    return count;
}

static int header_is_expected(char *line)
{
    char *columns[POSE_COLUMNS];
    int index;
    if (split_csv(line, columns, POSE_COLUMNS) != POSE_COLUMNS) return 0;
    for (index = 0; index < POSE_COLUMNS; ++index) {
        if (strcmp(columns[index], EXPECTED_HEADER[index]) != 0) return 0;
    }
    return 1;
}

int main(int argc, char **argv)
{
    FILE *input;
    FILE *output;
    char line[4096];
    AerakiaStaticImuPoseMean poses[AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES];
    AerakiaStaticImuCalibrationConfig config;
    AerakiaStaticImuCalibrationResult result;
    AerakiaStaticImuCalibrationStatus status;
    uint32_t pose_count = 0U;

    if (argc != 3) {
        fprintf(stderr, "usage: %s POSE_MEANS.csv RESULT.json\n", argv[0]);
        return 2;
    }
    input = fopen(argv[1], "r");
    output = fopen(argv[2], "w");
    if (input == NULL || output == NULL) {
        perror("open calibration file");
        if (input != NULL) fclose(input);
        if (output != NULL) fclose(output);
        return 2;
    }
    if (fgets(line, sizeof(line), input) == NULL || !header_is_expected(line)) {
        fputs("missing pose-means header\n", stderr);
        fclose(input);
        fclose(output);
        return 2;
    }
    while (fgets(line, sizeof(line), input) != NULL) {
        char *columns[POSE_COLUMNS];
        double values[POSE_COLUMNS];
        int index;
        if (pose_count >= AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES
            || split_csv(line, columns, POSE_COLUMNS) != POSE_COLUMNS) {
            fputs("invalid or excessive pose mean\n", stderr);
            fclose(input);
            fclose(output);
            return 2;
        }
        for (index = 0; index < POSE_COLUMNS; ++index) {
            if (!parse_number(columns[index], &values[index])) {
                fputs("non-finite pose mean\n", stderr);
                fclose(input);
                fclose(output);
                return 2;
            }
        }
        poses[pose_count].acceleration_m_s2.x = (float)values[0];
        poses[pose_count].acceleration_m_s2.y = (float)values[1];
        poses[pose_count].acceleration_m_s2.z = (float)values[2];
        poses[pose_count].angular_rate_rad_s.x = (float)values[3];
        poses[pose_count].angular_rate_rad_s.y = (float)values[4];
        poses[pose_count].angular_rate_rad_s.z = (float)values[5];
        if (values[6] < 1.0 || values[6] > (double)UINT32_MAX
            || floor(values[6]) != values[6]) {
            fputs("invalid pose sample count\n", stderr);
            fclose(input);
            fclose(output);
            return 2;
        }
        poses[pose_count].sample_count = (uint32_t)values[6];
        pose_count++;
    }
    fclose(input);
    aerakia_static_imu_calibration_default_config(&config);
    status = aerakia_static_imu_calibrate(poses, pose_count, &config, &result);
    fprintf(output,
            "{\n  \"schema_version\": 1,\n  \"status\": %d,\n  \"accepted\": %s,\n"
            "  \"pose_count\": %u,\n"
            "  \"accelerometer_bias_m_s2\": [%.9g, %.9g, %.9g],\n"
            "  \"gyroscope_bias_rad_s\": [%.9g, %.9g, %.9g],\n"
            "  \"gravity_residual_rms_m_s2\": %.9g,\n"
            "  \"gravity_residual_max_abs_m_s2\": %.9g,\n"
            "  \"gyroscope_residual_rms_rad_s\": %.9g,\n"
            "  \"geometry_eigenvalues\": [%.9g, %.9g, %.9g],\n"
            "  \"direction_coverage_eigenvalues\": [%.9g, %.9g, %.9g],\n"
            "  \"maximum_leave_one_out_bias_delta_m_s2\": %.9g\n}\n",
            (int)status, result.accepted ? "true" : "false", result.pose_count,
            result.accelerometer_bias_m_s2.x, result.accelerometer_bias_m_s2.y,
            result.accelerometer_bias_m_s2.z, result.gyroscope_bias_rad_s.x,
            result.gyroscope_bias_rad_s.y, result.gyroscope_bias_rad_s.z,
            result.gravity_residual_rms_m_s2, result.gravity_residual_max_abs_m_s2,
            result.gyroscope_residual_rms_rad_s, result.geometry_eigenvalues[0],
            result.geometry_eigenvalues[1], result.geometry_eigenvalues[2],
            result.direction_coverage_eigenvalues[0], result.direction_coverage_eigenvalues[1],
            result.direction_coverage_eigenvalues[2],
            result.maximum_leave_one_out_bias_delta_m_s2);
    fclose(output);
    return status == AERAKIA_STATIC_IMU_CALIBRATION_OK ? 0 : 1;
}
