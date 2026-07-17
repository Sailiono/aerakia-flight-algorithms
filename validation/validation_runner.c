/**
 * @file validation_runner.c
 * @brief Replay hardware-neutral CSV data through comparable estimators.
 */

#include <aerakia/eskf_adapter.h>
#include <aerakia/mahony.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_LINE_LENGTH 16384
#define MAX_COLUMNS 128

typedef struct {
    int seq, ts_us;
    int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
    int mag_valid, mag_update, magnetic_declination;
    int truth_roll, truth_pitch, truth_yaw;
    int ref_q_w, ref_q_x, ref_q_y, ref_q_z;
    int position_ref_valid, ref_position_n, ref_position_e, ref_position_d;
    int ref_velocity_n, ref_velocity_e, ref_velocity_d;
    int position_update, gps_position_n, gps_position_e, gps_position_d;
    int gps_velocity_n, gps_velocity_e, gps_velocity_d;
    int gps_position_variance, gps_velocity_variance;
    int heading_valid, heading_update, heading, heading_variance;
    int course_valid, course_update, course, course_variance, ground_speed;
    int gsf_yaw_valid, gsf_yaw_update, gsf_yaw, gsf_yaw_variance;
    int baro_update, baro_height, baro_variance, static_hint;
    int reset_counter, reset_event;
    int reset_q_w, reset_q_x, reset_q_y, reset_q_z;
} ColumnMap;

static int split_csv(char *line, char *columns[], int maximum_columns)
{
    int count = 0;
    char *cursor = line;
    while (count < maximum_columns) {
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

static int find_column(char *columns[], int count, const char *name)
{
    int index;
    for (index = 0; index < count; ++index) {
        if (strcmp(columns[index], name) == 0) return index;
    }
    return -1;
}

#define MAP(name, column) map->name = find_column(columns, count, column)

static int load_column_map(char *header, ColumnMap *map)
{
    char *columns[MAX_COLUMNS];
    const int count = split_csv(header, columns, MAX_COLUMNS);
    memset(map, -1, sizeof(*map));
    MAP(seq, "seq"); MAP(ts_us, "ts_us");
    MAP(acc_x, "raw_acc_mg_x"); MAP(acc_y, "raw_acc_mg_y"); MAP(acc_z, "raw_acc_mg_z");
    MAP(gyro_x, "raw_gyro_mdps_x"); MAP(gyro_y, "raw_gyro_mdps_y"); MAP(gyro_z, "raw_gyro_mdps_z");
    MAP(mag_x, "raw_mag_cuT_x"); MAP(mag_y, "raw_mag_cuT_y"); MAP(mag_z, "raw_mag_cuT_z");
    MAP(mag_valid, "mag_valid"); MAP(mag_update, "mag_update");
    MAP(magnetic_declination, "magnetic_declination_rad");
    MAP(truth_roll, "roll_mdeg"); MAP(truth_pitch, "pitch_mdeg"); MAP(truth_yaw, "yaw_mdeg");
    MAP(ref_q_w, "ref_q_w"); MAP(ref_q_x, "ref_q_x"); MAP(ref_q_y, "ref_q_y"); MAP(ref_q_z, "ref_q_z");
    MAP(position_ref_valid, "position_ref_valid");
    MAP(ref_position_n, "ref_position_n_m"); MAP(ref_position_e, "ref_position_e_m");
    MAP(ref_position_d, "ref_position_d_m");
    MAP(ref_velocity_n, "ref_velocity_n_m_s"); MAP(ref_velocity_e, "ref_velocity_e_m_s");
    MAP(ref_velocity_d, "ref_velocity_d_m_s");
    MAP(position_update, "position_update");
    MAP(gps_position_n, "gps_position_n_m"); MAP(gps_position_e, "gps_position_e_m");
    MAP(gps_position_d, "gps_position_d_m");
    MAP(gps_velocity_n, "gps_velocity_n_m_s"); MAP(gps_velocity_e, "gps_velocity_e_m_s");
    MAP(gps_velocity_d, "gps_velocity_d_m_s");
    MAP(gps_position_variance, "gps_position_variance_m2");
    MAP(gps_velocity_variance, "gps_velocity_variance_m2_s2");
    MAP(heading_valid, "gnss_heading_valid"); MAP(heading_update, "gnss_heading_update");
    MAP(heading, "gnss_heading_rad"); MAP(heading_variance, "gnss_heading_variance_rad2");
    MAP(course_valid, "gnss_course_valid"); MAP(course_update, "gnss_course_update");
    MAP(course, "gnss_course_rad"); MAP(course_variance, "gnss_course_variance_rad");
    MAP(ground_speed, "gnss_ground_speed_m_s");
    MAP(gsf_yaw_valid, "px4_gsf_yaw_valid"); MAP(gsf_yaw_update, "px4_gsf_yaw_update");
    MAP(gsf_yaw, "px4_gsf_yaw_rad");
    MAP(gsf_yaw_variance, "px4_gsf_yaw_variance_rad2");
    MAP(baro_update, "baro_update"); MAP(baro_height, "baro_height_up_m");
    MAP(baro_variance, "baro_variance_m2"); MAP(static_hint, "static_hint");
    MAP(reset_counter, "ref_attitude_reset_counter"); MAP(reset_event, "ref_attitude_reset_event");
    MAP(reset_q_w, "ref_delta_q_reset_w"); MAP(reset_q_x, "ref_delta_q_reset_x");
    MAP(reset_q_y, "ref_delta_q_reset_y"); MAP(reset_q_z, "ref_delta_q_reset_z");
    return map->seq >= 0 && map->ts_us >= 0
        && map->acc_x >= 0 && map->acc_y >= 0 && map->acc_z >= 0
        && map->gyro_x >= 0 && map->gyro_y >= 0 && map->gyro_z >= 0
        && map->mag_x >= 0 && map->mag_y >= 0 && map->mag_z >= 0
        && map->truth_roll >= 0 && map->truth_pitch >= 0 && map->truth_yaw >= 0;
}

static double parse_double(char *columns[], int count, int index, double fallback, int *ok)
{
    char *end;
    double value;
    if (index < 0) return fallback;
    if (index >= count) {
        *ok = 0;
        return fallback;
    }
    errno = 0;
    value = strtod(columns[index], &end);
    if (errno != 0 || end == columns[index] || (*end != '\0' && *end != '\r' && *end != '\n')) {
        *ok = 0;
        return fallback;
    }
    return value;
}

static uint64_t parse_uint64(char *columns[], int count, int index, int *ok)
{
    char *end;
    unsigned long long value;
    if (index < 0 || index >= count) {
        *ok = 0;
        return 0U;
    }
    errno = 0;
    value = strtoull(columns[index], &end, 10);
    if (errno != 0 || end == columns[index] || (*end != '\0' && *end != '\r' && *end != '\n')) *ok = 0;
    return (uint64_t)value;
}

static AerakiaVec3f parse_vector(
    char *columns[], int count, int x, int y, int z, double scale, int *ok
)
{
    AerakiaVec3f result;
    result.x = (float)(parse_double(columns, count, x, 0.0, ok) * scale);
    result.y = (float)(parse_double(columns, count, y, 0.0, ok) * scale);
    result.z = (float)(parse_double(columns, count, z, 0.0, ok) * scale);
    return result;
}

static void euler_to_quaternion(double roll, double pitch, double yaw, double q[4])
{
    const double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    const double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static void normalized_quaternion(char *columns[], int count, const ColumnMap *map,
                                  double roll, double pitch, double yaw, double q[4], int *ok)
{
    double norm;
    if (map->ref_q_w < 0) {
        euler_to_quaternion(roll, pitch, yaw, q);
        return;
    }
    q[0] = parse_double(columns, count, map->ref_q_w, 1.0, ok);
    q[1] = parse_double(columns, count, map->ref_q_x, 0.0, ok);
    q[2] = parse_double(columns, count, map->ref_q_y, 0.0, ok);
    q[3] = parse_double(columns, count, map->ref_q_z, 0.0, ok);
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm <= 1.0e-12) {
        *ok = 0;
        return;
    }
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}

static void body_to_ned(const double q[4], AerakiaVec3f body, double ned[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    ned[0] = (1.0 - 2.0 * (y * y + z * z)) * body.x
        + 2.0 * (x * y - w * z) * body.y + 2.0 * (x * z + w * y) * body.z;
    ned[1] = 2.0 * (x * y + w * z) * body.x
        + (1.0 - 2.0 * (x * x + z * z)) * body.y + 2.0 * (y * z - w * x) * body.z;
    ned[2] = 2.0 * (x * z - w * y) * body.x + 2.0 * (y * z + w * x) * body.y
        + (1.0 - 2.0 * (x * x + y * y)) * body.z;
}

static float radians_to_degrees(float radians)
{
    return radians * (180.0f / AERAKIA_PI_F);
}

static double navigation_nees(
    const AerakiaEskf *filter,
    const AerakiaVec3f reference_position,
    const AerakiaVec3f reference_velocity
)
{
    static const int state_indices[6] = {
        ESKF_IDX_DV, ESKF_IDX_DV + 1, ESKF_IDX_DV + 2,
        ESKF_IDX_DP, ESKF_IDX_DP + 1, ESKF_IDX_DP + 2
    };
    double augmented[6][12];
    double error[6];
    double result = 0.0;
    int row, column, pivot;

    error[0] = filter->core.state.v[0] - reference_velocity.x;
    error[1] = filter->core.state.v[1] - reference_velocity.y;
    error[2] = filter->core.state.v[2] - reference_velocity.z;
    error[3] = filter->core.state.p[0] - reference_position.x;
    error[4] = filter->core.state.p[1] - reference_position.y;
    error[5] = filter->core.state.p[2] - reference_position.z;

    for (row = 0; row < 6; ++row) {
        for (column = 0; column < 6; ++column) {
            augmented[row][column] = filter->core.P[state_indices[row]][state_indices[column]];
            augmented[row][column + 6] = row == column ? 1.0 : 0.0;
        }
    }
    for (column = 0; column < 6; ++column) {
        double largest = fabs(augmented[column][column]);
        pivot = column;
        for (row = column + 1; row < 6; ++row) {
            const double candidate = fabs(augmented[row][column]);
            if (candidate > largest) {
                largest = candidate;
                pivot = row;
            }
        }
        if (largest < 1.0e-15) return NAN;
        if (pivot != column) {
            int entry;
            for (entry = 0; entry < 12; ++entry) {
                const double temporary = augmented[column][entry];
                augmented[column][entry] = augmented[pivot][entry];
                augmented[pivot][entry] = temporary;
            }
        }
        {
            const double scale = augmented[column][column];
            int entry;
            for (entry = 0; entry < 12; ++entry) augmented[column][entry] /= scale;
        }
        for (row = 0; row < 6; ++row) {
            int entry;
            const double scale = augmented[row][column];
            if (row == column) continue;
            for (entry = 0; entry < 12; ++entry) {
                augmented[row][entry] -= scale * augmented[column][entry];
            }
        }
    }
    for (row = 0; row < 6; ++row) {
        double inverse_times_error = 0.0;
        for (column = 0; column < 6; ++column) {
            inverse_times_error += augmented[row][column + 6] * error[column];
        }
        result += error[row] * inverse_times_error;
    }
    return result >= 0.0 && isfinite(result) ? result : NAN;
}

int main(int argc, char *argv[])
{
    FILE *input, *output;
    char line[MAX_LINE_LENGTH];
    ColumnMap map;
    AerakiaMahony standard, robust;
    AerakiaMahonyConfig standard_config, robust_config;
    AerakiaEskf eskf;
    AerakiaEskfConfig eskf_config;
    AerakiaAttitudeEstimate standard_estimate, robust_estimate;
    AerakiaNavigationEstimate eskf_estimate;
    unsigned long samples = 0U, malformed = 0U, gps_updates = 0U, heading_updates = 0U;
    unsigned long zupt_updates = 0U;
    int eskf_initialized = 0, mag_reference_initialized = 0, mahony_reference_seeded = 0;
    int cold_start = 0;
    int reference_attitude_init = 0;
    int input_argument;
    int output_argument;
    int argument;
    clock_t start_clock;

    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s [--cold-start|--reference-attitude-init] "
                "INPUT_REPLAY_CSV OUTPUT_RESULTS_CSV\n",
                argv[0]);
        return 2;
    }
    input_argument = argc - 2;
    output_argument = argc - 1;
    for (argument = 1; argument < input_argument; ++argument) {
        if (strcmp(argv[argument], "--cold-start") == 0) {
            cold_start = 1;
        } else if (strcmp(argv[argument], "--reference-attitude-init") == 0) {
            reference_attitude_init = 1;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[argument]);
            return 2;
        }
    }
    if (cold_start && reference_attitude_init) {
        fputs("--cold-start and --reference-attitude-init are mutually exclusive\n", stderr);
        return 2;
    }
    input = fopen(argv[input_argument], "r");
    if (input == NULL) { perror("open input"); return 2; }
    output = fopen(argv[output_argument], "w");
    if (output == NULL) { perror("open output"); fclose(input); return 2; }
    if (fgets(line, sizeof(line), input) == NULL || !load_column_map(line, &map)) {
        fputs("Input does not satisfy the replay CSV contract\n", stderr);
        fclose(input); fclose(output); return 2;
    }

    aerakia_mahony_default_config(&standard_config);
    standard_config.adaptive_accelerometer = false;
    standard_config.yaw_only_magnetometer = false;
    standard_config.gate_magnetometer = false;
    standard_config.magnetometer_weight = 1.0f;
    aerakia_mahony_init(&standard, &standard_config);
    aerakia_mahony_default_config(&robust_config);
    aerakia_mahony_init(&robust, &robust_config);
    aerakia_eskf_default_config(&eskf_config);
    eskf_config.fuse_magnetometer = true;

    fputs(
        "seq,ts_us,truth_roll_deg,truth_pitch_deg,truth_yaw_deg,"
        "mahony_standard_roll_deg,mahony_standard_pitch_deg,mahony_standard_yaw_deg,"
        "mahony_robust_roll_deg,mahony_robust_pitch_deg,mahony_robust_yaw_deg,"
        "eskf_roll_deg,eskf_pitch_deg,eskf_yaw_deg,"
        "mahony_standard_acc_weight,mahony_robust_acc_weight,mahony_robust_mag_weight,"
        "eskf_mag_accepted,eskf_mag_innovation_rad,eskf_mag_test_ratio,"
        "eskf_position_accepted,eskf_velocity_accepted,eskf_navigation_recovered,"
        "eskf_navigation_recovery_count,eskf_healthy,eskf_static_aligned,"
        "eskf_static_tilt_aligned,eskf_static_heading_aligned,"
        "eskf_stationary_detected,eskf_zupt_applied,eskf_zupt_count,"
        "input_mag_update,input_position_update,position_ref_valid,"
        "ref_attitude_reset_counter,ref_attitude_reset_event,"
        "ref_delta_q_reset_w,ref_delta_q_reset_x,ref_delta_q_reset_y,ref_delta_q_reset_z,"
        "input_heading_update,eskf_heading_accepted,eskf_heading_innovation_rad,"
        "gnss_heading_valid,gnss_heading_rad,gnss_heading_variance_rad2,"
        "gnss_course_valid,gnss_course_update,gnss_course_rad,gnss_course_variance_rad,"
        "gnss_ground_speed_m_s,px4_gsf_yaw_valid,px4_gsf_yaw_update,"
        "px4_gsf_yaw_rad,px4_gsf_yaw_variance_rad2,"
        "ref_position_n_m,ref_position_e_m,ref_position_d_m,"
        "ref_velocity_n_m_s,ref_velocity_e_m_s,ref_velocity_d_m_s,"
        "eskf_position_n_m,eskf_position_e_m,eskf_position_d_m,"
        "eskf_velocity_n_m_s,eskf_velocity_e_m_s,eskf_velocity_d_m_s,"
        "eskf_position_nis,eskf_velocity_nis,eskf_navigation_nees,"
        "truth_q_w,truth_q_x,truth_q_y,truth_q_z,"
        "mahony_standard_q_w,mahony_standard_q_x,mahony_standard_q_y,mahony_standard_q_z,"
        "mahony_robust_q_w,mahony_robust_q_x,mahony_robust_q_y,mahony_robust_q_z,"
        "eskf_q_w,eskf_q_x,eskf_q_y,eskf_q_z\n",
        output
    );

    start_clock = clock();
    while (fgets(line, sizeof(line), input) != NULL) {
        char *columns[MAX_COLUMNS];
        const int count = split_csv(line, columns, MAX_COLUMNS);
        int ok = 1;
        const long sequence = (long)parse_double(columns, count, map.seq, 0.0, &ok);
        const uint64_t timestamp_us = parse_uint64(columns, count, map.ts_us, &ok);
        const double truth_roll = parse_double(columns, count, map.truth_roll, 0.0, &ok) * 0.001;
        const double truth_pitch = parse_double(columns, count, map.truth_pitch, 0.0, &ok) * 0.001;
        const double truth_yaw = parse_double(columns, count, map.truth_yaw, 0.0, &ok) * 0.001;
        const AerakiaVec3f acceleration = parse_vector(
            columns, count, map.acc_x, map.acc_y, map.acc_z, AERAKIA_GRAVITY_M_S2 / 1000.0, &ok
        );
        const AerakiaVec3f angular_rate = parse_vector(
            columns, count, map.gyro_x, map.gyro_y, map.gyro_z, AERAKIA_PI_F / 180000.0, &ok
        );
        const AerakiaVec3f magnetic = parse_vector(
            columns, count, map.mag_x, map.mag_y, map.mag_z, 0.01, &ok
        );
        const int mag_valid = (int)parse_double(columns, count, map.mag_valid, 1.0, &ok) != 0;
        const int mag_update = (int)parse_double(columns, count, map.mag_update, 1.0, &ok) != 0;
        const double magnetic_declination_rad = parse_double(
            columns, count, map.magnetic_declination, 0.0, &ok
        );
        const int position_update = (int)parse_double(columns, count, map.position_update, 0.0, &ok) != 0;
        const int heading_valid = (int)parse_double(
            columns, count, map.heading_valid, 0.0, &ok
        ) != 0;
        const int heading_update = heading_valid && ((int)parse_double(
            columns, count, map.heading_update, 0.0, &ok
        ) != 0);
        const double heading_rad = parse_double(columns, count, map.heading, 0.0, &ok);
        const double heading_variance = parse_double(
            columns, count, map.heading_variance, 1.0, &ok
        );
        const int course_valid = (int)parse_double(
            columns, count, map.course_valid, 0.0, &ok
        ) != 0;
        const int course_update = (int)parse_double(
            columns, count, map.course_update, 0.0, &ok
        ) != 0;
        const double course_rad = parse_double(columns, count, map.course, 0.0, &ok);
        const double course_variance = parse_double(
            columns, count, map.course_variance, 1.0, &ok
        );
        const double ground_speed = parse_double(columns, count, map.ground_speed, 0.0, &ok);
        const int gsf_yaw_valid = (int)parse_double(
            columns, count, map.gsf_yaw_valid, 0.0, &ok
        ) != 0;
        const int gsf_yaw_update = (int)parse_double(
            columns, count, map.gsf_yaw_update, 0.0, &ok
        ) != 0;
        const double gsf_yaw_rad = parse_double(columns, count, map.gsf_yaw, 0.0, &ok);
        const double gsf_yaw_variance = parse_double(
            columns, count, map.gsf_yaw_variance, 1.0, &ok
        );
        const int static_hint = (int)parse_double(columns, count, map.static_hint, 0.0, &ok) != 0;
        const int position_ref_valid = (int)parse_double(
            columns, count, map.position_ref_valid, 0.0, &ok
        ) != 0;
        const AerakiaVec3f reference_position = parse_vector(
            columns, count, map.ref_position_n, map.ref_position_e, map.ref_position_d, 1.0, &ok
        );
        const AerakiaVec3f reference_velocity = parse_vector(
            columns, count, map.ref_velocity_n, map.ref_velocity_e, map.ref_velocity_d, 1.0, &ok
        );
        double reference_q[4];
        AerakiaImuSample sample;

        normalized_quaternion(
            columns, count, &map, truth_roll * AERAKIA_PI_F / 180.0,
            truth_pitch * AERAKIA_PI_F / 180.0, truth_yaw * AERAKIA_PI_F / 180.0,
            reference_q, &ok
        );
        if (!ok) { malformed++; continue; }
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_us = timestamp_us;
        sample.acceleration_m_s2 = acceleration;
        sample.angular_rate_rad_s = angular_rate;
        sample.magnetic_field_ut = magnetic;
        sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID;
        if (mag_valid && mag_update) sample.flags |= AERAKIA_SAMPLE_MAG_VALID;
        if (static_hint) sample.flags |= AERAKIA_SAMPLE_STATIONARY;

        if (cold_start && !mag_reference_initialized) {
            eskf_config.magnetic_reference_ned[0] = (float)cos(magnetic_declination_rad);
            eskf_config.magnetic_reference_ned[1] = (float)sin(magnetic_declination_rad);
            eskf_config.magnetic_reference_ned[2] = 0.0f;
            mag_reference_initialized = 1;
        }
        if (!cold_start && !mag_reference_initialized && mag_valid && mag_update) {
            double reference_ned[3];
            body_to_ned(reference_q, magnetic, reference_ned);
            eskf_config.magnetic_reference_ned[0] = (float)reference_ned[0];
            eskf_config.magnetic_reference_ned[1] = (float)reference_ned[1];
            eskf_config.magnetic_reference_ned[2] = (float)reference_ned[2];
            if (eskf_initialized) {
                eskf_float_t core_reference[3] = {
                    reference_ned[0], reference_ned[1], reference_ned[2]
                };
                eskf_set_mag_reference(&eskf.core, core_reference);
            }
            mag_reference_initialized = 1;
        }
        if (!eskf_initialized) {
            aerakia_eskf_init(
                &eskf, &eskf_config, NULL, cold_start ? NULL : reference_q
            );
            eskf_initialized = 1;
        }
        if (reference_attitude_init && !mahony_reference_seeded) {
            const float mahony_seed[4] = {
                (float)reference_q[0], (float)reference_q[1],
                (float)reference_q[2], (float)reference_q[3]
            };
            if (aerakia_mahony_seed_attitude(&standard, mahony_seed)
                    != AERAKIA_STATUS_INITIALIZED
                || aerakia_mahony_seed_attitude(&robust, mahony_seed)
                    != AERAKIA_STATUS_INITIALIZED) {
                fputs("Invalid reference attitude seed\n", stderr);
                fclose(input); fclose(output); return 2;
            }
            mahony_reference_seeded = 1;
        }

        (void)aerakia_mahony_update(&standard, &sample, &standard_estimate);
        (void)aerakia_mahony_update(&robust, &sample, &robust_estimate);
        (void)aerakia_eskf_process_imu(&eskf, &sample, &eskf_estimate);

        if (position_update) {
            const AerakiaVec3f gps_position = parse_vector(
                columns, count, map.gps_position_n, map.gps_position_e, map.gps_position_d, 1.0, &ok
            );
            const AerakiaVec3f gps_velocity = parse_vector(
                columns, count, map.gps_velocity_n, map.gps_velocity_e, map.gps_velocity_d, 1.0, &ok
            );
            const float position_variance = (float)parse_double(
                columns, count, map.gps_position_variance, 1.0, &ok
            );
            const float velocity_variance = (float)parse_double(
                columns, count, map.gps_velocity_variance, 1.0, &ok
            );
            if (ok) {
                aerakia_eskf_update_gps(
                    &eskf, gps_position, gps_velocity, position_variance, velocity_variance
                );
                gps_updates++;
            }
        }
        if (heading_update && isfinite(heading_rad) && heading_variance > 0.0) {
            aerakia_eskf_update_heading(
                &eskf, (float)heading_rad, (float)heading_variance
            );
            heading_updates++;
        }
        if ((int)parse_double(columns, count, map.baro_update, 0.0, &ok) != 0) {
            aerakia_eskf_update_barometer(
                &eskf,
                (float)parse_double(columns, count, map.baro_height, 0.0, &ok),
                (float)parse_double(columns, count, map.baro_variance, 1.0, &ok)
            );
        }
        aerakia_eskf_get_estimate(&eskf, &eskf_estimate);
        if (eskf_estimate.zero_velocity_update_applied) zupt_updates++;

        fprintf(
            output,
            "%ld,%llu,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
            "%.6f,%.6f,%.6f,%d,%.9f,%.9f,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%u,"
            "%d,%d,%d,%.0f,%.0f,%.9f,%.9f,%.9f,%.9f,"
            "%d,%d,%.9f,%d,%.9f,%.9f,%d,%d,%.9f,%.9f,%.9f,%d,%d,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
            sequence, (unsigned long long)sample.timestamp_us, truth_roll, truth_pitch, truth_yaw,
            radians_to_degrees(standard_estimate.euler_rad.x),
            radians_to_degrees(standard_estimate.euler_rad.y),
            radians_to_degrees(standard_estimate.euler_rad.z),
            radians_to_degrees(robust_estimate.euler_rad.x),
            radians_to_degrees(robust_estimate.euler_rad.y),
            radians_to_degrees(robust_estimate.euler_rad.z),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.x),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.y),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.z),
            standard_estimate.accelerometer_weight, robust_estimate.accelerometer_weight,
            robust_estimate.magnetometer_weight, eskf_estimate.magnetometer_accepted ? 1 : 0,
            eskf_estimate.last_magnetometer_innovation.innovation[0],
            eskf_estimate.last_magnetometer_innovation.test_ratio,
            eskf_estimate.position_accepted ? 1 : 0, eskf_estimate.velocity_accepted ? 1 : 0,
            eskf_estimate.navigation_recovered ? 1 : 0, eskf_estimate.navigation_recovery_count,
            eskf_estimate.healthy ? 1 : 0, eskf_estimate.static_alignment_complete ? 1 : 0,
            eskf_estimate.static_tilt_alignment_complete ? 1 : 0,
            eskf_estimate.static_heading_alignment_complete ? 1 : 0,
            eskf_estimate.stationary_detected ? 1 : 0,
            eskf_estimate.zero_velocity_update_applied ? 1 : 0,
            eskf_estimate.zero_velocity_update_count, mag_update, position_update, position_ref_valid,
            parse_double(columns, count, map.reset_counter, 0.0, &ok),
            parse_double(columns, count, map.reset_event, 0.0, &ok),
            parse_double(columns, count, map.reset_q_w, 1.0, &ok),
            parse_double(columns, count, map.reset_q_x, 0.0, &ok),
            parse_double(columns, count, map.reset_q_y, 0.0, &ok),
            parse_double(columns, count, map.reset_q_z, 0.0, &ok),
            heading_update,
            heading_update && eskf_estimate.heading_accepted ? 1 : 0,
            heading_update ? eskf_estimate.last_heading_innovation.innovation[0] : 0.0,
            heading_valid, heading_rad, heading_variance,
            course_valid, course_update, course_rad, course_variance, ground_speed,
            gsf_yaw_valid, gsf_yaw_update, gsf_yaw_rad, gsf_yaw_variance,
            reference_position.x, reference_position.y, reference_position.z,
            reference_velocity.x, reference_velocity.y, reference_velocity.z,
            eskf_estimate.position_ned_m.x, eskf_estimate.position_ned_m.y,
            eskf_estimate.position_ned_m.z, eskf_estimate.velocity_ned_m_s.x,
            eskf_estimate.velocity_ned_m_s.y, eskf_estimate.velocity_ned_m_s.z,
            position_update ? eskf_estimate.last_position_innovation.nis : NAN,
            position_update ? eskf_estimate.last_velocity_innovation.nis : NAN,
            position_update && position_ref_valid
                ? navigation_nees(&eskf, reference_position, reference_velocity) : NAN,
            reference_q[0], reference_q[1], reference_q[2], reference_q[3],
            standard_estimate.quaternion_wxyz[0], standard_estimate.quaternion_wxyz[1],
            standard_estimate.quaternion_wxyz[2], standard_estimate.quaternion_wxyz[3],
            robust_estimate.quaternion_wxyz[0], robust_estimate.quaternion_wxyz[1],
            robust_estimate.quaternion_wxyz[2], robust_estimate.quaternion_wxyz[3],
            eskf_estimate.attitude.quaternion_wxyz[0],
            eskf_estimate.attitude.quaternion_wxyz[1],
            eskf_estimate.attitude.quaternion_wxyz[2],
            eskf_estimate.attitude.quaternion_wxyz[3]
        );
        samples++;
    }

    fprintf(
        stderr,
        "replayed=%lu malformed=%lu gps_updates=%lu heading_updates=%lu "
        "zupt_updates=%lu recoveries=%u cpu_ms=%.3f\n",
        samples, malformed, gps_updates, heading_updates, zupt_updates,
        eskf_initialized ? eskf.navigation_recovery_count : 0U,
        (double)(clock() - start_clock) * 1000.0 / (double)CLOCKS_PER_SEC
    );
    fclose(input); fclose(output);
    return samples > 0U ? 0 : 3;
}
