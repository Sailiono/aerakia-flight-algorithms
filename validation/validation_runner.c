/**
 * @file validation_runner.c
 * @brief Replay a golden CSV through comparable estimator configurations.
 */

#include <aerakia/eskf_adapter.h>
#include <aerakia/mahony.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_LINE_LENGTH 8192
#define MAX_COLUMNS 96

typedef struct {
    int seq;
    int ts_us;
    int acc_x;
    int acc_y;
    int acc_z;
    int gyro_x;
    int gyro_y;
    int gyro_z;
    int mag_x;
    int mag_y;
    int mag_z;
    int truth_roll;
    int truth_pitch;
    int truth_yaw;
} ColumnMap;

static int split_csv(char *line, char *columns[], int maximum_columns)
{
    int count = 0;
    char *cursor = line;
    while (count < maximum_columns) {
        char *comma;
        columns[count++] = cursor;
        comma = strchr(cursor, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }
    if (count > 0) {
        columns[count - 1][strcspn(columns[count - 1], "\r\n")] = '\0';
    }
    return count;
}

static int find_column(char *columns[], int count, const char *name)
{
    int index;
    for (index = 0; index < count; ++index) {
        if (strcmp(columns[index], name) == 0) {
            return index;
        }
    }
    return -1;
}

static int load_column_map(char *header, ColumnMap *map)
{
    char *columns[MAX_COLUMNS];
    const int count = split_csv(header, columns, MAX_COLUMNS);
    map->seq = find_column(columns, count, "seq");
    map->ts_us = find_column(columns, count, "ts_us");
    map->acc_x = find_column(columns, count, "raw_acc_mg_x");
    map->acc_y = find_column(columns, count, "raw_acc_mg_y");
    map->acc_z = find_column(columns, count, "raw_acc_mg_z");
    map->gyro_x = find_column(columns, count, "raw_gyro_mdps_x");
    map->gyro_y = find_column(columns, count, "raw_gyro_mdps_y");
    map->gyro_z = find_column(columns, count, "raw_gyro_mdps_z");
    map->mag_x = find_column(columns, count, "raw_mag_cuT_x");
    map->mag_y = find_column(columns, count, "raw_mag_cuT_y");
    map->mag_z = find_column(columns, count, "raw_mag_cuT_z");
    map->truth_roll = find_column(columns, count, "roll_mdeg");
    map->truth_pitch = find_column(columns, count, "pitch_mdeg");
    map->truth_yaw = find_column(columns, count, "yaw_mdeg");

    return map->seq >= 0 && map->ts_us >= 0
        && map->acc_x >= 0 && map->acc_y >= 0 && map->acc_z >= 0
        && map->gyro_x >= 0 && map->gyro_y >= 0 && map->gyro_z >= 0
        && map->mag_x >= 0 && map->mag_y >= 0 && map->mag_z >= 0
        && map->truth_roll >= 0 && map->truth_pitch >= 0 && map->truth_yaw >= 0;
}

static long parse_long(char *columns[], int count, int index, int *ok)
{
    char *end;
    long value;
    if (index < 0 || index >= count) {
        *ok = 0;
        return 0;
    }
    errno = 0;
    value = strtol(columns[index], &end, 10);
    if (errno != 0 || end == columns[index] || (*end != '\0' && *end != '\r' && *end != '\n')) {
        *ok = 0;
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
    if (errno != 0 || end == columns[index] || (*end != '\0' && *end != '\r' && *end != '\n')) {
        *ok = 0;
    }
    return (uint64_t)value;
}

static AerakiaImuSample parse_sample(char *columns[], int count, const ColumnMap *map, int *ok)
{
    AerakiaImuSample sample;
    const float mg_to_m_s2 = AERAKIA_GRAVITY_M_S2 / 1000.0f;
    const float mdps_to_rad_s = AERAKIA_PI_F / 180000.0f;
    memset(&sample, 0, sizeof(sample));

    sample.timestamp_us = parse_uint64(columns, count, map->ts_us, ok);
    sample.acceleration_m_s2.x = (float)parse_long(columns, count, map->acc_x, ok) * mg_to_m_s2;
    sample.acceleration_m_s2.y = (float)parse_long(columns, count, map->acc_y, ok) * mg_to_m_s2;
    sample.acceleration_m_s2.z = (float)parse_long(columns, count, map->acc_z, ok) * mg_to_m_s2;
    sample.angular_rate_rad_s.x = (float)parse_long(columns, count, map->gyro_x, ok) * mdps_to_rad_s;
    sample.angular_rate_rad_s.y = (float)parse_long(columns, count, map->gyro_y, ok) * mdps_to_rad_s;
    sample.angular_rate_rad_s.z = (float)parse_long(columns, count, map->gyro_z, ok) * mdps_to_rad_s;
    sample.magnetic_field_ut.x = (float)parse_long(columns, count, map->mag_x, ok) / 100.0f;
    sample.magnetic_field_ut.y = (float)parse_long(columns, count, map->mag_y, ok) / 100.0f;
    sample.magnetic_field_ut.z = (float)parse_long(columns, count, map->mag_z, ok) / 100.0f;
    sample.flags = AERAKIA_SAMPLE_ACCEL_VALID
        | AERAKIA_SAMPLE_GYRO_VALID
        | AERAKIA_SAMPLE_MAG_VALID;
    return sample;
}

static float radians_to_degrees(float radians)
{
    return radians * (180.0f / AERAKIA_PI_F);
}

int main(int argc, char *argv[])
{
    FILE *input;
    FILE *output;
    char line[MAX_LINE_LENGTH];
    ColumnMap map;
    AerakiaMahony standard;
    AerakiaMahony robust;
    AerakiaMahonyConfig standard_config;
    AerakiaMahonyConfig robust_config;
    AerakiaEskf eskf;
    AerakiaEskfConfig eskf_config;
    AerakiaAttitudeEstimate standard_estimate;
    AerakiaAttitudeEstimate robust_estimate;
    AerakiaNavigationEstimate eskf_estimate;
    unsigned long samples = 0U;
    unsigned long malformed = 0U;
    clock_t start_clock;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s INPUT_GOLDEN_CSV OUTPUT_RESULTS_CSV\n", argv[0]);
        return 2;
    }

    input = fopen(argv[1], "r");
    if (input == NULL) {
        perror("open input");
        return 2;
    }
    output = fopen(argv[2], "w");
    if (output == NULL) {
        perror("open output");
        fclose(input);
        return 2;
    }
    if (fgets(line, sizeof(line), input) == NULL || !load_column_map(line, &map)) {
        fputs("Input does not satisfy the golden CSV contract\n", stderr);
        fclose(input);
        fclose(output);
        return 2;
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
    eskf_config.magnetic_reference_ned[0] = 22.0f;
    eskf_config.magnetic_reference_ned[1] = 0.0f;
    eskf_config.magnetic_reference_ned[2] = 44.0f;
    aerakia_eskf_init(&eskf, &eskf_config, NULL, NULL);

    fputs(
        "seq,ts_us,truth_roll_deg,truth_pitch_deg,truth_yaw_deg,"
        "mahony_standard_roll_deg,mahony_standard_pitch_deg,mahony_standard_yaw_deg,"
        "mahony_robust_roll_deg,mahony_robust_pitch_deg,mahony_robust_yaw_deg,"
        "eskf_roll_deg,eskf_pitch_deg,eskf_yaw_deg,"
        "mahony_standard_acc_weight,mahony_robust_acc_weight,mahony_robust_mag_weight,"
        "eskf_mag_accepted,eskf_position_n_m,eskf_position_e_m,eskf_position_d_m\n",
        output
    );

    start_clock = clock();
    while (fgets(line, sizeof(line), input) != NULL) {
        char *columns[MAX_COLUMNS];
        const int count = split_csv(line, columns, MAX_COLUMNS);
        int ok = 1;
        const long sequence = parse_long(columns, count, map.seq, &ok);
        const float truth_roll = (float)parse_long(columns, count, map.truth_roll, &ok) / 1000.0f;
        const float truth_pitch = (float)parse_long(columns, count, map.truth_pitch, &ok) / 1000.0f;
        const float truth_yaw = (float)parse_long(columns, count, map.truth_yaw, &ok) / 1000.0f;
        const AerakiaImuSample sample = parse_sample(columns, count, &map, &ok);
        if (!ok) {
            malformed++;
            continue;
        }

        (void)aerakia_mahony_update(&standard, &sample, &standard_estimate);
        (void)aerakia_mahony_update(&robust, &sample, &robust_estimate);
        (void)aerakia_eskf_process_imu(&eskf, &sample, &eskf_estimate);

        fprintf(
            output,
            "%ld,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.9f,%.9f,%.9f\n",
            sequence,
            (unsigned long long)sample.timestamp_us,
            truth_roll, truth_pitch, truth_yaw,
            radians_to_degrees(standard_estimate.euler_rad.x),
            radians_to_degrees(standard_estimate.euler_rad.y),
            radians_to_degrees(standard_estimate.euler_rad.z),
            radians_to_degrees(robust_estimate.euler_rad.x),
            radians_to_degrees(robust_estimate.euler_rad.y),
            radians_to_degrees(robust_estimate.euler_rad.z),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.x),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.y),
            radians_to_degrees(eskf_estimate.attitude.euler_rad.z),
            standard_estimate.accelerometer_weight,
            robust_estimate.accelerometer_weight,
            robust_estimate.magnetometer_weight,
            eskf_estimate.magnetometer_accepted ? 1 : 0,
            eskf_estimate.position_ned_m.x,
            eskf_estimate.position_ned_m.y,
            eskf_estimate.position_ned_m.z
        );
        samples++;
    }

    fprintf(
        stderr,
        "replayed=%lu malformed=%lu cpu_ms=%.3f\n",
        samples,
        malformed,
        (double)(clock() - start_clock) * 1000.0 / (double)CLOCKS_PER_SEC
    );
    fclose(input);
    fclose(output);
    return samples > 0U ? 0 : 3;
}
