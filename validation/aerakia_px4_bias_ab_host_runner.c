/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Replay the M0 canonical event CSV through Aerakia at the exact physical
 * fusion-horizon timestamps exported by the official PX4 runner. Truth
 * columns are deliberately not mapped or read by this program.
 */

#include <aerakia/eskf_adapter.h>

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE_LENGTH 16384
#define MAX_COLUMNS 128

typedef struct {
    int timestamp_us;
    int delta_angle_dt_s, delta_angle_x_rad, delta_angle_y_rad, delta_angle_z_rad;
    int delta_velocity_dt_s, delta_velocity_x_m_s, delta_velocity_y_m_s, delta_velocity_z_m_s;
    int accel_clipping_x, accel_clipping_y, accel_clipping_z;
    int at_rest, in_air, in_transition;
    int gnss_position_update, gnss_position_n_m, gnss_position_e_m, gnss_position_d_m;
    int gnss_position_variance_m2;
    int gnss_velocity_update, gnss_velocity_n_m_s, gnss_velocity_e_m_s, gnss_velocity_d_m_s;
    int gnss_velocity_variance_m2_s2;
    int barometer_update, barometer_height_up_m, barometer_variance_m2;
} ColumnMap;

typedef struct {
    uint64_t timestamp_us;
    float delta_angle_dt_s, delta_angle_x_rad, delta_angle_y_rad, delta_angle_z_rad;
    float delta_velocity_dt_s, delta_velocity_x_m_s, delta_velocity_y_m_s, delta_velocity_z_m_s;
    int accel_clipping_x, accel_clipping_y, accel_clipping_z;
    int at_rest, in_air, in_transition;
    int gnss_update;
    AerakiaVec3f gnss_position, gnss_velocity;
    float gnss_position_variance_m2, gnss_velocity_variance_m2_s2;
    int barometer_update;
    float barometer_height_up_m, barometer_variance_m2;
} ReplayRow;

static FILE *open_portable_file(const char *path, const char *mode)
{
#if defined(_MSC_VER)
    FILE *stream = NULL;
    return fopen_s(&stream, path, mode) == 0 ? stream : NULL;
#else
    return fopen(path, mode);
#endif
}

static int split_csv(char *line, char *columns[], int maximum_columns)
{
    int count = 0;
    char *cursor = line;
    while (count < maximum_columns) {
        char *comma;
        if (strchr(cursor, '"') != NULL) return -1;
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

#define MAP(member, name) map->member = find_column(columns, count, name)

static int load_column_map(char *header, ColumnMap *map)
{
    char *columns[MAX_COLUMNS];
    const int count = split_csv(header, columns, MAX_COLUMNS);
    if (count < 0) return 0;
    memset(map, -1, sizeof(*map));
    MAP(timestamp_us, "timestamp_us");
    MAP(delta_angle_dt_s, "delta_angle_dt_s");
    MAP(delta_angle_x_rad, "delta_angle_x_rad");
    MAP(delta_angle_y_rad, "delta_angle_y_rad");
    MAP(delta_angle_z_rad, "delta_angle_z_rad");
    MAP(delta_velocity_dt_s, "delta_velocity_dt_s");
    MAP(delta_velocity_x_m_s, "delta_velocity_x_m_s");
    MAP(delta_velocity_y_m_s, "delta_velocity_y_m_s");
    MAP(delta_velocity_z_m_s, "delta_velocity_z_m_s");
    MAP(accel_clipping_x, "accel_clipping_x");
    MAP(accel_clipping_y, "accel_clipping_y");
    MAP(accel_clipping_z, "accel_clipping_z");
    MAP(at_rest, "at_rest"); MAP(in_air, "in_air"); MAP(in_transition, "in_transition");
    MAP(gnss_position_update, "gnss_position_update");
    MAP(gnss_position_n_m, "gnss_position_n_m");
    MAP(gnss_position_e_m, "gnss_position_e_m");
    MAP(gnss_position_d_m, "gnss_position_d_m");
    MAP(gnss_position_variance_m2, "gnss_position_variance_m2");
    MAP(gnss_velocity_update, "gnss_velocity_update");
    MAP(gnss_velocity_n_m_s, "gnss_velocity_n_m_s");
    MAP(gnss_velocity_e_m_s, "gnss_velocity_e_m_s");
    MAP(gnss_velocity_d_m_s, "gnss_velocity_d_m_s");
    MAP(gnss_velocity_variance_m2_s2, "gnss_velocity_variance_m2_s2");
    MAP(barometer_update, "barometer_update");
    MAP(barometer_height_up_m, "barometer_height_up_m");
    MAP(barometer_variance_m2, "barometer_variance_m2");
    return map->timestamp_us >= 0
        && map->delta_angle_dt_s >= 0 && map->delta_angle_x_rad >= 0
        && map->delta_angle_y_rad >= 0 && map->delta_angle_z_rad >= 0
        && map->delta_velocity_dt_s >= 0 && map->delta_velocity_x_m_s >= 0
        && map->delta_velocity_y_m_s >= 0 && map->delta_velocity_z_m_s >= 0
        && map->accel_clipping_x >= 0 && map->accel_clipping_y >= 0 && map->accel_clipping_z >= 0
        && map->at_rest >= 0 && map->in_air >= 0 && map->in_transition >= 0
        && map->gnss_position_update >= 0 && map->gnss_position_n_m >= 0
        && map->gnss_position_e_m >= 0 && map->gnss_position_d_m >= 0
        && map->gnss_position_variance_m2 >= 0 && map->gnss_velocity_update >= 0
        && map->gnss_velocity_n_m_s >= 0 && map->gnss_velocity_e_m_s >= 0
        && map->gnss_velocity_d_m_s >= 0 && map->gnss_velocity_variance_m2_s2 >= 0
        && map->barometer_update >= 0 && map->barometer_height_up_m >= 0
        && map->barometer_variance_m2 >= 0;
}

static double parse_double(char *columns[], int count, int index, int *ok)
{
    char *end;
    double value;
    if (index < 0 || index >= count) {
        *ok = 0;
        return 0.0;
    }
    errno = 0;
    value = strtod(columns[index], &end);
    if (errno != 0 || end == columns[index] || *end != '\0' || !isfinite(value)) *ok = 0;
    return value;
}

static int parse_bool(char *columns[], int count, int index, int *ok)
{
    const double value = parse_double(columns, count, index, ok);
    if (value != 0.0 && value != 1.0) *ok = 0;
    return value == 1.0;
}

static uint64_t parse_timestamp(char *columns[], int count, int index, int *ok)
{
    const double value = parse_double(columns, count, index, ok);
    if (value < 0.0 || floor(value) != value || value > 18446744073709551615.0) {
        *ok = 0;
        return 0U;
    }
    return (uint64_t)value;
}

static ReplayRow parse_row(char *columns[], int count, const ColumnMap *map, int *ok)
{
    ReplayRow row;
    memset(&row, 0, sizeof(row));
    row.timestamp_us = parse_timestamp(columns, count, map->timestamp_us, ok);
    row.delta_angle_dt_s = (float)parse_double(columns, count, map->delta_angle_dt_s, ok);
    row.delta_angle_x_rad = (float)parse_double(columns, count, map->delta_angle_x_rad, ok);
    row.delta_angle_y_rad = (float)parse_double(columns, count, map->delta_angle_y_rad, ok);
    row.delta_angle_z_rad = (float)parse_double(columns, count, map->delta_angle_z_rad, ok);
    row.delta_velocity_dt_s = (float)parse_double(columns, count, map->delta_velocity_dt_s, ok);
    row.delta_velocity_x_m_s = (float)parse_double(columns, count, map->delta_velocity_x_m_s, ok);
    row.delta_velocity_y_m_s = (float)parse_double(columns, count, map->delta_velocity_y_m_s, ok);
    row.delta_velocity_z_m_s = (float)parse_double(columns, count, map->delta_velocity_z_m_s, ok);
    row.accel_clipping_x = parse_bool(columns, count, map->accel_clipping_x, ok);
    row.accel_clipping_y = parse_bool(columns, count, map->accel_clipping_y, ok);
    row.accel_clipping_z = parse_bool(columns, count, map->accel_clipping_z, ok);
    row.at_rest = parse_bool(columns, count, map->at_rest, ok);
    row.in_air = parse_bool(columns, count, map->in_air, ok);
    row.in_transition = parse_bool(columns, count, map->in_transition, ok);
    row.gnss_update = parse_bool(columns, count, map->gnss_position_update, ok);
    if (row.gnss_update != parse_bool(columns, count, map->gnss_velocity_update, ok)) *ok = 0;
    row.gnss_position.x = (float)parse_double(columns, count, map->gnss_position_n_m, ok);
    row.gnss_position.y = (float)parse_double(columns, count, map->gnss_position_e_m, ok);
    row.gnss_position.z = (float)parse_double(columns, count, map->gnss_position_d_m, ok);
    row.gnss_position_variance_m2 = (float)parse_double(
        columns, count, map->gnss_position_variance_m2, ok
    );
    row.gnss_velocity.x = (float)parse_double(columns, count, map->gnss_velocity_n_m_s, ok);
    row.gnss_velocity.y = (float)parse_double(columns, count, map->gnss_velocity_e_m_s, ok);
    row.gnss_velocity.z = (float)parse_double(columns, count, map->gnss_velocity_d_m_s, ok);
    row.gnss_velocity_variance_m2_s2 = (float)parse_double(
        columns, count, map->gnss_velocity_variance_m2_s2, ok
    );
    row.barometer_update = parse_bool(columns, count, map->barometer_update, ok);
    row.barometer_height_up_m = (float)parse_double(columns, count, map->barometer_height_up_m, ok);
    row.barometer_variance_m2 = (float)parse_double(columns, count, map->barometer_variance_m2, ok);
    if (row.delta_angle_dt_s <= 0.0f || row.delta_velocity_dt_s <= 0.0f
        || row.gnss_position_variance_m2 <= 0.0f || row.gnss_velocity_variance_m2_s2 <= 0.0f
        || row.barometer_variance_m2 <= 0.0f) *ok = 0;
    return row;
}

static int append_schedule(uint64_t **schedule, size_t *count, size_t *capacity, uint64_t timestamp)
{
    uint64_t *grown;
    if (*count == *capacity) {
        const size_t new_capacity = *capacity == 0U ? 1024U : *capacity * 2U;
        grown = (uint64_t *)realloc(*schedule, new_capacity * sizeof(**schedule));
        if (grown == NULL) return 0;
        *schedule = grown;
        *capacity = new_capacity;
    }
    if (*count > 0U && timestamp <= (*schedule)[*count - 1U]) return 0;
    (*schedule)[(*count)++] = timestamp;
    return 1;
}

static int load_schedule(const char *path, uint64_t **schedule, size_t *count)
{
    FILE *stream = open_portable_file(path, "r");
    char line[MAX_LINE_LENGTH];
    int time_index;
    size_t capacity = 0U;
    if (stream == NULL || fgets(line, sizeof(line), stream) == NULL) {
        if (stream != NULL) fclose(stream);
        return 0;
    }
    {
        char *columns[MAX_COLUMNS];
        const int columns_count = split_csv(line, columns, MAX_COLUMNS);
        time_index = columns_count < 0 ? -1 : find_column(columns, columns_count, "fusion_horizon_timestamp_us");
    }
    if (time_index < 0) {
        fclose(stream);
        return 0;
    }
    while (fgets(line, sizeof(line), stream) != NULL) {
        char *columns[MAX_COLUMNS];
        int ok = 1;
        const int columns_count = split_csv(line, columns, MAX_COLUMNS);
        const uint64_t timestamp = columns_count < 0
            ? 0U : parse_timestamp(columns, columns_count, time_index, &ok);
        if (!ok || !append_schedule(schedule, count, &capacity, timestamp)) {
            fclose(stream);
            return 0;
        }
    }
    fclose(stream);
    return *count > 0U;
}

static void write_header(FILE *output)
{
    fputs(
        "fusion_horizon_timestamp_us,"
        "accel_bias_x_m_s2,accel_bias_y_m_s2,accel_bias_z_m_s2,"
        "accel_bias_variance_x_m2_s4,accel_bias_variance_y_m2_s4,"
        "accel_bias_variance_z_m2_s4,"
        "q_w,q_x,q_y,q_z,"
        "velocity_n_m_s,velocity_e_m_s,velocity_d_m_s,"
        "position_n_m,position_e_m,position_d_m,"
        "healthy,accel_bias_valid,accel_bias_learning_inhibit_supported,"
        "accel_bias_learning_inhibited\n",
        output
    );
}

static void write_estimate(FILE *output, uint64_t timestamp, const AerakiaNavigationEstimate *estimate)
{
    const int valid = estimate->healthy
        && estimate->covariance_diagonal[ESKF_IDX_DAB] > 0.0
        && estimate->covariance_diagonal[ESKF_IDX_DAB + 1] > 0.0
        && estimate->covariance_diagonal[ESKF_IDX_DAB + 2] > 0.0;
    fprintf(
        output,
        "%llu,%.9g,%.9g,%.9g,%.17g,%.17g,%.17g,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,0,0\n",
        (unsigned long long)timestamp,
        estimate->accelerometer_bias_m_s2.x, estimate->accelerometer_bias_m_s2.y,
        estimate->accelerometer_bias_m_s2.z,
        estimate->covariance_diagonal[ESKF_IDX_DAB],
        estimate->covariance_diagonal[ESKF_IDX_DAB + 1],
        estimate->covariance_diagonal[ESKF_IDX_DAB + 2],
        estimate->attitude.quaternion_wxyz[0], estimate->attitude.quaternion_wxyz[1],
        estimate->attitude.quaternion_wxyz[2], estimate->attitude.quaternion_wxyz[3],
        estimate->velocity_ned_m_s.x, estimate->velocity_ned_m_s.y, estimate->velocity_ned_m_s.z,
        estimate->position_ned_m.x, estimate->position_ned_m.y, estimate->position_ned_m.z,
        estimate->healthy ? 1 : 0, valid
    );
}

int main(int argc, char *argv[])
{
    FILE *input = NULL;
    FILE *output = NULL;
    char line[MAX_LINE_LENGTH];
    ColumnMap map;
    uint64_t *schedule = NULL;
    size_t schedule_count = 0U;
    size_t schedule_index = 0U;
    uint64_t previous_timestamp = 0U;
    int have_previous_timestamp = 0;
    AerakiaEskf filter;
    AerakiaEskfConfig config;

    if (argc != 4) {
        fprintf(stderr, "Usage: %s INPUT_CANONICAL.csv PX4_OUTPUT.csv OUTPUT_AERAKIA.csv\n", argv[0]);
        return 2;
    }
    if (!load_schedule(argv[2], &schedule, &schedule_count)) {
        fputs("PX4 output lacks a strictly increasing fusion-horizon schedule\n", stderr);
        return 2;
    }
    input = open_portable_file(argv[1], "r");
    output = open_portable_file(argv[3], "w");
    if (input == NULL || output == NULL || fgets(line, sizeof(line), input) == NULL
        || !load_column_map(line, &map)) {
        fputs("Cannot open or parse the canonical M0 input\n", stderr);
        free(schedule);
        if (input != NULL) fclose(input);
        if (output != NULL) fclose(output);
        return 2;
    }
    aerakia_eskf_default_config(&config);
    config.fuse_magnetometer = false;
    config.process_noise.sigma_acc = 0.1;
    config.process_noise.sigma_gyr = 0.01;
    config.process_noise.sigma_acc_bias = 0.001;
    config.process_noise.sigma_gyr_bias = 0.0001;
    aerakia_eskf_init(&filter, &config, NULL, NULL);
    write_header(output);

    while (fgets(line, sizeof(line), input) != NULL) {
        char *columns[MAX_COLUMNS];
        int ok = 1;
        const int count = split_csv(line, columns, MAX_COLUMNS);
        ReplayRow row;
        AerakiaImuSample sample;
        AerakiaNavigationEstimate estimate;
        if (count < 0) ok = 0;
        row = parse_row(columns, count, &map, &ok);
        if (!ok || (have_previous_timestamp && row.timestamp_us <= previous_timestamp)) {
            fputs("Malformed or non-monotonic canonical M0 input\n", stderr);
            free(schedule); fclose(input); fclose(output); return 2;
        }
        previous_timestamp = row.timestamp_us;
        have_previous_timestamp = 1;
        if (schedule_index < schedule_count && row.timestamp_us > schedule[schedule_index]) {
            fputs("PX4 fusion-horizon timestamp is absent from canonical event stream\n", stderr);
            free(schedule); fclose(input); fclose(output); return 2;
        }
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_us = row.timestamp_us;
        sample.acceleration_m_s2.x = row.delta_velocity_x_m_s / row.delta_velocity_dt_s;
        sample.acceleration_m_s2.y = row.delta_velocity_y_m_s / row.delta_velocity_dt_s;
        sample.acceleration_m_s2.z = row.delta_velocity_z_m_s / row.delta_velocity_dt_s;
        sample.angular_rate_rad_s.x = row.delta_angle_x_rad / row.delta_angle_dt_s;
        sample.angular_rate_rad_s.y = row.delta_angle_y_rad / row.delta_angle_dt_s;
        sample.angular_rate_rad_s.z = row.delta_angle_z_rad / row.delta_angle_dt_s;
        sample.flags = AERAKIA_SAMPLE_ACCEL_VALID | AERAKIA_SAMPLE_GYRO_VALID;
        if (row.at_rest) sample.flags |= AERAKIA_SAMPLE_STATIONARY;
        (void)aerakia_eskf_process_imu(&filter, &sample, &estimate);
        if (row.gnss_update) {
            AerakiaGpsObservation observation;
            memset(&observation, 0, sizeof(observation));
            observation.timestamp_us = row.timestamp_us;
            observation.position_ned_m = row.gnss_position;
            observation.velocity_ned_m_s = row.gnss_velocity;
            observation.position_variance_m2 = row.gnss_position_variance_m2;
            observation.velocity_variance_m2_s2 = row.gnss_velocity_variance_m2_s2;
            observation.source_id = 1U;
            observation.source_generation = 1U;
            observation.quality_sequence = row.timestamp_us;
            (void)aerakia_eskf_update_gps_observation(&filter, &observation);
        }
        if (row.barometer_update) {
            AerakiaBarometerObservation observation;
            observation.timestamp_us = row.timestamp_us;
            observation.height_up_m = row.barometer_height_up_m;
            observation.variance_m2 = row.barometer_variance_m2;
            (void)aerakia_eskf_update_barometer_observation(&filter, &observation);
        }
        aerakia_eskf_get_estimate(&filter, &estimate);
        if (schedule_index < schedule_count && row.timestamp_us == schedule[schedule_index]) {
            write_estimate(output, row.timestamp_us, &estimate);
            ++schedule_index;
        }
    }
    free(schedule);
    fclose(input);
    fclose(output);
    if (schedule_index != schedule_count) {
        fputs("Canonical stream ended before every PX4 fusion-horizon timestamp\n", stderr);
        return 2;
    }
    return 0;
}
