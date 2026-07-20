// SPDX-License-Identifier: Apache-2.0
//
// Drive the unmodified, pinned PX4 ecl_EKF from the M0 canonical event CSV.
// This runner deliberately never reads any truth columns.  Truth remains in
// the immutable CSV solely for the post-run scorer.

#include <EKF/ekf.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr float kDefaultOriginLatitudeDeg = 47.3566094f;
constexpr float kDefaultOriginLongitudeDeg = 8.5190237f;
constexpr float kDefaultOriginAltitudeM = 422.056f;

struct Options {
    std::string input_path;
    std::string output_path;
    double origin_latitude_deg{kDefaultOriginLatitudeDeg};
    double origin_longitude_deg{kDefaultOriginLongitudeDeg};
    float origin_altitude_m{kDefaultOriginAltitudeM};
};

struct Row {
    uint64_t timestamp_us{};
    float delta_angle_dt_s{};
    float delta_angle_x_rad{};
    float delta_angle_y_rad{};
    float delta_angle_z_rad{};
    float delta_velocity_dt_s{};
    float delta_velocity_x_m_s{};
    float delta_velocity_y_m_s{};
    float delta_velocity_z_m_s{};
    bool accel_clipping_x{};
    bool accel_clipping_y{};
    bool accel_clipping_z{};
    bool at_rest{};
    bool in_air{};
    bool in_transition{};
    bool gnss_position_update{};
    float gnss_position_n_m{};
    float gnss_position_e_m{};
    float gnss_position_d_m{};
    float gnss_position_variance_m2{};
    bool gnss_velocity_update{};
    float gnss_velocity_n_m_s{};
    float gnss_velocity_e_m_s{};
    float gnss_velocity_d_m_s{};
    float gnss_velocity_variance_m2_s2{};
    bool barometer_update{};
    float barometer_height_up_m{};
    float barometer_variance_m2{};
    bool magnetometer_update{};
    float magnetometer_x_ut{};
    float magnetometer_y_ut{};
    float magnetometer_z_ut{};
};

[[noreturn]] void usage(const std::string &reason)
{
    throw std::runtime_error(
        reason + "\nusage: aerakia_px4_bias_ab_runner "
        "[--origin-lat-deg VALUE --origin-lon-deg VALUE --origin-alt-m VALUE] "
        "INPUT_CANONICAL.csv OUTPUT_PX4.csv"
    );
}

double parse_finite_double(const std::string &text, const std::string &name)
{
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("non-finite or malformed " + name);
    }
    return value;
}

Options parse_options(int argc, char **argv)
{
    Options options;
    std::vector<std::string> positional;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--origin-lat-deg" || argument == "--origin-lon-deg"
            || argument == "--origin-alt-m") {
            if (++index >= argc) {
                usage("missing value for " + argument);
            }
            const double value = parse_finite_double(argv[index], argument);
            if (argument == "--origin-lat-deg") {
                options.origin_latitude_deg = value;
            } else if (argument == "--origin-lon-deg") {
                options.origin_longitude_deg = value;
            } else {
                options.origin_altitude_m = static_cast<float>(value);
            }
        } else if (!argument.empty() && argument.front() == '-') {
            usage("unknown option " + argument);
        } else {
            positional.push_back(argument);
        }
    }
    if (positional.size() != 2) {
        usage("exactly one input CSV and one output CSV are required");
    }
    if (std::abs(options.origin_latitude_deg) > 90.0
        || std::abs(options.origin_longitude_deg) > 180.0) {
        usage("origin latitude/longitude is outside WGS-84 bounds");
    }
    options.input_path = positional[0];
    options.output_path = positional[1];
    return options;
}

std::vector<std::string> split_csv(const std::string &line)
{
    // M0's canonical event stream is intentionally numeric and never permits
    // quoted CSV fields.  Rejecting quotes avoids silently accepting a parser
    // interpretation different from the Python scorer.
    if (line.find('"') != std::string::npos) {
        throw std::runtime_error("quoted CSV fields are not permitted by the M0 contract");
    }
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        if (!field.empty() && field.back() == '\r') {
            field.pop_back();
        }
        fields.push_back(field);
    }
    return fields;
}

std::unordered_map<std::string, std::size_t> index_columns(const std::string &header)
{
    std::unordered_map<std::string, std::size_t> columns;
    const std::vector<std::string> fields = split_csv(header);
    for (std::size_t index = 0; index < fields.size(); ++index) {
        if (fields[index].empty() || !columns.emplace(fields[index], index).second) {
            throw std::runtime_error("canonical CSV has an empty or duplicate header field");
        }
    }
    return columns;
}

const std::string &field_at(
    const std::vector<std::string> &fields,
    const std::unordered_map<std::string, std::size_t> &columns,
    const char *name
)
{
    const auto found = columns.find(name);
    if (found == columns.end()) {
        throw std::runtime_error(std::string("canonical CSV is missing required column ") + name);
    }
    if (found->second >= fields.size()) {
        throw std::runtime_error(std::string("canonical CSV row is missing value for ") + name);
    }
    return fields[found->second];
}

float get_float(
    const std::vector<std::string> &fields,
    const std::unordered_map<std::string, std::size_t> &columns,
    const char *name
)
{
    const double value = parse_finite_double(field_at(fields, columns, name), name);
    if (value < -std::numeric_limits<float>::max() || value > std::numeric_limits<float>::max()) {
        throw std::runtime_error(std::string("value outside float range for ") + name);
    }
    return static_cast<float>(value);
}

uint64_t get_timestamp(
    const std::vector<std::string> &fields,
    const std::unordered_map<std::string, std::size_t> &columns,
    const char *name
)
{
    const double value = parse_finite_double(field_at(fields, columns, name), name);
    if (value < 0.0 || std::floor(value) != value
        || value > static_cast<double>(std::numeric_limits<uint64_t>::max())) {
        throw std::runtime_error(std::string("invalid integer timestamp for ") + name);
    }
    return static_cast<uint64_t>(value);
}

bool get_bool(
    const std::vector<std::string> &fields,
    const std::unordered_map<std::string, std::size_t> &columns,
    const char *name
)
{
    const double value = parse_finite_double(field_at(fields, columns, name), name);
    if (value != 0.0 && value != 1.0) {
        throw std::runtime_error(std::string("boolean column must be 0 or 1: ") + name);
    }
    return value == 1.0;
}

Row parse_row(
    const std::vector<std::string> &fields,
    const std::unordered_map<std::string, std::size_t> &columns
)
{
    Row row{};
    row.timestamp_us = get_timestamp(fields, columns, "timestamp_us");
    row.delta_angle_dt_s = get_float(fields, columns, "delta_angle_dt_s");
    row.delta_angle_x_rad = get_float(fields, columns, "delta_angle_x_rad");
    row.delta_angle_y_rad = get_float(fields, columns, "delta_angle_y_rad");
    row.delta_angle_z_rad = get_float(fields, columns, "delta_angle_z_rad");
    row.delta_velocity_dt_s = get_float(fields, columns, "delta_velocity_dt_s");
    row.delta_velocity_x_m_s = get_float(fields, columns, "delta_velocity_x_m_s");
    row.delta_velocity_y_m_s = get_float(fields, columns, "delta_velocity_y_m_s");
    row.delta_velocity_z_m_s = get_float(fields, columns, "delta_velocity_z_m_s");
    row.accel_clipping_x = get_bool(fields, columns, "accel_clipping_x");
    row.accel_clipping_y = get_bool(fields, columns, "accel_clipping_y");
    row.accel_clipping_z = get_bool(fields, columns, "accel_clipping_z");
    row.at_rest = get_bool(fields, columns, "at_rest");
    row.in_air = get_bool(fields, columns, "in_air");
    row.in_transition = get_bool(fields, columns, "in_transition");
    row.gnss_position_update = get_bool(fields, columns, "gnss_position_update");
    row.gnss_position_n_m = get_float(fields, columns, "gnss_position_n_m");
    row.gnss_position_e_m = get_float(fields, columns, "gnss_position_e_m");
    row.gnss_position_d_m = get_float(fields, columns, "gnss_position_d_m");
    row.gnss_position_variance_m2 = get_float(fields, columns, "gnss_position_variance_m2");
    row.gnss_velocity_update = get_bool(fields, columns, "gnss_velocity_update");
    row.gnss_velocity_n_m_s = get_float(fields, columns, "gnss_velocity_n_m_s");
    row.gnss_velocity_e_m_s = get_float(fields, columns, "gnss_velocity_e_m_s");
    row.gnss_velocity_d_m_s = get_float(fields, columns, "gnss_velocity_d_m_s");
    row.gnss_velocity_variance_m2_s2 = get_float(fields, columns, "gnss_velocity_variance_m2_s2");
    row.barometer_update = get_bool(fields, columns, "barometer_update");
    row.barometer_height_up_m = get_float(fields, columns, "barometer_height_up_m");
    row.barometer_variance_m2 = get_float(fields, columns, "barometer_variance_m2");
    row.magnetometer_update = get_bool(fields, columns, "magnetometer_update");
    row.magnetometer_x_ut = get_float(fields, columns, "magnetometer_x_ut");
    row.magnetometer_y_ut = get_float(fields, columns, "magnetometer_y_ut");
    row.magnetometer_z_ut = get_float(fields, columns, "magnetometer_z_ut");

    if (row.delta_angle_dt_s <= 0.0f || row.delta_velocity_dt_s <= 0.0f) {
        throw std::runtime_error("IMU integration intervals must be positive");
    }
    if (row.gnss_position_variance_m2 <= 0.0f || row.gnss_velocity_variance_m2_s2 <= 0.0f
        || row.barometer_variance_m2 <= 0.0f) {
        throw std::runtime_error("declared measurement variances must be positive");
    }
    // PX4's gnssSample is a single atomic receiver report.  Allowing a row to
    // claim position without velocity (or the reverse) would make the two
    // filters consume materially different events.  A future protocol revision
    // can add explicit per-observation masks to both implementations.
    if (row.gnss_position_update != row.gnss_velocity_update) {
        throw std::runtime_error(
            "M0 requires paired GNSS position and velocity updates for PX4 parity"
        );
    }
    return row;
}

void configure_stock_profile(Ekf &ekf)
{
    parameters *params = ekf.getParamHandle();
    params->ekf2_imu_ctrl = 7;
    params->ekf2_abias_init = 0.2f;
    params->ekf2_abl_acclim = 25.0f;
    params->ekf2_abl_gyrlim = 3.0f;
    params->ekf2_abl_lim = 0.4f;
    params->ekf2_abl_tau = 0.5f;
    params->ekf2_acc_b_noise = 0.003f;
#if defined(CONFIG_EKF2_GNSS)
    params->ekf2_gps_ctrl = static_cast<int32_t>(GnssCtrl::HPOS)
                            | static_cast<int32_t>(GnssCtrl::VPOS)
                            | static_cast<int32_t>(GnssCtrl::VEL);
#endif
#if defined(CONFIG_EKF2_BAROMETER)
    params->ekf2_baro_ctrl = 1;
#endif
#if defined(CONFIG_EKF2_MAGNETOMETER)
    // Common-sensor M0 uses heading-only magnetic fusion.  It intentionally
    // does not grant PX4 its full magnetic-field-state advantage.
    params->ekf2_mag_type = MagFuseType::HEADING;
#endif
}

void feed_row(Ekf &ekf, const Row &row, const MapProjection &origin, float origin_altitude_m)
{
    imuSample imu{};
    imu.time_us = row.timestamp_us;
    imu.delta_ang = Vector3f{row.delta_angle_x_rad, row.delta_angle_y_rad, row.delta_angle_z_rad};
    imu.delta_ang_dt = row.delta_angle_dt_s;
    imu.delta_vel = Vector3f{
        row.delta_velocity_x_m_s, row.delta_velocity_y_m_s, row.delta_velocity_z_m_s
    };
    imu.delta_vel_dt = row.delta_velocity_dt_s;
    imu.delta_vel_clipping[0] = row.accel_clipping_x;
    imu.delta_vel_clipping[1] = row.accel_clipping_y;
    imu.delta_vel_clipping[2] = row.accel_clipping_z;
    ekf.setIMUData(imu);

    systemFlagUpdate system_flags{};
    system_flags.time_us = row.timestamp_us;
    system_flags.at_rest = row.at_rest;
    system_flags.in_air = row.in_air;
    system_flags.in_transition = row.in_transition;
    ekf.setSystemFlagData(system_flags);

#if defined(CONFIG_EKF2_GNSS)
    if (row.gnss_position_update) {
        gnssSample gnss{};
        gnss.time_us = row.timestamp_us;
        origin.reproject(
            row.gnss_position_n_m, row.gnss_position_e_m, gnss.lat, gnss.lon
        );
        gnss.alt = origin_altitude_m - row.gnss_position_d_m;
        gnss.vel = Vector3f{
            row.gnss_velocity_n_m_s, row.gnss_velocity_e_m_s, row.gnss_velocity_d_m_s
        };
        gnss.hacc = std::sqrt(row.gnss_position_variance_m2);
        gnss.vacc = std::sqrt(row.gnss_position_variance_m2);
        gnss.sacc = std::sqrt(row.gnss_velocity_variance_m2_s2);
        gnss.fix_type = 3;
        gnss.nsats = 16;
        gnss.pdop = 1.0f;
        gnss.yaw = NAN;
        ekf.setGpsData(gnss);
    }
#endif
#if defined(CONFIG_EKF2_BAROMETER)
    if (row.barometer_update) {
        baroSample barometer{};
        barometer.time_us = row.timestamp_us;
        barometer.hgt = origin_altitude_m + row.barometer_height_up_m;
        ekf.setBaroData(barometer);
    }
#endif
#if defined(CONFIG_EKF2_MAGNETOMETER)
    if (row.magnetometer_update) {
        magSample magnetometer{};
        magnetometer.time_us = row.timestamp_us;
        // PX4 ecl_EKF's magSample uses Gauss; M0 canonical input uses microtesla.
        magnetometer.mag = Vector3f{
            row.magnetometer_x_ut * 0.01f,
            row.magnetometer_y_ut * 0.01f,
            row.magnetometer_z_ut * 0.01f
        };
        ekf.setMagData(magnetometer);
    }
#endif
}

bool finite_vector(const Vector3f &value)
{
    return std::isfinite(value(0)) && std::isfinite(value(1)) && std::isfinite(value(2));
}

bool finite_quaternion(const Quatf &value)
{
    return std::isfinite(value(0)) && std::isfinite(value(1))
           && std::isfinite(value(2)) && std::isfinite(value(3));
}

void write_output_row(std::ofstream &output, const Ekf &ekf)
{
    const StateSample &state = ekf.state();
    const Vector3f accel_bias = ekf.getAccelBias();
    const Vector3f accel_bias_variance = ekf.getAccelBiasVariance();
    const bool finite = finite_quaternion(state.quat_nominal) && finite_vector(state.vel)
                        && finite_vector(state.pos) && finite_vector(accel_bias)
                        && finite_vector(accel_bias_variance);
    const bool healthy = finite && ekf.fault_status().value == 0U;
    const bool accel_bias_valid = healthy && ekf.attitude_valid()
                                   && accel_bias_variance(0) > 0.0f
                                   && accel_bias_variance(1) > 0.0f
                                   && accel_bias_variance(2) > 0.0f;

    output << ekf.time_delayed_us() << ','
           << accel_bias(0) << ',' << accel_bias(1) << ',' << accel_bias(2) << ','
           << accel_bias_variance(0) << ',' << accel_bias_variance(1) << ','
           << accel_bias_variance(2) << ','
           << state.quat_nominal(0) << ',' << state.quat_nominal(1) << ','
           << state.quat_nominal(2) << ',' << state.quat_nominal(3) << ','
           << state.vel(0) << ',' << state.vel(1) << ',' << state.vel(2) << ','
           << state.pos(0) << ',' << state.pos(1) << ',' << state.pos(2) << ','
           << (healthy ? 1 : 0) << ',' << (accel_bias_valid ? 1 : 0) << ','
           << 1 << ',' << (ekf.accel_bias_inhibited() ? 1 : 0) << '\n';
}

void write_header(std::ofstream &output)
{
    output << "fusion_horizon_timestamp_us,"
           << "accel_bias_x_m_s2,accel_bias_y_m_s2,accel_bias_z_m_s2,"
           << "accel_bias_variance_x_m2_s4,accel_bias_variance_y_m2_s4,"
           << "accel_bias_variance_z_m2_s4,"
           << "q_w,q_x,q_y,q_z,"
           << "velocity_n_m_s,velocity_e_m_s,velocity_d_m_s,"
           << "position_n_m,position_e_m,position_d_m,"
           << "healthy,accel_bias_valid,accel_bias_learning_inhibit_supported,"
           << "accel_bias_learning_inhibited\n";
}

} // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        std::ifstream input(options.input_path);
        if (!input.is_open()) {
            throw std::runtime_error("cannot open canonical input " + options.input_path);
        }
        std::ofstream output(options.output_path);
        if (!output.is_open()) {
            throw std::runtime_error("cannot open PX4 output " + options.output_path);
        }

        std::string header;
        if (!std::getline(input, header)) {
            throw std::runtime_error("canonical input has no header");
        }
        const auto columns = index_columns(header);
        output << std::setprecision(9);
        write_header(output);

        MapProjection origin(options.origin_latitude_deg, options.origin_longitude_deg);
        Ekf ekf;
        configure_stock_profile(ekf);

        uint64_t previous_timestamp = 0;
        bool have_previous_timestamp = false;
        uint64_t previous_horizon_timestamp = 0;
        bool have_previous_horizon_timestamp = false;
        std::size_t input_rows = 0;
        std::size_t output_rows = 0;
        std::string line;
        while (std::getline(input, line)) {
            if (line.empty()) {
                throw std::runtime_error("empty canonical CSV row is not permitted");
            }
            const Row row = parse_row(split_csv(line), columns);
            if (have_previous_timestamp && row.timestamp_us <= previous_timestamp) {
                throw std::runtime_error("canonical CSV timestamp is not strictly increasing");
            }
            previous_timestamp = row.timestamp_us;
            have_previous_timestamp = true;
            ++input_rows;
            feed_row(ekf, row, origin, options.origin_altitude_m);
            if (ekf.update()) {
                const uint64_t horizon_timestamp = ekf.time_delayed_us();
                // PX4 can update its output predictor several times while
                // filling its initial delayed IMU buffer. M0 compares only
                // unique, strictly advancing delayed-fusion horizons.
                if (have_previous_horizon_timestamp
                    && horizon_timestamp < previous_horizon_timestamp) {
                    throw std::runtime_error("PX4 delayed fusion horizon regressed");
                }
                if (!have_previous_horizon_timestamp
                    || horizon_timestamp > previous_horizon_timestamp) {
                    write_output_row(output, ekf);
                    previous_horizon_timestamp = horizon_timestamp;
                    have_previous_horizon_timestamp = true;
                    ++output_rows;
                }
            }
        }
        if (input_rows == 0 || output_rows == 0) {
            throw std::runtime_error("PX4 produced no delayed-horizon output rows");
        }
        std::cerr << "px4_m0_replay input_rows=" << input_rows
                  << " output_rows=" << output_rows << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "px4_m0_replay error: " << error.what() << '\n';
        return 2;
    }
}
