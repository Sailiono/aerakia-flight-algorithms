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

static int near(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static double yaw_from_quaternion(const eskf_float_t q[4])
{
    return atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
}

static void yaw_quaternion(double yaw, eskf_float_t q[4])
{
    q[0] = cos(0.5 * yaw);
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = sin(0.5 * yaw);
}

static void euler_quaternion(double roll, double pitch, double yaw, eskf_float_t q[4])
{
    const double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    const double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    const double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static double roll_from_quaternion(const eskf_float_t q[4])
{
    return atan2(
        2.0 * (q[0] * q[1] + q[2] * q[3]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    );
}

static double pitch_from_quaternion(const eskf_float_t q[4])
{
    double value = 2.0 * (q[0] * q[2] - q[3] * q[1]);
    if (value > 1.0) value = 1.0;
    if (value < -1.0) value = -1.0;
    return asin(value);
}

static void ned_to_body(const eskf_float_t q[4], const eskf_float_t ned[3],
                        eskf_float_t body[3])
{
    const eskf_float_t w = q[0], x = q[1], y = q[2], z = q[3];
    body[0] = (1.0 - 2.0 * (y * y + z * z)) * ned[0]
        + 2.0 * (x * y + w * z) * ned[1]
        + 2.0 * (x * z - w * y) * ned[2];
    body[1] = 2.0 * (x * y - w * z) * ned[0]
        + (1.0 - 2.0 * (x * x + z * z)) * ned[1]
        + 2.0 * (y * z + w * x) * ned[2];
    body[2] = 2.0 * (x * z + w * y) * ned[0]
        + 2.0 * (y * z - w * x) * ned[1]
        + (1.0 - 2.0 * (x * x + y * y)) * ned[2];
}

static int covariance_is_symmetric_psd(eskf_float_t covariance[15][15])
{
    double lower[15][15] = {{0.0}};
    int i, j, k;
    for (i = 0; i < 15; ++i) {
        for (j = 0; j <= i; ++j) {
            double sum = covariance[i][j];
            if (!isfinite(sum) || fabs(covariance[i][j] - covariance[j][i]) > 1.0e-9) return 0;
            for (k = 0; k < j; ++k) sum -= lower[i][k] * lower[j][k];
            if (i == j) {
                if (sum < -1.0e-9) return 0;
                lower[i][j] = sqrt(sum > 0.0 ? sum : 0.0);
            } else if (lower[j][j] > 1.0e-12) {
                lower[i][j] = sum / lower[j][j];
            } else if (fabs(sum) > 1.0e-8) {
                return 0;
            }
        }
    }
    return 1;
}

static void test_initialization(void)
{
    ESKF_Handle filter;

    eskf_init(&filter, NULL, NULL);

    check_true(filter.initialized, "filter initializes");
    check_true(near(filter.state.q[0], 1.0, 1e-12), "identity quaternion scalar");
    check_true(near(filter.state.q[1], 0.0, 1e-12), "identity quaternion x");
    check_true(near(filter.gravity[2], ESKF_GRAVITY, 1e-12), "NED gravity");
}

static void test_stationary_prediction(void)
{
    ESKF_Handle filter;
    const eskf_float_t acc[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t gyro[3] = {0.0, 0.0, 0.0};
    eskf_float_t covariance[15];
    double quaternion_norm;
    int i;

    eskf_init(&filter, NULL, NULL);
    for (i = 0; i < 400; ++i) {
        eskf_predict(&filter, acc, gyro, 0.0025);
    }

    check_true(near(filter.state.p[2], 0.0, 1e-9), "stationary position remains fixed");
    check_true(near(filter.state.v[2], 0.0, 1e-9), "stationary velocity remains zero");

    quaternion_norm = sqrt(
        filter.state.q[0] * filter.state.q[0]
        + filter.state.q[1] * filter.state.q[1]
        + filter.state.q[2] * filter.state.q[2]
        + filter.state.q[3] * filter.state.q[3]
    );
    check_true(near(quaternion_norm, 1.0, 1e-12), "quaternion remains normalized");

    eskf_get_covariance_diag(&filter, covariance);
    for (i = 0; i < 15; ++i) {
        check_true(isfinite(covariance[i]), "covariance is finite");
        check_true(covariance[i] >= 0.0, "covariance diagonal is non-negative");
    }
}

static void test_continuous_process_noise_discretization(void)
{
    ESKF_Handle filter;
    ESKF_Config config;
    const eskf_float_t acc[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t gyro[3] = {0.0, 0.0, 0.0};
    const double dt = 0.01;
    int row;
    int column;

    eskf_init(&filter, NULL, NULL);
    config.sigma_acc = 0.2;
    config.sigma_gyr = 0.02;
    config.sigma_acc_bias = 0.0;
    config.sigma_gyr_bias = 0.0;
    eskf_set_config(&filter, &config);
    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) {
            filter.P[row][column] = 0.0;
        }
    }

    eskf_predict(&filter, acc, gyro, dt);
    check_true(near(filter.P[0][0], 0.02 * 0.02 * dt, 1.0e-14),
               "gyro noise density discretizes with dt");
    check_true(near(filter.P[3][3], 0.2 * 0.2 * dt, 1.0e-14),
               "accelerometer noise density discretizes with dt");
    check_true(near(filter.P[3][6], 0.2 * 0.2 * dt * dt / 2.0, 1.0e-14),
               "integrated acceleration creates velocity-position covariance");
    check_true(near(filter.P[6][6], 0.2 * 0.2 * dt * dt * dt / 3.0, 1.0e-14),
               "integrated acceleration creates position covariance");
}

static void test_yaw_integration(void)
{
    ESKF_Handle filter;
    const eskf_float_t acc[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t gyro[3] = {0.0, 0.0, ESKF_PI / 2.0};
    int i;

    eskf_init(&filter, NULL, NULL);
    for (i = 0; i < 100; ++i) {
        eskf_predict(&filter, acc, gyro, 0.01);
    }

    check_true(near(filter.state.q[0], sqrt(0.5), 1e-6), "90-degree yaw scalar");
    check_true(near(filter.state.q[3], sqrt(0.5), 1e-6), "90-degree yaw z component");
}

static void test_position_update_and_gate(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    const eskf_float_t nearby[3] = {1.0, -2.0, 0.5};
    const eskf_float_t outlier[3] = {1000.0, 1000.0, 1000.0};
    eskf_float_t before[3];

    eskf_init(&filter, NULL, NULL);
    eskf_update_position(&filter, nearby, 1.0, &result);
    check_true(result.accepted, "nearby position update passes innovation gate");
    check_true(result.nis >= 0.0f && isfinite(result.nis), "position update reports finite NIS");
    check_true(filter.state.p[0] > 0.9, "position update corrects north state");
    check_true(filter.state.p[1] < -1.8, "position update corrects east state");

    before[0] = filter.state.p[0];
    before[1] = filter.state.p[1];
    before[2] = filter.state.p[2];
    eskf_update_position(&filter, outlier, 1.0, &result);
    check_true(!result.accepted, "position outlier is rejected");
    check_true(near(filter.state.p[0], before[0], 1e-12), "rejected update leaves north unchanged");
    check_true(near(filter.state.p[1], before[1], 1e-12), "rejected update leaves east unchanged");
    check_true(near(filter.state.p[2], before[2], 1e-12), "rejected update leaves down unchanged");
}

static void test_velocity_update_and_gate(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    const eskf_float_t nearby[3] = {0.5, -0.25, 0.1};
    const eskf_float_t outlier[3] = {100.0, 100.0, 100.0};
    eskf_float_t before[3];

    eskf_init(&filter, NULL, NULL);
    eskf_update_velocity(&filter, nearby, 1.0, &result);
    check_true(result.accepted, "nearby velocity update passes innovation gate");
    check_true(result.nis >= 0.0f && isfinite(result.nis), "velocity update reports finite NIS");
    before[0] = filter.state.v[0];
    before[1] = filter.state.v[1];
    before[2] = filter.state.v[2];
    eskf_update_velocity(&filter, outlier, 1.0, &result);
    check_true(!result.accepted, "velocity outlier is rejected");
    check_true(near(filter.state.v[0], before[0], 1e-12), "rejected velocity leaves state unchanged");
}

static void test_heading_measurement_models(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    eskf_float_t q[4];
    const eskf_float_t north_body[3] = {1.0, 0.0, 0.0};
    double before;
    double after;

    yaw_quaternion(20.0 * ESKF_PI / 180.0, q);
    eskf_init(&filter, NULL, q);
    before = yaw_from_quaternion(filter.state.q);
    eskf_update_mag(&filter, north_body, 0.01, &result);
    after = yaw_from_quaternion(filter.state.q);
    check_true(result.accepted, "heading-only magnetometer update is accepted");
    check_true(fabs(after) < fabs(before), "magnetometer update reduces yaw error");
    check_true(near(filter.state.q[1], 0.0, 1e-12), "magnetometer does not inject roll");
    check_true(near(filter.state.q[2], 0.0, 1e-12), "magnetometer does not inject pitch");

    yaw_quaternion(-15.0 * ESKF_PI / 180.0, q);
    eskf_init(&filter, NULL, q);
    before = yaw_from_quaternion(filter.state.q);
    eskf_update_heading(&filter, 0.0, 0.01, &result);
    after = yaw_from_quaternion(filter.state.q);
    check_true(result.accepted, "trusted heading update is accepted");
    check_true(fabs(after) < fabs(before), "trusted heading update reduces yaw error");

    euler_quaternion(25.0 * ESKF_PI / 180.0, -18.0 * ESKF_PI / 180.0,
                     30.0 * ESKF_PI / 180.0, q);
    eskf_init(&filter, NULL, q);
    {
        const double observed_heading = 20.0 * ESKF_PI / 180.0;
        const double error_before = fabs(atan2(
            sin(observed_heading - yaw_from_quaternion(filter.state.q)),
            cos(observed_heading - yaw_from_quaternion(filter.state.q))
        ));
        eskf_update_heading(&filter, 20.0 * ESKF_PI / 180.0, 0.01, &result);
        check_true(result.accepted, "tilted trusted heading update is accepted");
        check_true(
            fabs(atan2(
                sin(observed_heading - yaw_from_quaternion(filter.state.q)),
                cos(observed_heading - yaw_from_quaternion(filter.state.q))
            )) < error_before,
            "full tilted-heading Jacobian reduces the physical heading residual"
        );
        check_true(isfinite(result.nis), "tilted trusted heading reports finite NIS");
    }

    euler_quaternion(0.0, 90.0 * ESKF_PI / 180.0, 0.0, q);
    eskf_init(&filter, NULL, q);
    result.accepted = true;
    eskf_update_heading(&filter, 0.5, 0.01, &result);
    check_true(!result.accepted, "vertical body-forward heading is rejected as unobservable");
    check_true(near(filter.state.q[0], q[0], 1.0e-12), "rejected vertical heading leaves q_w");
    check_true(near(filter.state.q[1], q[1], 1.0e-12), "rejected vertical heading leaves q_x");
    check_true(near(filter.state.q[2], q[2], 1.0e-12), "rejected vertical heading leaves q_y");
    check_true(near(filter.state.q[3], q[3], 1.0e-12), "rejected vertical heading leaves q_z");
}

static void test_joseph_covariance_stays_psd(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    const eskf_float_t acc[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t gyro[3] = {0.001, -0.002, 0.003};
    int index;

    eskf_init(&filter, NULL, NULL);
    for (index = 0; index < 1000; ++index) {
        eskf_float_t position[3];
        eskf_float_t velocity[3];
        eskf_predict(&filter, acc, gyro, 0.005);
        position[0] = filter.state.p[0] + 0.01;
        position[1] = filter.state.p[1] - 0.01;
        position[2] = filter.state.p[2] + 0.005;
        velocity[0] = filter.state.v[0] + 0.005;
        velocity[1] = filter.state.v[1] - 0.005;
        velocity[2] = filter.state.v[2] + 0.002;
        if ((index % 5) == 0) eskf_update_velocity(&filter, velocity, 0.04, &result);
        if ((index % 20) == 0) eskf_update_position(&filter, position, 0.25, &result);
        if ((index % 10) == 0) {
            eskf_update_heading(&filter, yaw_from_quaternion(filter.state.q) + 0.001, 0.01, &result);
        }
        if ((index % 25) == 0) eskf_update_baro(&filter, -filter.state.p[2], 0.25, &result);
    }
    check_true(
        covariance_is_symmetric_psd(filter.P),
        "Joseph updates keep covariance finite, symmetric, and positive semidefinite"
    );
}

static void test_navigation_reset_preserves_attitude_and_biases(void)
{
    ESKF_Handle filter;
    eskf_float_t q[4];
    eskf_float_t q_before[4];
    const eskf_float_t position[3] = {100.0, -20.0, 5.0};
    const eskf_float_t velocity[3] = {12.0, 3.0, -1.0};
    int index;

    yaw_quaternion(0.4, q);
    eskf_init(&filter, NULL, q);
    filter.state.ab[0] = 0.12;
    filter.state.gb[2] = -0.03;
    for (index = 0; index < 4; ++index) q_before[index] = filter.state.q[index];
    eskf_reset_navigation(&filter, position, velocity, 25.0, 4.0);
    check_true(near(filter.state.p[0], 100.0, 1e-12), "navigation reset sets position");
    check_true(near(filter.state.v[0], 12.0, 1e-12), "navigation reset sets velocity");
    check_true(near(filter.state.ab[0], 0.12, 1e-12), "navigation reset preserves accel bias");
    check_true(near(filter.state.gb[2], -0.03, 1e-12), "navigation reset preserves gyro bias");
    for (index = 0; index < 4; ++index) {
        check_true(near(filter.state.q[index], q_before[index], 1e-12), "navigation reset preserves attitude");
    }
    check_true(near(filter.P[ESKF_IDX_DP][ESKF_IDX_DP], 25.0, 1e-12), "position variance resets");
    check_true(near(filter.P[ESKF_IDX_DV][ESKF_IDX_DV], 4.0, 1e-12), "velocity variance resets");
}

static void test_static_bias_alignment(void)
{
    ESKF_Handle filter;
    const eskf_float_t acc[3][3] = {
        {0.10, -0.20, -ESKF_GRAVITY + 0.30},
        {0.10, -0.20, -ESKF_GRAVITY + 0.30},
        {0.10, -0.20, -ESKF_GRAVITY + 0.30}
    };
    const eskf_float_t gyro[3][3] = {
        {0.01, 0.02, -0.03},
        {0.01, 0.02, -0.03},
        {0.01, 0.02, -0.03}
    };

    eskf_init(&filter, NULL, NULL);
    eskf_align_static_biases(&filter, acc, gyro, 3);

    check_true(near(filter.state.ab[0], 0.10, 1e-12), "accelerometer x bias aligns");
    check_true(near(filter.state.ab[2], 0.30, 1e-12), "accelerometer z bias aligns");
    check_true(near(filter.state.gb[2], -0.03, 1e-12), "gyroscope bias aligns");
    check_true(near(filter.P[ESKF_IDX_DAB][ESKF_IDX_DAB], 4e-2, 1e-12),
               "single-pose accel bias uncertainty remains observable later");
    check_true(near(filter.P[ESKF_IDX_DGB][ESKF_IDX_DGB], 1e-4, 1e-12),
               "stationary gyro bias alignment is confident");
}

static void test_static_attitude_alignment(void)
{
    ESKF_Handle filter;
    eskf_float_t truth_q[4];
    eskf_float_t initial_q[4];
    eskf_float_t acceleration_body[3];
    eskf_float_t magnetic_body[3];
    const eskf_float_t acceleration_ned[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t magnetic_ned[3] = {22.0, 0.0, 44.0};
    const double truth_roll = 25.0 * ESKF_PI / 180.0;
    const double truth_pitch = -18.0 * ESKF_PI / 180.0;
    const double truth_yaw = 35.0 * ESKF_PI / 180.0;

    euler_quaternion(truth_roll, truth_pitch, truth_yaw, truth_q);
    ned_to_body(truth_q, acceleration_ned, acceleration_body);
    ned_to_body(truth_q, magnetic_ned, magnetic_body);
    eskf_init(&filter, NULL, NULL);
    eskf_set_mag_reference(&filter, magnetic_ned);
    check_true(eskf_align_static_tilt(&filter, acceleration_body),
               "static acceleration aligns tilt");
    check_true(near(roll_from_quaternion(filter.state.q), truth_roll, 1.0e-10),
               "static tilt recovers roll");
    check_true(near(pitch_from_quaternion(filter.state.q), truth_pitch, 1.0e-10),
               "static tilt recovers pitch");
    check_true(near(yaw_from_quaternion(filter.state.q), 0.0, 1.0e-10),
               "static tilt preserves initial yaw");
    check_true(eskf_align_static_heading(&filter, magnetic_body),
               "static magnetic field aligns heading");
    check_true(near(roll_from_quaternion(filter.state.q), truth_roll, 1.0e-10),
               "static heading preserves roll");
    check_true(near(pitch_from_quaternion(filter.state.q), truth_pitch, 1.0e-10),
               "static heading preserves pitch");
    check_true(near(yaw_from_quaternion(filter.state.q), truth_yaw, 1.0e-10),
               "static heading recovers yaw");

    yaw_quaternion(-0.7, initial_q);
    eskf_init(&filter, NULL, initial_q);
    check_true(eskf_align_static_tilt(&filter, acceleration_body),
               "tilt alignment works with an existing yaw");
    check_true(near(yaw_from_quaternion(filter.state.q), -0.7, 1.0e-10),
               "tilt alignment leaves an existing yaw unchanged");
    {
        const eskf_float_t zero[3] = {0.0, 0.0, 0.0};
        check_true(!eskf_align_static_tilt(&filter, zero),
                   "zero acceleration cannot align tilt");
        check_true(!eskf_align_static_heading(&filter, zero),
                   "zero magnetic field cannot align heading");
    }
}

static void test_attitude_covariance_reset(void)
{
    ESKF_Handle filter;
    const eskf_float_t variance[3] = {0.001, 0.002, 0.03};
    int axis;
    int index;

    eskf_init(&filter, NULL, NULL);
    filter.P[0][6] = filter.P[6][0] = 0.2;
    filter.P[1][9] = filter.P[9][1] = -0.1;
    check_true(eskf_reset_attitude_covariance(&filter, variance),
               "attitude covariance reset accepts positive variances");
    for (axis = 0; axis < 3; ++axis) {
        check_true(near(filter.P[axis][axis], variance[axis], 1.0e-14),
                   "attitude covariance reset applies requested diagonal");
        for (index = 3; index < 15; ++index) {
            check_true(near(filter.P[axis][index], 0.0, 1.0e-14)
                       && near(filter.P[index][axis], 0.0, 1.0e-14),
                       "attitude covariance reset clears stale cross covariance");
        }
    }
}

int main(void)
{
    test_initialization();
    test_stationary_prediction();
    test_continuous_process_noise_discretization();
    test_yaw_integration();
    test_position_update_and_gate();
    test_velocity_update_and_gate();
    test_heading_measurement_models();
    test_joseph_covariance_stays_psd();
    test_navigation_reset_preserves_attitude_and_biases();
    test_static_bias_alignment();
    test_static_attitude_alignment();
    test_attitude_covariance_reset();

    if (failures != 0) {
        fprintf(stderr, "%d ESKF assertion(s) failed\n", failures);
        return 1;
    }

    puts("ESKF tests passed");
    return 0;
}
