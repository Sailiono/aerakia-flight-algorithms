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

static void test_continuous_noise_discretization(void)
{
    ESKF_Handle filter;
    ESKF_Config config;
    const eskf_float_t acc[3] = {0.0, 0.0, -ESKF_GRAVITY};
    const eskf_float_t gyro[3] = {0.0, 0.0, 0.0};
    const double dt = 0.1;
    int row;
    int column;

    eskf_init(&filter, NULL, NULL);
    for (row = 0; row < 15; ++row) {
        for (column = 0; column < 15; ++column) filter.P[row][column] = 0.0;
    }
    config.sigma_acc = 0.3;
    config.sigma_gyr = 0.2;
    config.sigma_acc_bias = 0.01;
    config.sigma_gyr_bias = 0.02;
    eskf_set_config(&filter, &config);
    eskf_predict(&filter, acc, gyro, dt);

    check_true(near(filter.P[ESKF_IDX_DTHETA][ESKF_IDX_DTHETA], 0.2 * 0.2 * dt, 1e-14),
               "gyro noise density discretizes with sigma squared times dt");
    check_true(near(filter.P[ESKF_IDX_DV][ESKF_IDX_DV], 0.3 * 0.3 * dt, 1e-14),
               "accelerometer noise density discretizes into velocity");
    check_true(near(filter.P[ESKF_IDX_DP][ESKF_IDX_DP],
                    0.3 * 0.3 * dt * dt * dt / 3.0, 1e-14),
               "accelerometer noise density discretizes into position");
    check_true(near(filter.P[ESKF_IDX_DV][ESKF_IDX_DP],
                    0.3 * 0.3 * dt * dt * 0.5, 1e-14),
               "accelerometer process noise preserves velocity-position correlation");
    check_true(near(filter.P[ESKF_IDX_DAB][ESKF_IDX_DAB], 0.01 * 0.01 * dt, 1e-14),
               "accelerometer bias random walk discretizes with dt");
    check_true(near(filter.P[ESKF_IDX_DGB][ESKF_IDX_DGB], 0.02 * 0.02 * dt, 1e-14),
               "gyroscope bias random walk discretizes with dt");
}

static void test_covariance_update_invariants(void)
{
    ESKF_Handle filter;
    ESKF_InnovResult result;
    eskf_float_t q[4];
    const eskf_float_t position[3] = {0.02, -0.01, 0.03};
    const eskf_float_t velocity[3] = {0.01, 0.02, -0.01};
    int iteration;
    int row;
    int column;

    yaw_quaternion(0.12, q);
    eskf_init(&filter, NULL, q);
    for (iteration = 0; iteration < 50; ++iteration) {
        eskf_update_position(&filter, position, 1.0e-6, &result);
        eskf_update_velocity(&filter, velocity, 1.0e-6, &result);
        eskf_update_heading(&filter, 0.0, 1.0e-8, &result);
    }
    for (row = 0; row < 15; ++row) {
        check_true(isfinite(filter.P[row][row]), "updated covariance diagonal is finite");
        check_true(filter.P[row][row] >= -1.0e-15,
                   "updated covariance diagonal remains non-negative");
        for (column = 0; column < 15; ++column) {
            check_true(fabs(filter.P[row][column] - filter.P[column][row]) < 1.0e-12,
                       "updated covariance remains symmetric");
        }
    }
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
    before[0] = filter.state.v[0];
    before[1] = filter.state.v[1];
    before[2] = filter.state.v[2];
    eskf_update_velocity(&filter, outlier, 1.0, &result);
    check_true(!result.accepted, "velocity outlier is rejected");
    check_true(near(filter.state.v[0], before[0], 1e-12), "rejected velocity leaves state unchanged");
}

static void test_heading_updates_are_yaw_only(void)
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
}

int main(void)
{
    test_initialization();
    test_stationary_prediction();
    test_continuous_noise_discretization();
    test_covariance_update_invariants();
    test_yaw_integration();
    test_position_update_and_gate();
    test_velocity_update_and_gate();
    test_heading_updates_are_yaw_only();
    test_navigation_reset_preserves_attitude_and_biases();
    test_static_bias_alignment();

    if (failures != 0) {
        fprintf(stderr, "%d ESKF assertion(s) failed\n", failures);
        return 1;
    }

    puts("ESKF tests passed");
    return 0;
}
