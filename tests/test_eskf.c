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
    test_yaw_integration();
    test_position_update_and_gate();
    test_static_bias_alignment();

    if (failures != 0) {
        fprintf(stderr, "%d ESKF assertion(s) failed\n", failures);
        return 1;
    }

    puts("ESKF tests passed");
    return 0;
}
