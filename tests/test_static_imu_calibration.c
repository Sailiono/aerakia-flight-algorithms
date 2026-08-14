#include <aerakia/static_imu_calibration.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_true(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failures++;
    }
}

static int near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void expect_rejected(
    const AerakiaStaticImuCalibrationResult *result,
    AerakiaStaticImuCalibrationStatus status,
    const char *message)
{
    check_true(result->status == status, message);
    check_true(!result->accepted, "rejected result must not be accepted");
}

static AerakiaStaticImuPoseMean make_pose(
    float direction_x,
    float direction_y,
    float direction_z,
    AerakiaVec3f accelerometer_bias,
    AerakiaVec3f gyroscope_bias
)
{
    AerakiaStaticImuPoseMean pose;
    pose.acceleration_m_s2.x = accelerometer_bias.x + direction_x * AERAKIA_GRAVITY_M_S2;
    pose.acceleration_m_s2.y = accelerometer_bias.y + direction_y * AERAKIA_GRAVITY_M_S2;
    pose.acceleration_m_s2.z = accelerometer_bias.z + direction_z * AERAKIA_GRAVITY_M_S2;
    pose.angular_rate_rad_s = gyroscope_bias;
    pose.sample_count = 400U;
    return pose;
}

static void make_six_axis_poses(
    AerakiaStaticImuPoseMean poses[6],
    AerakiaVec3f accelerometer_bias,
    AerakiaVec3f gyroscope_bias
)
{
    poses[0] = make_pose(1.0f, 0.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[1] = make_pose(-1.0f, 0.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[2] = make_pose(0.0f, 1.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[3] = make_pose(0.0f, -1.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[4] = make_pose(0.0f, 0.0f, 1.0f, accelerometer_bias, gyroscope_bias);
    poses[5] = make_pose(0.0f, 0.0f, -1.0f, accelerometer_bias, gyroscope_bias);
}

static void test_exact_six_pose_solution(void)
{
    AerakiaStaticImuPoseMean poses[6];
    AerakiaStaticImuCalibrationResult result;
    const AerakiaVec3f accelerometer_bias = {0.12f, -0.08f, 0.05f};
    const AerakiaVec3f gyroscope_bias = {0.006f, -0.004f, 0.002f};
    AerakiaStaticImuCalibrationStatus status;

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);

    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_OK, "six-axis solution is accepted");
    check_true(result.accepted, "six-axis result reports accepted");
    check_true(near(result.accelerometer_bias_m_s2.x, accelerometer_bias.x, 1.0e-5f),
               "six-axis solution recovers accelerometer x bias");
    check_true(near(result.accelerometer_bias_m_s2.y, accelerometer_bias.y, 1.0e-5f),
               "six-axis solution recovers accelerometer y bias");
    check_true(near(result.accelerometer_bias_m_s2.z, accelerometer_bias.z, 1.0e-5f),
               "six-axis solution recovers accelerometer z bias");
    check_true(near(result.gyroscope_bias_rad_s.x, gyroscope_bias.x, 1.0e-6f),
               "six-axis solution recovers gyro x bias");
    check_true(near(result.gyroscope_bias_rad_s.y, gyroscope_bias.y, 1.0e-6f),
               "six-axis solution recovers gyro y bias");
    check_true(near(result.gyroscope_bias_rad_s.z, gyroscope_bias.z, 1.0e-6f),
               "six-axis solution recovers gyro z bias");
    check_true(result.gravity_residual_rms_m_s2 < 1.0e-4f,
               "exact gravity means leave negligible residual");
    check_true(result.direction_coverage_eigenvalues[0] > 0.30f,
               "six-axis pose set has three-dimensional coverage");
}

static void test_noisy_eight_pose_solution(void)
{
    static const float directions[8][3] = {
        { 0.577350269f,  0.577350269f,  0.577350269f},
        { 0.577350269f,  0.577350269f, -0.577350269f},
        { 0.577350269f, -0.577350269f,  0.577350269f},
        { 0.577350269f, -0.577350269f, -0.577350269f},
        {-0.577350269f,  0.577350269f,  0.577350269f},
        {-0.577350269f,  0.577350269f, -0.577350269f},
        {-0.577350269f, -0.577350269f,  0.577350269f},
        {-0.577350269f, -0.577350269f, -0.577350269f}
    };
    static const float acceleration_noise[8][3] = {
        { 0.006f, -0.004f,  0.002f}, {-0.004f,  0.003f, -0.005f},
        { 0.003f,  0.005f, -0.002f}, {-0.005f, -0.002f,  0.004f},
        { 0.004f, -0.003f,  0.005f}, {-0.003f,  0.004f, -0.004f},
        { 0.002f,  0.003f,  0.001f}, {-0.002f, -0.004f, -0.003f}
    };
    AerakiaStaticImuPoseMean poses[8];
    AerakiaStaticImuCalibrationResult result;
    const AerakiaVec3f accelerometer_bias = {-0.11f, 0.09f, 0.04f};
    const AerakiaVec3f gyroscope_bias = {-0.005f, 0.003f, 0.002f};
    AerakiaStaticImuCalibrationStatus status;
    int index;

    for (index = 0; index < 8; ++index) {
        poses[index] = make_pose(
            directions[index][0], directions[index][1], directions[index][2],
            accelerometer_bias, gyroscope_bias
        );
        poses[index].acceleration_m_s2.x += acceleration_noise[index][0];
        poses[index].acceleration_m_s2.y += acceleration_noise[index][1];
        poses[index].acceleration_m_s2.z += acceleration_noise[index][2];
        poses[index].angular_rate_rad_s.x += 0.001f * (float)((index % 3) - 1);
        poses[index].angular_rate_rad_s.y += 0.0005f * (float)((index % 2) ? 1 : -1);
    }
    status = aerakia_static_imu_calibrate(poses, 8U, NULL, &result);

    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_OK, "noisy eight-pose solution is accepted");
    check_true(near(result.accelerometer_bias_m_s2.x, accelerometer_bias.x, 0.015f),
               "noisy solution keeps accelerometer x bias accurate");
    check_true(near(result.accelerometer_bias_m_s2.y, accelerometer_bias.y, 0.015f),
               "noisy solution keeps accelerometer y bias accurate");
    check_true(near(result.accelerometer_bias_m_s2.z, accelerometer_bias.z, 0.015f),
               "noisy solution keeps accelerometer z bias accurate");
    check_true(result.gravity_residual_rms_m_s2 < 0.02f,
               "noisy solution reports bounded gravity residual");
    check_true(result.gyroscope_residual_rms_rad_s < 0.01f,
               "noisy solution reports bounded gyro residual");
}

static void test_rejection_paths(void)
{
    AerakiaStaticImuPoseMean poses[6];
    AerakiaStaticImuCalibrationConfig config;
    AerakiaStaticImuCalibrationResult result;
    const AerakiaVec3f accelerometer_bias = {0.10f, -0.05f, 0.03f};
    const AerakiaVec3f gyroscope_bias = {0.002f, 0.001f, -0.003f};
    AerakiaStaticImuCalibrationStatus status;

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    status = aerakia_static_imu_calibrate(poses, 1U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_INSUFFICIENT_POSES,
               "one pose is rejected before attempting a sphere fit");
    expect_rejected(&result, status, "insufficient poses are not accepted");
    status = aerakia_static_imu_calibrate(
        poses, AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES + 1U, NULL, &result
    );
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_TOO_MANY_POSES,
               "unbounded preflight pose count is rejected");
    expect_rejected(&result, status, "excessive pose count is not accepted");

    aerakia_static_imu_calibration_default_config(&config);
    config.minimum_pose_count = AERAKIA_STATIC_IMU_CALIBRATION_MAX_POSES + 1U;
    status = aerakia_static_imu_calibrate(poses, 6U, &config, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_INVALID_ARGUMENT,
               "impossible configured pose count is rejected as invalid configuration");
    expect_rejected(&result, status, "invalid configuration is not accepted");

    aerakia_static_imu_calibration_default_config(&config);
    config.minimum_pose_count = 4U;
    poses[0] = make_pose(1.0f, 0.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[1] = make_pose(-1.0f, 0.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[2] = make_pose(0.0f, 1.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    poses[3] = make_pose(0.0f, -1.0f, 0.0f, accelerometer_bias, gyroscope_bias);
    status = aerakia_static_imu_calibrate(poses, 4U, &config, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY,
               "coplanar gravity directions are rejected");
    expect_rejected(&result, status, "degenerate geometry is not accepted");

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    aerakia_static_imu_calibration_default_config(&config);
    config.maximum_gravity_residual_m_s2 = 0.02f;
    config.maximum_gravity_residual_rms_m_s2 = 0.02f;
    poses[2].acceleration_m_s2.x += 4.0f;
    status = aerakia_static_imu_calibrate(poses, 6U, &config, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_GRAVITY_RESIDUAL_EXCEEDED,
               "a large accelerometer outlier rejects the complete calibration");
    expect_rejected(&result, status, "gravity residual failure is not accepted");

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    poses[5].angular_rate_rad_s.x = 0.08f;
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_NONSTATIONARY_INPUT,
               "nonstationary gyro mean is rejected");
    expect_rejected(&result, status, "nonstationary input is not accepted");

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    poses[3].acceleration_m_s2.x = NAN;
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_NONFINITE_INPUT,
               "non-finite accelerometer mean is rejected");
    expect_rejected(&result, status, "non-finite input is not accepted");

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    poses[0].angular_rate_rad_s.x += 0.040f;
    poses[1].angular_rate_rad_s.x -= 0.040f;
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_GYROSCOPE_RESIDUAL_EXCEEDED,
               "inconsistent stationary gyro means are rejected");
    expect_rejected(&result, status, "gyroscope residual failure is not accepted");
}

static void test_near_threshold_geometry_and_noise(void)
{
    AerakiaStaticImuPoseMean poses[6];
    AerakiaStaticImuCalibrationConfig config;
    AerakiaStaticImuCalibrationResult result;
    const AerakiaVec3f accelerometer_bias = {0.07f, -0.11f, 0.03f};
    const AerakiaVec3f gyroscope_bias = {0.003f, -0.002f, 0.001f};
    AerakiaStaticImuCalibrationStatus status;
    int index;

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    /* Deterministic small mean noise represents finite static averaging. */
    for (index = 0; index < 6; ++index) {
        poses[index].acceleration_m_s2.x += 0.008f * (float)((index % 3) - 1);
        poses[index].acceleration_m_s2.y += 0.006f * (float)((index % 2) ? 1 : -1);
        poses[index].acceleration_m_s2.z += 0.004f * (float)((index % 4) - 2);
    }
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_OK,
               "small static-mean noise remains inside accepted calibration envelope");
    check_true(near(result.accelerometer_bias_m_s2.x, accelerometer_bias.x, 0.02f),
               "small static-mean noise retains usable x bias estimate");

    aerakia_static_imu_calibration_default_config(&config);
    config.minimum_pose_count = 4U;
    poses[0] = make_pose(1.0f, 0.0f, 0.00f, accelerometer_bias, gyroscope_bias);
    poses[1] = make_pose(-1.0f, 0.0f, 0.00f, accelerometer_bias, gyroscope_bias);
    poses[2] = make_pose(0.0f, 1.0f, 0.01f, accelerometer_bias, gyroscope_bias);
    poses[3] = make_pose(0.0f, -1.0f, -0.01f, accelerometer_bias, gyroscope_bias);
    status = aerakia_static_imu_calibrate(poses, 4U, &config, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_DEGENERATE_GEOMETRY,
               "near-coplanar directions fail the geometry gate");
}

static void test_single_pose_influence_rejection(void)
{
    AerakiaStaticImuPoseMean poses[6];
    AerakiaStaticImuCalibrationResult result;
    const AerakiaVec3f accelerometer_bias = {0.10f, -0.05f, 0.03f};
    const AerakiaVec3f gyroscope_bias = {0.002f, 0.001f, -0.003f};
    AerakiaStaticImuCalibrationStatus status;

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    /*
     * This is not large enough to rely only on the radial-residual gate.
     * Leaving the contaminated +Y pose out, however, materially moves the
     * gravity-sphere centre and must reject the entire six-pose record.
     */
    poses[2].acceleration_m_s2.x += 1.0f;
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_OUTLIER_SENSITIVITY_EXCEEDED,
               "single-pose influence rejects a moderately corrupted pose mean");
    check_true(!result.accepted, "influence-rejected calibration is not accepted");
    check_true(result.maximum_leave_one_out_bias_delta_m_s2 > 0.02f,
               "influence diagnostic reports the material seed change");

    make_six_axis_poses(poses, accelerometer_bias, gyroscope_bias);
    poses[2].acceleration_m_s2.x += 0.10f;
    status = aerakia_static_imu_calibrate(poses, 6U, NULL, &result);
    check_true(status == AERAKIA_STATIC_IMU_CALIBRATION_OK,
               "near-threshold pose perturbation remains an explicit accepted boundary");
    check_true(result.maximum_leave_one_out_bias_delta_m_s2 < 0.02f,
               "accepted boundary has bounded leave-one-out influence");
}

int main(void)
{
    test_exact_six_pose_solution();
    test_noisy_eight_pose_solution();
    test_rejection_paths();
    test_near_threshold_geometry_and_noise();
    test_single_pose_influence_rejection();
    if (failures != 0) {
        fprintf(stderr, "%d static IMU calibration test(s) failed\n", failures);
        return 1;
    }
    puts("static IMU calibration tests passed");
    return 0;
}
