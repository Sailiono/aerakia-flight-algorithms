/**
 * @file types.h
 * @brief Hardware-neutral measurement and estimate contracts.
 */

#ifndef AERAKIA_TYPES_H
#define AERAKIA_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define AERAKIA_PI_F 3.14159265358979323846f
#define AERAKIA_GRAVITY_M_S2 9.80665f

typedef struct {
    float x;
    float y;
    float z;
} AerakiaVec3f;

typedef enum {
    AERAKIA_SAMPLE_ACCEL_VALID = 1U << 0,
    AERAKIA_SAMPLE_GYRO_VALID = 1U << 1,
    AERAKIA_SAMPLE_MAG_VALID = 1U << 2,
    /** Application asserts that the vehicle is stationary for alignment/ZUPT. */
    AERAKIA_SAMPLE_STATIONARY = 1U << 3
} AerakiaSampleFlags;

/**
 * One calibrated sensor sample published by the private driver layer.
 *
 * Frames: body FRD. Units: SI except magnetic field in microtesla.
 * timestamp_us must be monotonic and identify the physical sample time.
 */
typedef struct {
    uint64_t timestamp_us;
    AerakiaVec3f acceleration_m_s2;
    AerakiaVec3f angular_rate_rad_s;
    AerakiaVec3f magnetic_field_ut;
    uint32_t flags;
} AerakiaImuSample;

typedef enum {
    AERAKIA_STATUS_OK = 0,
    AERAKIA_STATUS_INITIALIZED = 1,
    AERAKIA_STATUS_ALIGNING = 2,
    AERAKIA_STATUS_INVALID_ARGUMENT = -1,
    AERAKIA_STATUS_MISSING_MEASUREMENT = -2,
    AERAKIA_STATUS_TIMESTAMP_ERROR = -3,
    AERAKIA_STATUS_NUMERICAL_ERROR = -4
} AerakiaStatus;

typedef struct {
    float quaternion_wxyz[4];
    AerakiaVec3f euler_rad;
    AerakiaVec3f gravity_body_unit;
    AerakiaVec3f gyro_bias_rad_s;
    float accelerometer_weight;
    float magnetometer_weight;
    bool healthy;
} AerakiaAttitudeEstimate;

#endif
