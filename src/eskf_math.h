/**
 * @file eskf_math.h
 * @brief Lightweight Linear Algebra Library for ESKF
 *
 * This library provides optimized math primitives for the Error-State Kalman Filter.
 * All functions operate on static arrays - NO dynamic memory allocation.
 *
 * Coordinate Convention: NED (North-East-Down)
 * Quaternion Convention: [w, x, y, z] (scalar first)
 *
 * Maintained by Aerakia contributors.
 */

#ifndef AERAKIA_ESKF_MATH_H
#define AERAKIA_ESKF_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <aerakia/eskf_types.h>

/* ============================================================================
 * Constants
 * ============================================================================ */

/*
 * Keep the implementation's arithmetic consistent with the selected core
 * scalar type.  The public default remains double; the float candidate must
 * not silently promote its hot-path math to double on a single-precision FPU.
 * These are private implementation helpers, not public API macros.
 */
#define ESKF_SCALAR(value) ((eskf_float_t)(value))

#if defined(AERAKIA_ESKF_CORE_USE_FLOAT)
#define ESKF_ABS(value) fabsf(value)
#define ESKF_ASIN(value) asinf(value)
#define ESKF_ATAN2(y, x) atan2f((y), (x))
#define ESKF_COS(value) cosf(value)
#define ESKF_HYPOT(x, y) hypotf((x), (y))
#define ESKF_MAX(x, y) fmaxf((x), (y))
#define ESKF_MIN(x, y) fminf((x), (y))
#define ESKF_SIN(value) sinf(value)
#define ESKF_SQRT(value) sqrtf(value)
#else
#define ESKF_ABS(value) fabs(value)
#define ESKF_ASIN(value) asin(value)
#define ESKF_ATAN2(y, x) atan2((y), (x))
#define ESKF_COS(value) cos(value)
#define ESKF_HYPOT(x, y) hypot((x), (y))
#define ESKF_MAX(x, y) fmax((x), (y))
#define ESKF_MIN(x, y) fmin((x), (y))
#define ESKF_SIN(value) sin(value)
#define ESKF_SQRT(value) sqrt(value)
#endif

/* ============================================================================
 * Vector3 Operations
 * ============================================================================ */

/**
 * @brief Add two 3D vectors: out = a + b
 */
void eskf_vec3_add(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]);

/**
 * @brief Subtract two 3D vectors: out = a - b
 */
void eskf_vec3_sub(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]);

/**
 * @brief Scale a 3D vector: out = a * s
 */
void eskf_vec3_scale(const eskf_float_t a[3], eskf_float_t s, eskf_float_t out[3]);

/**
 * @brief Compute the Euclidean norm of a 3D vector
 */
eskf_float_t eskf_vec3_norm(const eskf_float_t a[3]);

/**
 * @brief Normalize a 3D vector in-place
 * @return The original norm (before normalization)
 */
eskf_float_t eskf_vec3_normalize(eskf_float_t a[3]);

/**
 * @brief Compute dot product of two 3D vectors
 */
eskf_float_t eskf_vec3_dot(const eskf_float_t a[3], const eskf_float_t b[3]);

/**
 * @brief Compute cross product: out = a × b
 */
void eskf_vec3_cross(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]);

/**
 * @brief Copy a 3D vector: out = a
 */
void eskf_vec3_copy(const eskf_float_t a[3], eskf_float_t out[3]);

/**
 * @brief Set a 3D vector to zero
 */
void eskf_vec3_zero(eskf_float_t a[3]);

/* ============================================================================
 * Quaternion Operations
 * Convention: q = [w, x, y, z] (scalar first)
 * ============================================================================ */

/**
 * @brief Set quaternion to identity [1, 0, 0, 0]
 */
void eskf_quat_identity(eskf_float_t q[4]);

/**
 * @brief Normalize a quaternion in-place
 */
void eskf_quat_normalize(eskf_float_t q[4]);

/**
 * @brief Quaternion multiplication: out = p ⊗ q
 * Represents the composition of rotations: first q, then p
 */
void eskf_quat_mult(const eskf_float_t p[4], const eskf_float_t q[4], eskf_float_t out[4]);

/**
 * @brief Compute quaternion conjugate: out = q*
 */
void eskf_quat_conjugate(const eskf_float_t q[4], eskf_float_t out[4]);

/**
 * @brief Create quaternion from axis-angle representation
 * @param axis Unit rotation axis (must be normalized)
 * @param angle Rotation angle in radians
 * @param out Output quaternion
 */
void eskf_quat_from_axis_angle(const eskf_float_t axis[3], eskf_float_t angle, eskf_float_t out[4]);

/**
 * @brief Create quaternion from small rotation vector (theta)
 * Uses small angle approximation: q ≈ [1, theta/2]
 * @param theta Rotation vector (rad)
 * @param out Output quaternion
 */
void eskf_quat_from_rotation_vector(const eskf_float_t theta[3], eskf_float_t out[4]);

/**
 * @brief Convert quaternion to 3x3 rotation matrix (Body to Earth)
 * R_nb: transforms vectors from Body frame to Earth frame
 */
void eskf_quat_to_rot_mat3(const eskf_float_t q[4], eskf_float_t R[3][3]);

/**
 * @brief Copy a quaternion: out = q
 */
void eskf_quat_copy(const eskf_float_t q[4], eskf_float_t out[4]);

/* ============================================================================
 * 3x3 Matrix Operations
 * ============================================================================ */

/*
 * Matrix inputs intentionally omit const. In C99, adding const to the element
 * type of a multidimensional array changes the adjusted pointer type and makes
 * ordinary matrices incompatible under strict conformance rules. These
 * functions nevertheless treat every documented input matrix as read-only.
 */

/**
 * @brief Set 3x3 matrix to identity
 */
void eskf_mat3_identity(eskf_float_t m[3][3]);

/**
 * @brief Set 3x3 matrix to zero
 */
void eskf_mat3_zero(eskf_float_t m[3][3]);

/**
 * @brief Multiply 3x3 matrix by 3D vector: out = M * v
 */
void eskf_mat3_mul_vec3(eskf_float_t M[3][3], const eskf_float_t v[3], eskf_float_t out[3]);

/**
 * @brief Multiply two 3x3 matrices: out = A * B
 */
void eskf_mat3_mul_mat3(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]);

/**
 * @brief Transpose a 3x3 matrix: out = A^T
 */
void eskf_mat3_transpose(eskf_float_t A[3][3], eskf_float_t out[3][3]);

/**
 * @brief Invert a 3x3 matrix using Cramer's rule
 * @return true if successful, false if matrix is singular
 */
bool eskf_mat3_inv(eskf_float_t A[3][3], eskf_float_t out[3][3]);

/**
 * @brief Compute skew-symmetric matrix from vector: out = [v]×
 */
void eskf_mat3_skew(const eskf_float_t v[3], eskf_float_t out[3][3]);

/**
 * @brief Scale a 3x3 matrix: out = A * s
 */
void eskf_mat3_scale(eskf_float_t A[3][3], eskf_float_t s, eskf_float_t out[3][3]);

/**
 * @brief Subtract two 3x3 matrices: out = A - B
 */
void eskf_mat3_sub(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]);

/**
 * @brief Add two 3x3 matrices: out = A + B
 */
void eskf_mat3_add(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]);

/**
 * @brief Copy a 3x3 matrix: out = A
 */
void eskf_mat3_copy(eskf_float_t A[3][3], eskf_float_t out[3][3]);

/* ============================================================================
 * 15x15 Matrix Operations (Specialized for ESKF)
 * Error State Order: dtheta(0-2), dv(3-5), dp(6-8), dab(9-11), dgb(12-14)
 * ============================================================================ */

/**
 * @brief Set 15x15 matrix to zero
 */
void eskf_mat15_zero(eskf_float_t m[15][15]);

/**
 * @brief Set 15x15 matrix to identity
 */
void eskf_mat15_identity(eskf_float_t m[15][15]);

/**
 * @brief Multiply two 15x15 matrices: out = A * B
 * Note: For performance-critical code, use eskf_mat15_propagate instead
 */
void eskf_mat15_mul_mat15(eskf_float_t A[15][15], eskf_float_t B[15][15],
                     eskf_float_t out[15][15]);

/**
 * @brief Add two 15x15 matrices: out = A + B
 */
void eskf_mat15_add(eskf_float_t A[15][15], eskf_float_t B[15][15],
               eskf_float_t out[15][15]);

/**
 * @brief Propagate covariance: P_out = F * P * F^T + Q
 * This is an optimized operation exploiting the sparsity of F
 * @param P Current covariance matrix (15x15)
 * @param F State transition Jacobian (15x15)
 * @param Q Process noise covariance (15x15)
 */
void eskf_mat15_propagate(eskf_float_t P[15][15],
                     eskf_float_t F[15][15],
                     eskf_float_t Q[15][15]);

/**
 * @brief Force symmetry on a 15x15 matrix: P = 0.5 * (P + P^T)
 */
void eskf_mat15_symmetrize(eskf_float_t P[15][15]);

/**
 * @brief Copy a 15x15 matrix: out = A
 */
void eskf_mat15_copy(eskf_float_t A[15][15], eskf_float_t out[15][15]);

#endif
