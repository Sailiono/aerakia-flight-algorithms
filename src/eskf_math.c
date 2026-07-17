/**
 * @file eskf_math.c
 * @brief Lightweight Linear Algebra Library for ESKF - Implementation
 *
 * All functions operate on static arrays - NO dynamic memory allocation.
 *
 * Maintained by Aerakia contributors.
 */

#include "eskf_math.h"
#include <string.h>

/* ============================================================================
 * Vector3 Operations
 * ============================================================================ */

void eskf_vec3_add(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

void eskf_vec3_sub(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

void eskf_vec3_scale(const eskf_float_t a[3], eskf_float_t s, eskf_float_t out[3]) {
    out[0] = a[0] * s;
    out[1] = a[1] * s;
    out[2] = a[2] * s;
}

eskf_float_t eskf_vec3_norm(const eskf_float_t a[3]) {
    return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

eskf_float_t eskf_vec3_normalize(eskf_float_t a[3]) {
    eskf_float_t n = eskf_vec3_norm(a);
    if (n > ESKF_EPSILON) {
        eskf_float_t inv_n = 1.0 / n;
        a[0] *= inv_n;
        a[1] *= inv_n;
        a[2] *= inv_n;
    }
    return n;
}

eskf_float_t eskf_vec3_dot(const eskf_float_t a[3], const eskf_float_t b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void eskf_vec3_cross(const eskf_float_t a[3], const eskf_float_t b[3], eskf_float_t out[3]) {
    /* out = a × b */
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

void eskf_vec3_copy(const eskf_float_t a[3], eskf_float_t out[3]) {
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
}

void eskf_vec3_zero(eskf_float_t a[3]) {
    a[0] = 0.0;
    a[1] = 0.0;
    a[2] = 0.0;
}

/* ============================================================================
 * Quaternion Operations
 * Convention: q = [w, x, y, z] (scalar first)
 * ============================================================================ */

void eskf_quat_identity(eskf_float_t q[4]) {
    q[0] = 1.0;  /* w */
    q[1] = 0.0;  /* x */
    q[2] = 0.0;  /* y */
    q[3] = 0.0;  /* z */
}

void eskf_quat_normalize(eskf_float_t q[4]) {
    eskf_float_t n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > ESKF_EPSILON) {
        eskf_float_t inv_n = 1.0 / n;
        q[0] *= inv_n;
        q[1] *= inv_n;
        q[2] *= inv_n;
        q[3] *= inv_n;
    }
}

void eskf_quat_mult(const eskf_float_t p[4], const eskf_float_t q[4], eskf_float_t out[4]) {
    /*
     * Quaternion multiplication: out = p ⊗ q
     * Represents: first rotate by q, then rotate by p
     *
     * Formula (Hamilton convention):
     * [pw, px, py, pz] ⊗ [qw, qx, qy, qz] =
     * [pw*qw - px*qx - py*qy - pz*qz,
     *  pw*qx + px*qw + py*qz - pz*qy,
     *  pw*qy - px*qz + py*qw + pz*qx,
     *  pw*qz + px*qy - py*qx + pz*qw]
     */
    out[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    out[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    out[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    out[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
}

void eskf_quat_conjugate(const eskf_float_t q[4], eskf_float_t out[4]) {
    out[0] =  q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void eskf_quat_from_axis_angle(const eskf_float_t axis[3], eskf_float_t angle, eskf_float_t out[4]) {
    /*
     * Quaternion from axis-angle:
     * q = [cos(θ/2), sin(θ/2)*axis]
     */
    eskf_float_t half_angle = 0.5 * angle;
    eskf_float_t s = sin(half_angle);
    out[0] = cos(half_angle);
    out[1] = s * axis[0];
    out[2] = s * axis[1];
    out[3] = s * axis[2];
}

void eskf_quat_from_rotation_vector(const eskf_float_t theta[3], eskf_float_t out[4]) {
    /*
     * Convert rotation vector to quaternion.
     * theta = axis * angle (rotation vector)
     *
     * For small angles (|theta| < 1e-6), use approximation:
     *   q ≈ [1, theta/2] (then normalize)
     *
     * For larger angles, compute properly:
     *   angle = |theta|
     *   axis = theta / angle
     *   q = [cos(angle/2), sin(angle/2)*axis]
     */
    eskf_float_t theta_mag = eskf_vec3_norm(theta);

    if (theta_mag < ESKF_EPSILON) {
        /* Small angle approximation */
        out[0] = 1.0;
        out[1] = 0.5 * theta[0];
        out[2] = 0.5 * theta[1];
        out[3] = 0.5 * theta[2];
        eskf_quat_normalize(out);
    } else {
        /* Full computation */
        eskf_float_t half_angle = 0.5 * theta_mag;
        eskf_float_t s = sin(half_angle) / theta_mag;
        out[0] = cos(half_angle);
        out[1] = s * theta[0];
        out[2] = s * theta[1];
        out[3] = s * theta[2];
    }
}

void eskf_quat_to_rot_mat3(const eskf_float_t q[4], eskf_float_t R[3][3]) {
    /*
     * Convert quaternion to rotation matrix R_nb (Body to Earth)
     *
     * R = | 1-2(y²+z²)   2(xy-wz)    2(xz+wy)  |
     *     | 2(xy+wz)     1-2(x²+z²)  2(yz-wx)  |
     *     | 2(xz-wy)     2(yz+wx)    1-2(x²+y²)|
     */
    eskf_float_t w = q[0], x = q[1], y = q[2], z = q[3];

    eskf_float_t xx = x * x;
    eskf_float_t yy = y * y;
    eskf_float_t zz = z * z;
    eskf_float_t xy = x * y;
    eskf_float_t xz = x * z;
    eskf_float_t yz = y * z;
    eskf_float_t wx = w * x;
    eskf_float_t wy = w * y;
    eskf_float_t wz = w * z;

    R[0][0] = 1.0 - 2.0*(yy + zz);
    R[0][1] = 2.0*(xy - wz);
    R[0][2] = 2.0*(xz + wy);

    R[1][0] = 2.0*(xy + wz);
    R[1][1] = 1.0 - 2.0*(xx + zz);
    R[1][2] = 2.0*(yz - wx);

    R[2][0] = 2.0*(xz - wy);
    R[2][1] = 2.0*(yz + wx);
    R[2][2] = 1.0 - 2.0*(xx + yy);
}

void eskf_quat_copy(const eskf_float_t q[4], eskf_float_t out[4]) {
    out[0] = q[0];
    out[1] = q[1];
    out[2] = q[2];
    out[3] = q[3];
}

/* ============================================================================
 * 3x3 Matrix Operations
 * ============================================================================ */

void eskf_mat3_identity(eskf_float_t m[3][3]) {
    m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0;
    m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0;
    m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0;
}

void eskf_mat3_zero(eskf_float_t m[3][3]) {
    memset(m, 0, sizeof(eskf_float_t) * 9);
}

void eskf_mat3_mul_vec3(eskf_float_t M[3][3], const eskf_float_t v[3], eskf_float_t out[3]) {
    out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
    out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}

void eskf_mat3_mul_mat3(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        }
    }
}

void eskf_mat3_transpose(eskf_float_t A[3][3], eskf_float_t out[3][3]) {
    out[0][0] = A[0][0]; out[0][1] = A[1][0]; out[0][2] = A[2][0];
    out[1][0] = A[0][1]; out[1][1] = A[1][1]; out[1][2] = A[2][1];
    out[2][0] = A[0][2]; out[2][1] = A[1][2]; out[2][2] = A[2][2];
}

bool eskf_mat3_inv(eskf_float_t A[3][3], eskf_float_t out[3][3]) {
    /*
     * 3x3 matrix inversion using Cramer's rule
     *
     * inv(A) = (1/det(A)) * adj(A)
     * where adj(A) is the adjugate (transpose of cofactor matrix)
     */

    /* Compute cofactors */
    eskf_float_t c00 = A[1][1]*A[2][2] - A[1][2]*A[2][1];
    eskf_float_t c01 = A[1][2]*A[2][0] - A[1][0]*A[2][2];
    eskf_float_t c02 = A[1][0]*A[2][1] - A[1][1]*A[2][0];

    eskf_float_t c10 = A[0][2]*A[2][1] - A[0][1]*A[2][2];
    eskf_float_t c11 = A[0][0]*A[2][2] - A[0][2]*A[2][0];
    eskf_float_t c12 = A[0][1]*A[2][0] - A[0][0]*A[2][1];

    eskf_float_t c20 = A[0][1]*A[1][2] - A[0][2]*A[1][1];
    eskf_float_t c21 = A[0][2]*A[1][0] - A[0][0]*A[1][2];
    eskf_float_t c22 = A[0][0]*A[1][1] - A[0][1]*A[1][0];

    /* Compute determinant */
    eskf_float_t det = A[0][0]*c00 + A[0][1]*c01 + A[0][2]*c02;

    if (fabs(det) < ESKF_EPSILON) {
        /* Singular matrix */
        eskf_mat3_identity(out);
        return false;
    }

    eskf_float_t inv_det = 1.0 / det;

    /* Adjugate matrix (transpose of cofactor) */
    out[0][0] = c00 * inv_det;
    out[0][1] = c10 * inv_det;
    out[0][2] = c20 * inv_det;

    out[1][0] = c01 * inv_det;
    out[1][1] = c11 * inv_det;
    out[1][2] = c21 * inv_det;

    out[2][0] = c02 * inv_det;
    out[2][1] = c12 * inv_det;
    out[2][2] = c22 * inv_det;

    return true;
}

void eskf_mat3_skew(const eskf_float_t v[3], eskf_float_t out[3][3]) {
    /*
     * Skew-symmetric matrix from vector:
     * [v]× = |  0   -v2   v1 |
     *        |  v2   0   -v0 |
     *        | -v1   v0   0  |
     */
    out[0][0] =  0.0;    out[0][1] = -v[2];   out[0][2] =  v[1];
    out[1][0] =  v[2];   out[1][1] =  0.0;    out[1][2] = -v[0];
    out[2][0] = -v[1];   out[2][1] =  v[0];   out[2][2] =  0.0;
}

void eskf_mat3_scale(eskf_float_t A[3][3], eskf_float_t s, eskf_float_t out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = A[i][j] * s;
        }
    }
}

void eskf_mat3_sub(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = A[i][j] - B[i][j];
        }
    }
}

void eskf_mat3_add(eskf_float_t A[3][3], eskf_float_t B[3][3], eskf_float_t out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = A[i][j] + B[i][j];
        }
    }
}

void eskf_mat3_copy(eskf_float_t A[3][3], eskf_float_t out[3][3]) {
    memcpy(out, A, sizeof(eskf_float_t) * 9);
}

/* ============================================================================
 * 15x15 Matrix Operations (Specialized for ESKF)
 * ============================================================================ */

void eskf_mat15_zero(eskf_float_t m[15][15]) {
    memset(m, 0, sizeof(eskf_float_t) * 225);
}

void eskf_mat15_identity(eskf_float_t m[15][15]) {
    eskf_mat15_zero(m);
    for (int i = 0; i < 15; i++) {
        m[i][i] = 1.0;
    }
}

void eskf_mat15_mul_mat15(eskf_float_t A[15][15], eskf_float_t B[15][15],
                     eskf_float_t out[15][15]) {
    /*
     * Standard matrix multiplication: out = A * B
     * Note: This is O(n³) and should be avoided in hot paths.
     * Use eskf_mat15_propagate for covariance updates.
     */
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 15; k++) {
                sum += A[i][k] * B[k][j];
            }
            out[i][j] = sum;
        }
    }
}

void eskf_mat15_add(eskf_float_t A[15][15], eskf_float_t B[15][15],
               eskf_float_t out[15][15]) {
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            out[i][j] = A[i][j] + B[i][j];
        }
    }
}

void eskf_mat15_propagate(eskf_float_t P[15][15],
                     eskf_float_t F[15][15],
                     eskf_float_t Q[15][15]) {
    /*
     * Covariance propagation: P = F * P * F^T + Q
     *
     * This is a two-step process:
     *   1. temp = F * P
     *   2. P = temp * F^T + Q
     *
     * Note: For maximum efficiency on embedded systems, this should be
     * rewritten to exploit F's sparsity. The current implementation is
     * general-purpose for correctness validation.
     */
    eskf_float_t temp[15][15];

    /* Step 1: temp = F * P */
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 15; k++) {
                sum += F[i][k] * P[k][j];
            }
            temp[i][j] = sum;
        }
    }

    /* Step 2: P = temp * F^T + Q */
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = Q[i][j];
            for (int k = 0; k < 15; k++) {
                sum += temp[i][k] * F[j][k];  /* F^T[k][j] = F[j][k] */
            }
            P[i][j] = sum;
        }
    }
}

void eskf_mat15_symmetrize(eskf_float_t P[15][15]) {
    /*
     * Force symmetry: P = 0.5 * (P + P^T)
     * Important for numerical stability of covariance matrix
     */
    for (int i = 0; i < 15; i++) {
        for (int j = i + 1; j < 15; j++) {
            eskf_float_t avg = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = avg;
            P[j][i] = avg;
        }
    }
}

void eskf_mat15_copy(eskf_float_t A[15][15], eskf_float_t out[15][15]) {
    memcpy(out, A, sizeof(eskf_float_t) * 225);
}
