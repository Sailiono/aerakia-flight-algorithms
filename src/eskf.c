/**
 * @file eskf.c
 * @brief Error-State Kalman Filter - Core Implementation
 *
 * Based on Joan Solà "Quaternion kinematics for the error-state Kalman filter"
 *
 * This file implements the ESKF algorithm with strict adherence to:
 *   - C99 standard
 *   - No dynamic memory allocation
 *   - Double precision floating point
 *   - Explicit mathematical documentation
 *
 * Maintained by Aerakia contributors.
 */

#include <aerakia/eskf.h>
#include "eskf_math.h"
#include "eskf_models.h"
#include <string.h>

/* ============================================================================
 * Innovation Gating Thresholds (Integrity Monitoring)
 * ============================================================================ */

/** 99.7300204% chi-square limit for a scalar observation (equivalent to 3 sigma). */
#define ESKF_NIS_LIMIT_1D_3SIGMA 9.0

/** 99.7300204% chi-square limit for a three-dimensional observation. */
#define ESKF_NIS_LIMIT_3D_3SIGMA 14.1564136091267

/* ============================================================================
 * Private Helper Functions
 * ============================================================================ */

/**
 * @brief Inject error state into nominal state
 *
 * This is the "ESKF soul" - the operation that transfers error estimates
 * back to the nominal state after a measurement update.
 *
 * Operations:
 *   - Position:   p += dp
 *   - Velocity:   v += dv
 *   - Quaternion: q = q ⊗ δq, where δq ≈ [1, dθ/2]
 *   - Biases:     ab += dab, gb += dgb
 *
 * @param h   Pointer to filter handle
 * @param dx  Error state vector [15]
 */
static void _inject_error(ESKF_Handle *h, const eskf_float_t dx[15]) {
    if (!h) return;

    /* Position: p += dp */
    h->state.p[0] += dx[ESKF_IDX_DP + 0];
    h->state.p[1] += dx[ESKF_IDX_DP + 1];
    h->state.p[2] += dx[ESKF_IDX_DP + 2];

    /* Velocity: v += dv */
    h->state.v[0] += dx[ESKF_IDX_DV + 0];
    h->state.v[1] += dx[ESKF_IDX_DV + 1];
    h->state.v[2] += dx[ESKF_IDX_DV + 2];

    /* Accel Bias: ab += dab */
    h->state.ab[0] += dx[ESKF_IDX_DAB + 0];
    h->state.ab[1] += dx[ESKF_IDX_DAB + 1];
    h->state.ab[2] += dx[ESKF_IDX_DAB + 2];

    /* Gyro Bias: gb += dgb */
    h->state.gb[0] += dx[ESKF_IDX_DGB + 0];
    h->state.gb[1] += dx[ESKF_IDX_DGB + 1];
    h->state.gb[2] += dx[ESKF_IDX_DGB + 2];

    /* Quaternion: q = q ⊗ δq
     * δq is constructed from the rotation error dtheta:
     * δq = [1, dθ/2] ≈ exp(dθ/2) for small angles
     */
    eskf_float_t dtheta[3] = {
        dx[ESKF_IDX_DTHETA + 0],
        dx[ESKF_IDX_DTHETA + 1],
        dx[ESKF_IDX_DTHETA + 2]
    };

    eskf_float_t dq[4];
    eskf_quat_from_rotation_vector(dtheta, dq);

    eskf_float_t q_new[4];
    eskf_quat_mult(h->state.q, dq, q_new);
    eskf_quat_normalize(q_new);
    eskf_quat_copy(q_new, h->state.q);

    /* Note: Error state is implicitly reset to zero after injection.
     * In ESKF, we don't maintain a separate dx vector - it's computed
     * fresh each update cycle from K*z. */
}

/** Apply the covariance reset Jacobian after attitude-error injection. */
static void _reset_error_covariance(ESKF_Handle *h, const eskf_float_t dx[15]) {
    eskf_float_t G[15][15];
    eskf_float_t Q_zero[15][15];
    eskf_float_t dtheta[3];
    eskf_float_t skew[3][3];
    int i;
    int j;

    if (!h || !dx) return;
    dtheta[0] = dx[ESKF_IDX_DTHETA + 0];
    dtheta[1] = dx[ESKF_IDX_DTHETA + 1];
    dtheta[2] = dx[ESKF_IDX_DTHETA + 2];
    eskf_mat3_skew(dtheta, skew);
    eskf_mat15_identity(G);
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            G[i][j] -= 0.5 * skew[i][j];
        }
    }
    eskf_mat15_zero(Q_zero);
    eskf_mat15_propagate(h->P, G, Q_zero);
    eskf_mat15_symmetrize(h->P);
}

static eskf_float_t _wrap_pi(eskf_float_t angle) {
    return atan2(sin(angle), cos(angle));
}

static void _quaternion_from_euler(eskf_float_t roll,
                                   eskf_float_t pitch,
                                   eskf_float_t yaw,
                                   eskf_float_t q[4]) {
    const eskf_float_t cr = cos(0.5 * roll);
    const eskf_float_t sr = sin(0.5 * roll);
    const eskf_float_t cp = cos(0.5 * pitch);
    const eskf_float_t sp = sin(0.5 * pitch);
    const eskf_float_t cy = cos(0.5 * yaw);
    const eskf_float_t sy = sin(0.5 * yaw);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    eskf_quat_normalize(q);
}

/**
 * @brief Generic Kalman measurement update for 3D observations with Gating
 *
 * Implements standard Kalman update equations with innovation test:
 *   S = H * P * H^T + R
 *   NIS = z^T * S^{-1} * z  (Normalized Innovation Squared)
 *   If NIS > nis_limit: REJECT update
 *   Else: K = P * H^T * S^{-1}, dx = K * z,
 *         P = (I-KH)P(I-KH)^T + KRK^T (Joseph form)
 *
 * @param h       Pointer to filter handle
 * @param z       Residual vector (3x1)
 * @param H       Jacobian matrix (3x15)
 * @param R       Measurement noise covariance (3x3)
 * @param nis_limit Innovation NIS/chi-square limit, 0 = disable gating
 * @param result  Output: Innovation test result (can be NULL)
 * @return        true if update was accepted and applied
 */
static bool _measurement_update_3d(ESKF_Handle *h,
                                    const eskf_float_t z[3],
                                    eskf_float_t H[3][15],
                                    eskf_float_t R[3][3],
                                    eskf_float_t nis_limit,
                                    ESKF_InnovResult *result) {
    if (!h) return false;

    /* --- 1. Compute PHt = P * H^T (15x3) --- */
    eskf_float_t PHt[15][3];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 3; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 15; k++) {
                sum += h->P[i][k] * H[j][k];  /* H^T[k][j] = H[j][k] */
            }
            PHt[i][j] = sum;
        }
    }

    /* --- 2. Compute S = H * PHt + R (3x3) --- */
    eskf_float_t S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            eskf_float_t sum = R[i][j];
            for (int k = 0; k < 15; k++) {
                sum += H[i][k] * PHt[k][j];
            }
            S[i][j] = sum;
        }
    }

    /* --- 3. Invert S --- */
    eskf_float_t S_inv[3][3];
    if (!eskf_mat3_inv(S, S_inv)) {
        return false;  /* Singular matrix */
    }

    /* --- 4. Innovation Gating (NIS Test) --- */
    if (nis_limit > 0.0 || result != NULL) {
        /* Compute NIS = z^T * S_inv * z (Mahalanobis distance squared) */
        eskf_float_t S_inv_z[3];
        for (int i = 0; i < 3; i++) {
            S_inv_z[i] = S_inv[i][0] * z[0] + S_inv[i][1] * z[1] + S_inv[i][2] * z[2];
        }
        eskf_float_t nis = z[0] * S_inv_z[0] + z[1] * S_inv_z[1] + z[2] * S_inv_z[2];

        /* Fill result structure if provided */
        if (result != NULL) {
            result->innovation[0] = z[0];
            result->innovation[1] = z[1];
            result->innovation[2] = z[2];
            result->innov_var[0] = S[0][0];
            result->innov_var[1] = S[1][1];
            result->innov_var[2] = S[2][2];
            result->nis = (float)nis;
            result->test_ratio = (nis_limit > 0.0) ? (nis / nis_limit) : 0.0;
            result->accepted = (nis_limit <= 0.0) || (nis <= nis_limit);
        }

        /* Reject update if innovation exceeds gate */
        if (nis_limit > 0.0 && nis > nis_limit) {
            return false;  /* Measurement rejected by gate */
        }
    }

    /* --- 5. Compute Kalman Gain K = PHt * S_inv (15x3) --- */
    eskf_float_t K[15][3];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 3; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += PHt[i][k] * S_inv[k][j];
            }
            K[i][j] = sum;
        }
    }

    /* --- 6. Compute error state dx = K * z (15x1) --- */
    eskf_float_t dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i][0] * z[0] + K[i][1] * z[1] + K[i][2] * z[2];
    }

    /* --- 7. Joseph covariance update --- */
    /* Compute KH (15x15) */
    eskf_float_t KH[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += K[i][k] * H[k][j];
            }
            KH[i][j] = sum;
        }
    }

    /* I_KH = I - KH */
    eskf_float_t I_KH[15][15];
    eskf_mat15_identity(I_KH);
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            I_KH[i][j] -= KH[i][j];
        }
    }

    /* AP = (I - KH) * P */
    eskf_float_t AP[15][15];
    eskf_mat15_mul_mat15(I_KH, h->P, AP);

    /* P_new = AP * (I - KH)^T + K * R * K^T */
    eskf_float_t P_new[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = 0.0;
            for (int k = 0; k < 15; k++) {
                sum += AP[i][k] * I_KH[j][k];
            }
            for (int a = 0; a < 3; a++) {
                for (int b = 0; b < 3; b++) {
                    sum += K[i][a] * R[a][b] * K[j][b];
                }
            }
            P_new[i][j] = sum;
        }
    }

    /* Force symmetry and copy back */
    eskf_mat15_symmetrize(P_new);
    eskf_mat15_copy(P_new, h->P);

    /* --- 8. Inject error into nominal state --- */
    _inject_error(h, dx);
    _reset_error_covariance(h, dx);

    return true;
}

/**
 * @brief Efficient Kalman measurement update for 1D (scalar) observations with Gating
 *
 * Optimized for scalar measurements like barometer.
 * NIS = z^2 / S, test against a one-dimensional chi-square limit.
 *
 * @param h       Pointer to filter handle
 * @param z       Residual (scalar)
 * @param H       Jacobian row vector (1x15)
 * @param R       Measurement noise variance (scalar)
 * @param nis_limit Innovation NIS/chi-square limit, 0 = disable gating
 * @param result  Output: Innovation test result (can be NULL)
 * @return        true if update was accepted and applied
 */
static bool _measurement_update_1d(ESKF_Handle *h,
                                    eskf_float_t z,
                                    const eskf_float_t H[15],
                                    eskf_float_t R,
                                    eskf_float_t nis_limit,
                                    ESKF_InnovResult *result) {
    if (!h) return false;

    /* --- 1. Compute PHt = P * H^T (15x1) --- */
    eskf_float_t PHt[15];
    for (int i = 0; i < 15; i++) {
        eskf_float_t sum = 0.0;
        for (int k = 0; k < 15; k++) {
            sum += h->P[i][k] * H[k];
        }
        PHt[i] = sum;
    }

    /* --- 2. Compute S = H * PHt + R (scalar) --- */
    eskf_float_t S = R;
    for (int k = 0; k < 15; k++) {
        S += H[k] * PHt[k];
    }

    /* --- 3. Check for singularity --- */
    if (fabs(S) < ESKF_EPSILON) {
        return false;
    }

    /* --- 4. Innovation Gating (NIS Test for scalar) --- */
    if (nis_limit > 0.0 || result != NULL) {
        /* Compute NIS = z^2 / S (1-DOF chi-squared) */
        eskf_float_t nis = (z * z) / S;

        /* Fill result structure if provided */
        if (result != NULL) {
            result->innovation[0] = z;
            result->innovation[1] = 0.0;
            result->innovation[2] = 0.0;
            result->innov_var[0] = S;
            result->innov_var[1] = 0.0;
            result->innov_var[2] = 0.0;
            result->nis = (float)nis;
            result->test_ratio = (nis_limit > 0.0) ? (nis / nis_limit) : 0.0;
            result->accepted = (nis_limit <= 0.0) || (nis <= nis_limit);
        }

        /* Reject update if innovation exceeds gate */
        if (nis_limit > 0.0 && nis > nis_limit) {
            return false;  /* Measurement rejected by gate */
        }
    }

    /* --- 5. Compute Kalman Gain K = PHt / S (15x1) --- */
    eskf_float_t S_inv = 1.0 / S;
    eskf_float_t K[15];
    for (int i = 0; i < 15; i++) {
        K[i] = PHt[i] * S_inv;
    }

    /* --- 6. Compute error state dx = K * z (15x1) --- */
    eskf_float_t dx[15];
    for (int i = 0; i < 15; i++) {
        dx[i] = K[i] * z;
    }

    /* --- 7. Joseph covariance update --- */
    /* Compute KH (15x15): KH[i][j] = K[i] * H[j] */
    eskf_float_t KH[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            KH[i][j] = K[i] * H[j];
        }
    }

    /* I_KH = I - KH */
    eskf_float_t I_KH[15][15];
    eskf_mat15_identity(I_KH);
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            I_KH[i][j] -= KH[i][j];
        }
    }

    /* AP = (I - KH) * P */
    eskf_float_t AP[15][15];
    eskf_mat15_mul_mat15(I_KH, h->P, AP);

    /* P_new = AP * (I - KH)^T + K * R * K^T */
    eskf_float_t P_new[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            eskf_float_t sum = K[i] * R * K[j];
            for (int k = 0; k < 15; k++) {
                sum += AP[i][k] * I_KH[j][k];
            }
            P_new[i][j] = sum;
        }
    }

    /* Force symmetry and copy back */
    eskf_mat15_symmetrize(P_new);
    eskf_mat15_copy(P_new, h->P);

    /* --- 8. Inject error into nominal state --- */
    _inject_error(h, dx);
    _reset_error_covariance(h, dx);

    return true;
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

void eskf_init(ESKF_Handle *h,
               const eskf_float_t init_pos[3],
               const eskf_float_t init_q[4]) {
    if (!h) return;

    /* Clear entire structure */
    memset(h, 0, sizeof(ESKF_Handle));

    /* Initialize nominal state */
    if (init_pos) {
        eskf_vec3_copy(init_pos, h->state.p);
    } else {
        eskf_vec3_zero(h->state.p);
    }

    eskf_vec3_zero(h->state.v);

    if (init_q) {
        eskf_quat_copy(init_q, h->state.q);
        eskf_quat_normalize(h->state.q);
    } else {
        eskf_quat_identity(h->state.q);
    }

    eskf_vec3_zero(h->state.ab);
    eskf_vec3_zero(h->state.gb);

    /* Initialize error state to zero */
    memset(&h->error_state, 0, sizeof(ESKF_ErrorState));

    /* Initialize covariance with reasonable defaults */
    eskf_mat15_zero(h->P);
    /* Attitude uncertainty: ~10 degrees ≈ 0.17 rad, variance ≈ 0.03 */
    h->P[0][0] = 0.03; h->P[1][1] = 0.03; h->P[2][2] = 0.03;
    /* Velocity uncertainty: ~1 m/s */
    h->P[3][3] = 1.0; h->P[4][4] = 1.0; h->P[5][5] = 1.0;
    /* Position uncertainty: ~10 m */
    h->P[6][6] = 100.0; h->P[7][7] = 100.0; h->P[8][8] = 100.0;
    /* Accel bias uncertainty: ~0.1 m/s² */
    h->P[9][9] = 0.01; h->P[10][10] = 0.01; h->P[11][11] = 0.01;
    /* Gyro bias uncertainty: ~0.01 rad/s */
    h->P[12][12] = 0.0001; h->P[13][13] = 0.0001; h->P[14][14] = 0.0001;

    /* Default configuration (typical MEMS IMU values) */
    h->cfg.sigma_acc = 0.1;        /* m/s²/√Hz */
    h->cfg.sigma_gyr = 0.01;       /* rad/s/√Hz */
    h->cfg.sigma_acc_bias = 0.001; /* m/s³/√Hz */
    h->cfg.sigma_gyr_bias = 0.0001;/* rad/s²/√Hz */

    /* Gravity in NED frame: [0, 0, +g] (down is positive) */
    h->gravity[0] = 0.0;
    h->gravity[1] = 0.0;
    h->gravity[2] = ESKF_GRAVITY;

    /* Default magnetic reference: North */
    h->mag_ref[0] = 1.0;
    h->mag_ref[1] = 0.0;
    h->mag_ref[2] = 0.0;

    h->initialized = true;
}

void eskf_set_config(ESKF_Handle *h, const ESKF_Config *cfg) {
    if (!h || !cfg) return;
    h->cfg = *cfg;
}

void eskf_set_mag_reference(ESKF_Handle *h, const eskf_float_t mag_ref[3]) {
    if (!h || !mag_ref) return;
    eskf_vec3_copy(mag_ref, h->mag_ref);
    eskf_vec3_normalize(h->mag_ref);
}

/* ============================================================================
 * Prediction Step
 * ============================================================================ */

void eskf_predict(ESKF_Handle *h,
                  const eskf_float_t acc_m[3],
                  const eskf_float_t gyr_m[3],
                  eskf_float_t dt) {
    if (!h || !h->initialized || !acc_m || !gyr_m || dt <= 0.0) return;

    /* ========================================
     * Step 1: De-bias IMU inputs
     * ======================================== */
    eskf_float_t acc_correct[3], gyr_correct[3];
    eskf_vec3_sub(acc_m, h->state.ab, acc_correct);  /* acc_correct = acc_m - ab */
    eskf_vec3_sub(gyr_m, h->state.gb, gyr_correct);  /* gyr_correct = gyr_m - gb */

    /* ========================================
     * Step 2: Get current rotation matrix R_nb (Body to Earth)
     * ======================================== */
    eskf_float_t R[3][3];
    eskf_quat_to_rot_mat3(h->state.q, R);

    /* ========================================
     * Step 3: Rotate acceleration to Earth frame
     * a_earth = R * acc_correct
     * ======================================== */
    eskf_float_t acc_earth[3];
    eskf_mat3_mul_vec3(R, acc_correct, acc_earth);

    /* ========================================
     * Step 4: Add gravity (NED: g = [0, 0, +9.81])
     * a_total = a_earth + g
     * ======================================== */
    eskf_float_t acc_total[3];
    eskf_vec3_add(acc_earth, h->gravity, acc_total);

    /* Linearize at the pre-integration state used by the nominal update. */
    eskf_float_t F[15][15];
    eskf_model_transition(h->state.q, acc_correct, gyr_correct, dt, F);

    /* ========================================
     * Step 5: Integrate Nominal State
     * ======================================== */

    /* Position: p += v*dt + 0.5*a*dt² */
    eskf_float_t dt2_half = 0.5 * dt * dt;
    h->state.p[0] += h->state.v[0] * dt + acc_total[0] * dt2_half;
    h->state.p[1] += h->state.v[1] * dt + acc_total[1] * dt2_half;
    h->state.p[2] += h->state.v[2] * dt + acc_total[2] * dt2_half;

    /* Velocity: v += a*dt */
    h->state.v[0] += acc_total[0] * dt;
    h->state.v[1] += acc_total[1] * dt;
    h->state.v[2] += acc_total[2] * dt;

    /* Quaternion: q = q ⊗ δq
     * where δq is from the gyro rotation: dtheta = gyr_correct * dt
     */
    eskf_float_t d_theta[3];
    eskf_vec3_scale(gyr_correct, dt, d_theta);

    eskf_float_t dq[4];
    eskf_quat_from_rotation_vector(d_theta, dq);

    eskf_float_t q_new[4];
    eskf_quat_mult(h->state.q, dq, q_new);
    eskf_quat_normalize(q_new);
    eskf_quat_copy(q_new, h->state.q);

    /* ========================================
     * Step 7: Build Process Noise Q (15x15)
     *
     * First-order discretization of continuous white-noise densities:
     *   Q_θθ = σ_gyr² * dt
     *   Q_vv = σ_acc² * dt
     *   Q_vp = Q_pv = σ_acc² * dt² / 2
     *   Q_pp = σ_acc² * dt³ / 3
     *   Q_ab = σ_ab² * dt     (random walk on accel bias)
     *   Q_gb = σ_gb² * dt     (random walk on gyro bias)
     *
     * sigma_acc and sigma_gyr are noise densities, not per-sample standard
     * deviations. Using dt² for them would make covariance depend incorrectly
     * on sample rate and become overconfident at high IMU rates.
     * ======================================== */
    eskf_float_t Q[15][15];
    eskf_model_process_noise(&h->cfg, dt, Q);

    /* ========================================
     * Step 8: Propagate Covariance
     * P = F * P * F^T + Q
     * ======================================== */
    eskf_mat15_propagate(h->P, F, Q);

    /* Ensure numerical stability */
    eskf_mat15_symmetrize(h->P);
}

/* ============================================================================
 * Measurement Updates
 * ============================================================================ */

void eskf_update_position(ESKF_Handle *h,
                          const eskf_float_t pos_m[3],
                          eskf_float_t R_pos,
                          ESKF_InnovResult *result) {
    if (!h || !h->initialized || !pos_m || R_pos <= 0.0) return;

    /* Residual: z = measurement - prediction */
    eskf_float_t z[3];
    eskf_vec3_sub(pos_m, h->state.p, z);

    /* Jacobian H (3x15): H selects position error at indices 6-8 */
    eskf_float_t H[3][15];
    memset(H, 0, sizeof(H));
    H[0][6] = 1.0;
    H[1][7] = 1.0;
    H[2][8] = 1.0;

    /* Measurement noise R (3x3 diagonal) */
    eskf_float_t R[3][3];
    eskf_mat3_zero(R);
    R[0][0] = R_pos;
    R[1][1] = R_pos;
    R[2][2] = R_pos;

    _measurement_update_3d(h, z, H, R, ESKF_NIS_LIMIT_3D_3SIGMA, result);
}

void eskf_update_velocity(ESKF_Handle *h,
                          const eskf_float_t velocity_m_s[3],
                          eskf_float_t R_velocity,
                          ESKF_InnovResult *result) {
    eskf_float_t z[3];
    eskf_float_t H[3][15];
    eskf_float_t R[3][3];
    if (!h || !h->initialized || !velocity_m_s || R_velocity <= 0.0) return;

    eskf_vec3_sub(velocity_m_s, h->state.v, z);
    memset(H, 0, sizeof(H));
    H[0][ESKF_IDX_DV + 0] = 1.0;
    H[1][ESKF_IDX_DV + 1] = 1.0;
    H[2][ESKF_IDX_DV + 2] = 1.0;
    eskf_mat3_zero(R);
    R[0][0] = R_velocity;
    R[1][1] = R_velocity;
    R[2][2] = R_velocity;
    _measurement_update_3d(h, z, H, R, ESKF_NIS_LIMIT_3D_3SIGMA, result);
}

void eskf_update_mag(ESKF_Handle *h,
                     const eskf_float_t mag_m[3],
                     eskf_float_t R_mag,
                     ESKF_InnovResult *result) {
    if (!h || !h->initialized || !mag_m || R_mag <= 0.0) return;

    /* Normalize measured magnetic field */
    eskf_float_t mag_norm[3];
    eskf_vec3_copy(mag_m, mag_norm);
    if (eskf_vec3_normalize(mag_norm) < ESKF_EPSILON) return;

    {
        eskf_float_t residual;
        eskf_float_t H[15];
        memset(H, 0, sizeof(H));
        if (!eskf_model_magnetic_heading(
                h->state.q, mag_norm, h->mag_ref,
                &residual, &H[ESKF_IDX_DTHETA])) return;
        _measurement_update_1d(
            h, residual, H, R_mag, ESKF_NIS_LIMIT_1D_3SIGMA, result
        );
    }
}

void eskf_update_heading(ESKF_Handle *h,
                         eskf_float_t heading_ned_rad,
                         eskf_float_t R_heading,
                         ESKF_InnovResult *result) {
    eskf_float_t current_heading;
    eskf_float_t H[15];
    if (result != NULL) memset(result, 0, sizeof(*result));
    if (!h || !h->initialized || !isfinite(heading_ned_rad) || R_heading <= 0.0) return;

    memset(H, 0, sizeof(H));
    if (!eskf_model_heading(
            h->state.q, &current_heading, &H[ESKF_IDX_DTHETA])) return;
    _measurement_update_1d(
        h,
        _wrap_pi(heading_ned_rad - current_heading),
        H,
        R_heading,
        ESKF_NIS_LIMIT_1D_3SIGMA,
        result
    );
}

void eskf_update_baro(ESKF_Handle *h,
                      eskf_float_t baro_height_m,
                      eskf_float_t R_baro,
                      ESKF_InnovResult *result) {
    if (!h || !h->initialized || R_baro <= 0.0) return;

    /* Convert baro (up-positive) to NED (down-positive) */
    eskf_float_t meas_down = -baro_height_m;

    /* Residual: z = measurement - prediction */
    eskf_float_t z = meas_down - h->state.p[2];

    /* Jacobian H (1x15): H[8] = 1.0 for p_D */
    eskf_float_t H[15];
    memset(H, 0, sizeof(H));
    H[ESKF_IDX_DP + 2] = 1.0;  /* Index 8 */

    _measurement_update_1d(h, z, H, R_baro, ESKF_NIS_LIMIT_1D_3SIGMA, result);
}

void eskf_update_static_constraint(ESKF_Handle *h, eskf_float_t R_zupt) {
    if (!h || !h->initialized || R_zupt <= 0.0) return;

    /* ZUPT: velocity = [0, 0, 0] */
    eskf_float_t z[3];
    z[0] = 0.0 - h->state.v[0];
    z[1] = 0.0 - h->state.v[1];
    z[2] = 0.0 - h->state.v[2];

    /* Jacobian H (3x15): H selects velocity error at indices 3-5 */
    eskf_float_t H[3][15];
    memset(H, 0, sizeof(H));
    H[0][3] = 1.0;
    H[1][4] = 1.0;
    H[2][5] = 1.0;

    /* Measurement noise R (3x3 diagonal) */
    eskf_float_t R[3][3];
    eskf_mat3_zero(R);
    R[0][0] = R_zupt;
    R[1][1] = R_zupt;
    R[2][2] = R_zupt;

    /* No gating for ZUPT (gate=0, result=NULL) */
    _measurement_update_3d(h, z, H, R, 0.0, NULL);
}

void eskf_reset_navigation(ESKF_Handle *h,
                           const eskf_float_t position_ned_m[3],
                           const eskf_float_t velocity_ned_m_s[3],
                           eskf_float_t position_variance_m2,
                           eskf_float_t velocity_variance_m2_s2) {
    int i;
    int j;
    if (!h || !h->initialized || !position_ned_m || !velocity_ned_m_s
        || position_variance_m2 <= 0.0 || velocity_variance_m2_s2 <= 0.0) return;

    eskf_vec3_copy(position_ned_m, h->state.p);
    eskf_vec3_copy(velocity_ned_m_s, h->state.v);
    for (i = ESKF_IDX_DV; i < ESKF_IDX_DP + 3; ++i) {
        for (j = 0; j < ESKF_ERROR_STATE_DIM; ++j) {
            h->P[i][j] = 0.0;
            h->P[j][i] = 0.0;
        }
    }
    for (i = 0; i < 3; ++i) {
        h->P[ESKF_IDX_DV + i][ESKF_IDX_DV + i] = velocity_variance_m2_s2;
        h->P[ESKF_IDX_DP + i][ESKF_IDX_DP + i] = position_variance_m2;
    }
}

void eskf_reset_position(ESKF_Handle *h,
                         const eskf_float_t position_ned_m[3],
                         eskf_float_t position_variance_m2) {
    int i;
    int j;
    if (!h || !h->initialized || !position_ned_m || position_variance_m2 <= 0.0) return;

    eskf_vec3_copy(position_ned_m, h->state.p);
    for (i = ESKF_IDX_DP; i < ESKF_IDX_DP + 3; ++i) {
        for (j = 0; j < ESKF_ERROR_STATE_DIM; ++j) {
            h->P[i][j] = 0.0;
            h->P[j][i] = 0.0;
        }
    }
    for (i = 0; i < 3; ++i) {
        h->P[ESKF_IDX_DP + i][ESKF_IDX_DP + i] = position_variance_m2;
    }
}

void eskf_reset_velocity(ESKF_Handle *h,
                         const eskf_float_t velocity_ned_m_s[3],
                         eskf_float_t velocity_variance_m2_s2) {
    int i;
    int j;
    if (!h || !h->initialized || !velocity_ned_m_s
        || velocity_variance_m2_s2 <= 0.0) return;

    eskf_vec3_copy(velocity_ned_m_s, h->state.v);
    for (i = ESKF_IDX_DV; i < ESKF_IDX_DV + 3; ++i) {
        for (j = 0; j < ESKF_ERROR_STATE_DIM; ++j) {
            h->P[i][j] = 0.0;
            h->P[j][i] = 0.0;
        }
    }
    for (i = 0; i < 3; ++i) {
        h->P[ESKF_IDX_DV + i][ESKF_IDX_DV + i] = velocity_variance_m2_s2;
    }
}

/* ============================================================================
 * Calibration / Alignment
 * ============================================================================ */

bool eskf_align_static_tilt(ESKF_Handle *h,
                            const eskf_float_t acceleration_mean_m_s2[3]) {
    eskf_float_t R_nb[3][3];
    eskf_float_t q[4];
    eskf_float_t horizontal;
    eskf_float_t roll;
    eskf_float_t pitch;
    eskf_float_t yaw;
    if (!h || !h->initialized || !acceleration_mean_m_s2) return false;
    if (!isfinite(acceleration_mean_m_s2[0])
        || !isfinite(acceleration_mean_m_s2[1])
        || !isfinite(acceleration_mean_m_s2[2])) return false;

    horizontal = hypot(acceleration_mean_m_s2[1], acceleration_mean_m_s2[2]);
    if (hypot(acceleration_mean_m_s2[0], horizontal) < ESKF_EPSILON) return false;
    roll = atan2(-acceleration_mean_m_s2[1], -acceleration_mean_m_s2[2]);
    pitch = atan2(acceleration_mean_m_s2[0], horizontal);
    eskf_quat_to_rot_mat3(h->state.q, R_nb);
    yaw = atan2(R_nb[1][0], R_nb[0][0]);
    _quaternion_from_euler(roll, pitch, yaw, q);
    eskf_quat_copy(q, h->state.q);
    return true;
}

bool eskf_align_static_heading(ESKF_Handle *h,
                               const eskf_float_t magnetic_mean[3]) {
    eskf_float_t magnetic_body[3];
    eskf_float_t magnetic_ned[3];
    eskf_float_t R_nb[3][3];
    eskf_float_t q[4];
    eskf_float_t roll;
    eskf_float_t pitch;
    eskf_float_t yaw;
    eskf_float_t residual;
    if (!h || !h->initialized || !magnetic_mean) return false;
    if (!isfinite(magnetic_mean[0]) || !isfinite(magnetic_mean[1])
        || !isfinite(magnetic_mean[2])) return false;
    eskf_vec3_copy(magnetic_mean, magnetic_body);
    if (eskf_vec3_normalize(magnetic_body) < ESKF_EPSILON) return false;
    eskf_quat_to_rot_mat3(h->state.q, R_nb);
    eskf_mat3_mul_vec3(R_nb, magnetic_body, magnetic_ned);
    if (hypot(magnetic_ned[0], magnetic_ned[1]) < ESKF_EPSILON
        || hypot(h->mag_ref[0], h->mag_ref[1]) < ESKF_EPSILON) return false;

    roll = atan2(R_nb[2][1], R_nb[2][2]);
    pitch = asin(fmax(-1.0, fmin(1.0, -R_nb[2][0])));
    yaw = atan2(R_nb[1][0], R_nb[0][0]);
    residual = _wrap_pi(
        atan2(h->mag_ref[1], h->mag_ref[0])
        - atan2(magnetic_ned[1], magnetic_ned[0])
    );
    _quaternion_from_euler(roll, pitch, _wrap_pi(yaw + residual), q);
    eskf_quat_copy(q, h->state.q);
    return true;
}

bool eskf_reset_attitude_covariance(
    ESKF_Handle *h,
    const eskf_float_t attitude_variance_rad2[3]
) {
    int axis;
    int index;
    if (!h || !h->initialized || !attitude_variance_rad2) return false;
    for (axis = 0; axis < 3; ++axis) {
        if (!isfinite(attitude_variance_rad2[axis])
            || attitude_variance_rad2[axis] <= 0.0) return false;
    }
    for (axis = 0; axis < 3; ++axis) {
        for (index = 0; index < ESKF_ERROR_STATE_DIM; ++index) {
            h->P[axis][index] = 0.0;
            h->P[index][axis] = 0.0;
        }
        h->P[axis][axis] = attitude_variance_rad2[axis];
    }
    return true;
}

void eskf_align_static_biases(ESKF_Handle *h,
                               const eskf_float_t (*acc_buf)[3],
                               const eskf_float_t (*gyr_buf)[3],
                               int n_samples) {
    if (!h || !acc_buf || !gyr_buf || n_samples <= 0) return;

    /* Compute mean of IMU readings */
    eskf_float_t acc_mean[3] = {0.0, 0.0, 0.0};
    eskf_float_t gyr_mean[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < n_samples; i++) {
        acc_mean[0] += acc_buf[i][0];
        acc_mean[1] += acc_buf[i][1];
        acc_mean[2] += acc_buf[i][2];
        gyr_mean[0] += gyr_buf[i][0];
        gyr_mean[1] += gyr_buf[i][1];
        gyr_mean[2] += gyr_buf[i][2];
    }

    eskf_float_t inv_n = 1.0 / (eskf_float_t)n_samples;
    eskf_vec3_scale(acc_mean, inv_n, acc_mean);
    eskf_vec3_scale(gyr_mean, inv_n, gyr_mean);

    eskf_align_static_bias_means(h, acc_mean, gyr_mean);
}

void eskf_align_static_bias_means(ESKF_Handle *h,
                                  const eskf_float_t acceleration_mean_m_s2[3],
                                  const eskf_float_t angular_rate_mean_rad_s[3]) {
    eskf_float_t R_nb[3][3];
    eskf_float_t expected_specific_force_body[3];
    int i;
    int j;
    if (!h || !h->initialized || !acceleration_mean_m_s2 || !angular_rate_mean_rad_s) return;

    eskf_vec3_copy(angular_rate_mean_rad_s, h->state.gb);
    eskf_quat_to_rot_mat3(h->state.q, R_nb);
    for (i = 0; i < 3; ++i) {
        expected_specific_force_body[i] = -R_nb[2][i] * ESKF_GRAVITY;
        h->state.ab[i] = acceleration_mean_m_s2[i] - expected_specific_force_body[i];
    }

    /*
     * A stationary mean directly observes gyro bias, but a single gravity
     * direction cannot separate horizontal accelerometer bias from a small
     * tilt error.  Keep the accelerometer-bias covariance broad enough for
     * later GNSS-aided motion to correct it; marking all six biases equally
     * certain makes the filter inconsistent after a one-pose alignment.
     */
    const eskf_float_t P_accel_bias = 4e-2; /* conservative 0.2 m/s^2 startup prior */
    const eskf_float_t P_gyro_bias = 1e-4;  /* (0.01 rad/s)^2 */
    h->P[9][9]   = P_accel_bias;
    h->P[10][10] = P_accel_bias;
    h->P[11][11] = P_accel_bias;
    h->P[12][12] = P_gyro_bias;
    h->P[13][13] = P_gyro_bias;
    h->P[14][14] = P_gyro_bias;

    /* Zero cross-correlations with biases */
    for (i = 0; i < 9; i++) {
        for (j = 9; j < 15; j++) {
            h->P[i][j] = 0.0;
            h->P[j][i] = 0.0;
        }
    }
}

/* ============================================================================
 * State Access
 * ============================================================================ */

void eskf_get_state(const ESKF_Handle *h,
                    eskf_float_t *p,
                    eskf_float_t *v,
                    eskf_float_t *q,
                    eskf_float_t *ab,
                    eskf_float_t *gb) {
    if (!h) return;

    if (p)  { p[0] = h->state.p[0]; p[1] = h->state.p[1]; p[2] = h->state.p[2]; }
    if (v)  { v[0] = h->state.v[0]; v[1] = h->state.v[1]; v[2] = h->state.v[2]; }
    if (q)  { q[0] = h->state.q[0]; q[1] = h->state.q[1]; q[2] = h->state.q[2]; q[3] = h->state.q[3]; }
    if (ab) { ab[0] = h->state.ab[0]; ab[1] = h->state.ab[1]; ab[2] = h->state.ab[2]; }
    if (gb) { gb[0] = h->state.gb[0]; gb[1] = h->state.gb[1]; gb[2] = h->state.gb[2]; }
}

void eskf_get_covariance_diag(const ESKF_Handle *h, eskf_float_t P_diag[15]) {
    if (!h || !P_diag) return;

    for (int i = 0; i < 15; i++) {
        P_diag[i] = h->P[i][i];
    }
}
