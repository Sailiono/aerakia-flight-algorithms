/**
 * @file eskf_internal.h
 * @brief Private core helpers shared with host-only validation experiments.
 *
 * This header is intentionally outside the public include tree.  It exposes
 * the exact nominal-state injection and covariance-reset transaction used by
 * the core measurement updates so a host replay can test an atomic lagged
 * correction without duplicating ESKF mathematics.
 */

#ifndef AERAKIA_ESKF_INTERNAL_H
#define AERAKIA_ESKF_INTERNAL_H

#include <aerakia/eskf.h>

/** Apply one complete ESKF error-state injection and reset covariance. */
void eskf_internal_apply_error_state(
    ESKF_Handle *handle,
    const eskf_float_t error_state[ESKF_ERROR_STATE_DIM]
);

#endif /* AERAKIA_ESKF_INTERNAL_H */
