/**
 * @file mag_gate.h
 * @brief Re-entrant magnetic-field anomaly gate.
 */

#ifndef AERAKIA_MAG_GATE_H
#define AERAKIA_MAG_GATE_H

#include <stdbool.h>

typedef struct {
    float ema_alpha;
    float absolute_threshold_ut;
    float relative_threshold;
    unsigned int confirmation_samples;
    unsigned int rejection_samples;
} AerakiaMagGateConfig;

typedef struct {
    AerakiaMagGateConfig config;
    float magnitude_ema_ut;
    unsigned int consecutive_anomalies;
    unsigned int rejection_remaining;
    bool initialized;
    bool enabled;
} AerakiaMagGate;

/** Fill a configuration with conservative defaults. */
void aerakia_mag_gate_default_config(AerakiaMagGateConfig *config);

/** Initialize or reset a gate. A NULL config selects the defaults. */
void aerakia_mag_gate_init(AerakiaMagGate *gate, const AerakiaMagGateConfig *config);

/** Enable or bypass anomaly rejection without discarding the learned baseline. */
void aerakia_mag_gate_set_enabled(AerakiaMagGate *gate, bool enabled);

/**
 * Decide whether a magnetic sample should be used.
 *
 * @param gate       Caller-owned gate state.
 * @param mx_ut      Body X magnetic field in microtesla.
 * @param my_ut      Body Y magnetic field in microtesla.
 * @param mz_ut      Body Z magnetic field in microtesla.
 * @return true when the sample is suitable for fusion.
 */
bool aerakia_mag_gate_accept(
    AerakiaMagGate *gate,
    float mx_ut,
    float my_ut,
    float mz_ut
);

#endif
