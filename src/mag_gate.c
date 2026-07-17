/**
 * @file mag_gate.c
 * @brief Re-entrant magnetic-field anomaly gate implementation.
 */

#include <aerakia/mag_gate.h>

#include <math.h>
#include <stddef.h>

static bool config_is_valid(const AerakiaMagGateConfig *config)
{
    return config != NULL
        && config->ema_alpha > 0.0f
        && config->ema_alpha <= 1.0f
        && config->absolute_threshold_ut > 0.0f
        && config->relative_threshold >= 0.0f
        && config->confirmation_samples > 0U;
}

void aerakia_mag_gate_default_config(AerakiaMagGateConfig *config)
{
    if (config == NULL) {
        return;
    }

    config->ema_alpha = 0.2f;
    config->absolute_threshold_ut = 6.0f;
    config->relative_threshold = 0.12f;
    config->confirmation_samples = 2U;
    config->rejection_samples = 3U;
    config->recovery_samples = 5U;
}

void aerakia_mag_gate_init(AerakiaMagGate *gate, const AerakiaMagGateConfig *config)
{
    AerakiaMagGateConfig defaults;

    if (gate == NULL) {
        return;
    }

    aerakia_mag_gate_default_config(&defaults);
    gate->config = config_is_valid(config) ? *config : defaults;
    if (gate->config.recovery_samples == 0U) {
        gate->config.recovery_samples = defaults.recovery_samples;
    }
    gate->magnitude_ema_ut = 0.0f;
    gate->consecutive_anomalies = 0U;
    gate->consecutive_recoveries = 0U;
    gate->rejection_remaining = 0U;
    gate->disturbed = false;
    gate->initialized = false;
    gate->enabled = true;
}

void aerakia_mag_gate_set_enabled(AerakiaMagGate *gate, bool enabled)
{
    if (gate != NULL) {
        gate->enabled = enabled;
    }
}

bool aerakia_mag_gate_accept(
    AerakiaMagGate *gate,
    float mx_ut,
    float my_ut,
    float mz_ut
)
{
    float magnitude;
    float difference;
    float threshold;
    float relative_threshold;

    if (gate == NULL) {
        return false;
    }
    if (!gate->enabled) {
        return true;
    }
    if (!isfinite(mx_ut) || !isfinite(my_ut) || !isfinite(mz_ut)) {
        return false;
    }

    magnitude = sqrtf(mx_ut * mx_ut + my_ut * my_ut + mz_ut * mz_ut);
    if (!(magnitude > 0.0f)) {
        return false;
    }

    if (!gate->initialized) {
        gate->magnitude_ema_ut = magnitude;
        gate->initialized = true;
        return true;
    }

    difference = fabsf(magnitude - gate->magnitude_ema_ut);
    relative_threshold = gate->config.relative_threshold * gate->magnitude_ema_ut;
    threshold = fmaxf(gate->config.absolute_threshold_ut, relative_threshold);

    if (gate->disturbed) {
        if (gate->rejection_remaining > 0U) {
            gate->rejection_remaining--;
            return false;
        }
        if (difference > threshold) {
            gate->consecutive_recoveries = 0U;
            return false;
        }
        gate->consecutive_recoveries++;
        if (gate->consecutive_recoveries < gate->config.recovery_samples) return false;
        gate->disturbed = false;
        gate->consecutive_recoveries = 0U;
        gate->consecutive_anomalies = 0U;
        gate->magnitude_ema_ut = gate->config.ema_alpha * magnitude
            + (1.0f - gate->config.ema_alpha) * gate->magnitude_ema_ut;
        return true;
    }

    if (difference > threshold) {
        gate->consecutive_anomalies++;
        if (gate->consecutive_anomalies >= gate->config.confirmation_samples) {
            gate->consecutive_anomalies = 0U;
            gate->disturbed = true;
            gate->rejection_remaining = gate->config.rejection_samples > 0U
                ? gate->config.rejection_samples - 1U
                : 0U;
            return false;
        }
        return true;
    }

    gate->consecutive_anomalies = 0U;
    gate->magnitude_ema_ut = gate->config.ema_alpha * magnitude
        + (1.0f - gate->config.ema_alpha) * gate->magnitude_ema_ut;
    return true;
}
