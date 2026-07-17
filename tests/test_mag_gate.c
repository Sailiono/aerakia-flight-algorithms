#include <aerakia/mag_gate.h>

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

static void test_default_sequence(void)
{
    AerakiaMagGate gate;

    aerakia_mag_gate_init(&gate, NULL);
    check_true(aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "baseline accepted");
    check_true(aerakia_mag_gate_accept(&gate, 51.0f, 0.0f, 0.0f), "normal variation accepted");
    check_true(aerakia_mag_gate_accept(&gate, 80.0f, 0.0f, 0.0f), "first anomaly waits for confirmation");
    check_true(!aerakia_mag_gate_accept(&gate, 80.0f, 0.0f, 0.0f), "confirmed anomaly rejected");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "rejection window sample one");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "rejection window sample two");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "recovery confirmation one");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "recovery confirmation two");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "recovery confirmation three");
    check_true(!aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "recovery confirmation four");
    check_true(aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "gate recovers after stable field");
}

static void test_persistent_anomaly_stays_latched(void)
{
    AerakiaMagGate gate;
    int index;

    aerakia_mag_gate_init(&gate, NULL);
    check_true(aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "persistent baseline");
    check_true(aerakia_mag_gate_accept(&gate, 80.0f, 0.0f, 0.0f), "persistent first anomaly");
    check_true(!aerakia_mag_gate_accept(&gate, 80.0f, 0.0f, 0.0f), "persistent anomaly latches");
    for (index = 0; index < 50; ++index) {
        check_true(
            !aerakia_mag_gate_accept(&gate, 80.0f, 0.0f, 0.0f),
            "persistent anomaly remains rejected"
        );
    }
}

static void test_bypass_and_invalid_input(void)
{
    AerakiaMagGate gate;

    aerakia_mag_gate_init(&gate, NULL);
    check_true(aerakia_mag_gate_accept(&gate, 50.0f, 0.0f, 0.0f), "invalid-input baseline");
    check_true(!aerakia_mag_gate_accept(&gate, NAN, 0.0f, 0.0f), "NaN sample rejected");

    aerakia_mag_gate_set_enabled(&gate, false);
    check_true(aerakia_mag_gate_accept(&gate, NAN, 0.0f, 0.0f), "disabled gate is a transparent bypass");
}

static void test_instances_are_independent(void)
{
    AerakiaMagGate first;
    AerakiaMagGate second;

    aerakia_mag_gate_init(&first, NULL);
    aerakia_mag_gate_init(&second, NULL);
    (void)aerakia_mag_gate_accept(&first, 50.0f, 0.0f, 0.0f);
    (void)aerakia_mag_gate_accept(&second, 100.0f, 0.0f, 0.0f);

    check_true(aerakia_mag_gate_accept(&first, 51.0f, 0.0f, 0.0f), "first instance keeps its baseline");
    check_true(aerakia_mag_gate_accept(&second, 101.0f, 0.0f, 0.0f), "second instance keeps its baseline");
}

static void test_zero_recovery_uses_compatible_default(void)
{
    AerakiaMagGate gate;
    AerakiaMagGateConfig config;

    aerakia_mag_gate_default_config(&config);
    config.absolute_threshold_ut = 9.0f;
    config.recovery_samples = 0U;
    aerakia_mag_gate_init(&gate, &config);
    check_true(fabsf(gate.config.absolute_threshold_ut - 9.0f) < 1.0e-6f,
               "legacy config fields are preserved");
    check_true(gate.config.recovery_samples == 5U, "zero recovery selects compatible default");
}

int main(void)
{
    test_default_sequence();
    test_bypass_and_invalid_input();
    test_instances_are_independent();
    test_zero_recovery_uses_compatible_default();
    test_persistent_anomaly_stays_latched();

    if (failures != 0) {
        fprintf(stderr, "%d magnetic-gate assertion(s) failed\n", failures);
        return 1;
    }

    puts("Magnetic gate tests passed");
    return 0;
}
