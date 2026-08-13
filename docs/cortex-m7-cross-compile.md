# Cortex-M7 cross-compile preflight

## Scope

This is a pre-hardware build and layout check for the portable library on the
FCOne Cortex-M7 profile. It establishes that the source compiles under the
declared target ABI and makes the library's own dependencies visible. It is
not an FCOne firmware image, WCET measurement, stack measurement, or target
runtime qualification.

The reproducible entry point is:

```bash
python3 validation/run_cortex_m7_cross_compile.py \
  --out build/cortex-m7-cross/result.json
```

The generated compact evidence is
[`cortex_m7_cross_compile_v1.json`](../validation/public/cortex_m7_cross_compile_v1.json).
It records source hashes, the exact compiler version, flags, layouts, section
sums, and external dependencies without retaining generated objects.

The `git_commit` field identifies the reviewed source revision whose tree
produced the recorded source manifest. The unit test
`tests/test_cortex_m7_cross_compile.py` rejects a checked-in report when that
manifest no longer matches either the current portable source set or the named
commit, so evidence must be regenerated after any portable source or header
change. The runner also refuses to generate evidence while a portable source,
header, or the script itself has uncommitted changes.

## Method

The script compiles all eight portable C translation units with
`arm-none-eabi-gcc 14.2.1`, `-mcpu=cortex-m7`, Thumb-2, hard-float ABI, and
`-mfpu=fpv5-d16`. Every compile uses strict C99 and warnings-as-errors.

It evaluates two explicit profiles:

- `double`: reviewed public default;
- `float`: `AERAKIA_ESKF_CORE_USE_FLOAT=1`, an evaluation candidate only.

Each profile is archived and also joined with a relocatable link. The latter
resolves cross-file library calls before `nm -u` runs, so the report lists only
what a future FCOne image must supply through startup/application libraries.
The float profile additionally compiles `eskf.c`, `eskf_math.c`, and
`eskf_models.c` with `-Wdouble-promotion -Werror=double-promotion`.

## Result

| Measure | Reviewed double | Float candidate | Interpretation |
| --- | ---: | ---: | --- |
| Portable `.text` section sum | 28,780 B | 27,580 B | Object-section sum only; the full eight-unit float candidate is 1,200 B (4.2%) smaller before final linking. |
| `AerakiaEskf` layout | 2,928 B | 1,840 B | 1,088 B (37.2%) smaller. |
| `ESKF_Handle` layout | 2,136 B | 1,068 B | Core state/covariance is exactly half-sized. |
| `AerakiaNavigationEstimate` layout | 504 B | 504 B | Public diagnostic/output layout is intentionally unchanged. |
| Float-core promotion check | N/A | pass | The ESKF core has no compiler-detected float-to-double arithmetic promotion. |

The translation-unit set includes the multi-pose static IMU calibration
calculator, preventing the target preflight from silently omitting a newly
added public algorithm source.

That calculator deliberately remains double-precision in both profiles: it is
a bounded pre-arm/offline operation, not the 100 Hz propagation path. Its
`fabs`/`sqrt` dependencies are therefore visible in the float-profile report.
This does not authorize running it in a time-critical task; FCOne validates and
applies an accepted result before the first estimator sample.

The preflight also exposed and corrected a material problem in the previous
float candidate: `eskf.c`, `eskf_math.c`, and `eskf_models.c` used double
`sin/cos/atan2/...` functions and unsuffixed arithmetic literals even in the
float profile. The core now uses type-matched functions and scalar literals.
The final float-core dependency list contains only `*f` libm functions. Some
whole-library compiler helpers remain because the public adapter and barometer
supervisor intentionally use double timestamp/accumulation paths; this is
reported rather than hidden and requires target measurement.

The corrected float profile also passed the existing byte-identical 20-second,
400 Hz double-versus-float host comparison. Maximum float-minus-double state
differences were `0.0011745 deg`, `0.0039432 m`, and `0.0006704 m/s` for clean
motion, and `0.0000974 deg`, `0.0000258 m`, and `0.0000125 m/s` during the
cold-start navigation-outage scenario. Both tracks stayed 100% healthy. This
is numerical differential evidence, not an argument to promote float yet.

## Consequences for FCOne v2

1. Do not assume the default double profile is viable at target rate. The
   STM32H7 FPv5-D16 unit is single precision; the default profile can require
   software double math and must be benchmarked instead of inferred from this
   compile result.
2. Do not select float only because it compiles or uses less static storage.
   The board build must measure WCET, stack high-water mark, final Flash/RAM,
   numerical health, scheduler jitter, and logging load for the *entire shadow
   image* (incumbent + ESKF + Mahony + logs).
3. Keep the portable default as double until those target measurements and a
   reviewed profile-selection decision exist. The float build is now an honest
   candidate suitable for that comparison.

## Explicit limits

The preflight does not link startup code, HAL, FreeRTOS, private FCOne adapters,
driver/DMA buffers, logging, or an interrupt vector table. Linker garbage
collection can change final section sizes. It also does not measure target
execution time, cache/DMA coherence, exception behavior, watchdog recovery,
or long-run numerical stability. Those remain board and integration gates.
