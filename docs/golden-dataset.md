# Golden dataset format

The offline analysis tools use a normalized CSV representation so estimator work is independent of the device that recorded the data.

The converter accepts a text capture with a required `line` column and either `host_ts_us` or `ts_sec`. Each line may contain one or more hardware-neutral `[IMUCSV]` records. It emits `golden_imu.csv` with timestamps, IMU values, attitude outputs, and diagnostic fields found in the input. Missing optional values remain empty.

At minimum, timing analysis expects:

| Column | Unit | Meaning |
| --- | --- | --- |
| `ts_us` | µs | Device/sample timestamp |
| `dt_us` | µs | Sample interval |
| `seq` | count | Monotonic sample sequence |

Estimator analyses additionally use columns such as `acc_*`, `gyr_*`, `mag_*`, quaternion/Euler outputs, and gravity estimates when present. Run `python simulation/tools/analyze_golden_imu.py --help` for analysis options.

The file format is a public adapter boundary: a private application may produce the same CSV directly or use an internal recorder, without adding device code or binary transport details to this repository.
