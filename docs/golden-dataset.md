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

## PX4 ULog replay extension

The ULog converter writes the same required IMU/timing columns plus optional hardware-neutral aiding/reference fields:

| Group | Representative columns | Use |
| --- | --- | --- |
| Sparse magnetic input | `mag_valid`, `mag_update` | Fuse only newly published valid samples |
| Attitude reference | `ref_q_*`, `roll_mdeg`, `pitch_mdeg`, `yaw_mdeg` | PX4 engineering reference |
| Reference reset | `ref_attitude_reset_counter`, `ref_attitude_reset_event`, `ref_delta_q_reset_*` | Segment diagnostics without interpolating across resets |
| Relative navigation | `position_ref_valid`, `ref_position_*`, `ref_velocity_*` | PX4 local reference with the first valid sample as origin |
| GPS aiding | `position_update`, `gps_position_*`, `gps_velocity_*`, variance columns | Relative NED aiding; absolute coordinates are discarded |
| Barometer/static | `baro_update`, `baro_height_up_m`, `static_hint` | Explicit lower-rate aiding and application assertion |

Quaternion interpolation normalizes sign-equivalent samples and never interpolates across a reset-counter boundary. Raw ULogs are not part of the public dataset contract and should remain outside the repository unless publication is separately approved.
