# Coordinate conventions

Aerakia algorithms use the following conventions:

| Quantity | Convention |
| --- | --- |
| Navigation frame | NED: North, East, Down |
| Body frame | FRD: Forward, Right, Down |
| Quaternion | Scalar first: `[w, x, y, z]` |
| Quaternion direction | Body to navigation frame |
| Accelerometer | Specific force in body frame, m/s² |
| Gyroscope | Body angular rate, rad/s |
| Position | `[north, east, down]`, m |
| Velocity | `[north, east, down]`, m/s |
| Barometric height | Up-positive, m; converted internally to Down |
| Magnetic reference | Normalized vector in NED |

At rest, level, and with an identity quaternion, the accelerometer input is approximately `[0, 0, -9.80665]` m/s². The filter adds the NED gravity vector `[0, 0, +9.80665]` m/s² during prediction.

Callers are responsible for axis remapping, scale conversion, calibration, timestamp validation, and measurement-quality checks before invoking the estimator.
